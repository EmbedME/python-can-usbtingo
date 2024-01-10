# This file is part of the python-can-usbtingo project, a plugin for
# python-can to support the USBtingo USB to CAN-FD interface.
#
# Copyright(c) 2023-2024 Thomas Fischl (https://www.fischl.de)
# 
# python-can-usbtingo is free software: you can redistribute it and/or modify
# it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# python-can-usbtingo is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU LESSER GENERAL PUBLIC LICENSE for more details.
#
# You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
# along with python-can-usbtingo.  If not, see <http://www.gnu.org/licenses/>

from typing import Any, Optional, Tuple
from can import BusABC, Message, typechecking, CanTimeoutError, CanOperationError, CanInitializationError, BusState
from can.util import len2dlc, dlc2len
import platform
import usb1
import struct
import threading
import time
import queue
import zipfile
import logging

log = logging.getLogger(__name__)

class USBtingoBus(BusABC):

    """
    CAN bus implemented for USBtingo, a USB to CAN-FD interface.
    https://www.fischl.de/usbtingo/
    """

    USB_VID = 0x1FC9
    USB_PID = 0x8320
    
    TIMESTAMP_FACTOR = 100000

    CMD_GET_DEVICEINFO = 0x03
    CMD_SET_PROTOCOL = 0x04
    CMD_SET_BAUDRATE = 0x05
    CMD_SET_MODE = 0x06
    CMD_FILTER_DISABLE_ALL = 0x20
    CMD_FILTER_SET_STD = 0x21
    CMD_FILTER_SET_EXT = 0x22    
    CMD_MCAN_REG_READ = 0x30
    CMD_MCAN_REG_WRITE = 0x31
    CMD_LOGIC_SETCONFIG = 0x40
    CMD_LOGIC_GETTXERRORS = 0x41

    RXMSG_TYPE_CAN = 0x81
    RXMSG_TYPE_TXEVENT = 0x82

    PROTOCOL_CAN_20 = 0
    PROTOCOL_CAN_FD = 1
    PROTOCOL_CAN_FD_NONISO = 2

    MODE_OFF = 0
    MODE_ACTIVE = 1
    MODE_LISTENONLY = 2

    def __init__(self, channel=None, can_filters=None, **kwargs):

        """
        :param str channel:
            The can interface name, corresponds to the USB serial number.
            
        :param list can_filters:
            See :meth:`can.BusABC.set_filters`.    

        :param int bitrate:
            Bitrate of channel in bit/s

        :param int data_bitrate:
            Bitrate to use for data phase in CAN FD.
            Defaults to arbitration bitrate.
            
        :param bool fd:
            If CAN-FD should be enabled.
            
        :param bool receive_own_messages:
            If messages transmitted should also be received back.

        :param str channel:
            USB serial number used to identify device.
            Defaults to channel.
            
        :param can.bus.BusState state:
            BusState of the channel.
            Default is ACTIVE.            
        """
        
        self.bitrate = kwargs.get("bitrate", 500000)
        self.data_bitrate = kwargs.get("data_bitrate", self.bitrate)
        self.is_fd = kwargs.get("fd", False)
        self.serialnumber = kwargs.get("serial", channel)
        self.cprotocol = kwargs.get("protocol", self.PROTOCOL_CAN_FD if self.is_fd else self.PROTOCOL_CAN_20)
        self.receive_own_messages = kwargs.get("receive_own_messages", False)
        state = kwargs.get("state", BusState.ACTIVE)

        if state not in [BusState.ACTIVE, BusState.PASSIVE]:
            raise ValueError("BusState must be Active or Passive")
        mode = self.MODE_ACTIVE if state == BusState.ACTIVE else self.MODE_LISTENONLY
        self._state = state

        self.ctx = usb1.USBContext()
        self.ctx.open()
        self.usbdev = self.get_usb_device(self.serialnumber)
        if self.usbdev is None:
            raise CanInitializationError("USBtingo not found")

        self.usbhandle = self.usbdev.open()
        self.usbhandle.claimInterface(0)        
        
        self.running = True
        self.txmm = 0
        self.txconfirmation_queue = queue.Queue()
        self.tx_queue = queue.Queue()
        self.rx_queue = queue.Queue()

        flags = 0
        if self.cprotocol > 0:
            flags = flags | (1 << 0) #brse

        self.deviceinfo_read()
        self.command_write(self.CMD_SET_MODE, 0)
        self.command_write(self.CMD_SET_PROTOCOL, self.cprotocol | (flags << 8))
        self.command_write(self.CMD_SET_BAUDRATE, 0, 0, struct.pack("<I", self.bitrate))
        self.command_write(self.CMD_SET_BAUDRATE, 1, 0, struct.pack("<I", self.data_bitrate))
        self.command_write(self.CMD_SET_MODE, mode)
        self.timestamp_offset = time.time()
        self.recordingActive = False

        self.eventthread = USBtingoUSBEventHandler(self)
        self.eventthread.start()

        th2in = usb1.USBTransferHelper()
        th2in.setEventCallback(usb1.TRANSFER_COMPLETED, self.usbtransfer_ep2in_callback)
        self.ep2in_transfer = []
        for _ in range(4):
            t = self.usbhandle.getTransfer()
            t.setBulk(0x82, 1400 * 512, th2in)
            self.ep2in_transfer.append(t)

        ep3in_nrofpackets = 256
        if platform.machine().startswith('arm'):
            # On most ARM systems URB max length is 16kB, use it as transfer length limit to prevent
            # unnecessary kernel activity, especially with short transfers.
            ep3in_nrofpackets = 32

        th3in = usb1.USBTransferHelper()
        th3in.setEventCallback(usb1.TRANSFER_COMPLETED, self.usbtransfer_ep3in_callback)
        self.ep3in_transfer = []
        for _ in range(8192 // ep3in_nrofpackets):
            t = self.usbhandle.getTransfer()
            t.setBulk(0x83, ep3in_nrofpackets * 512, th3in)
            t.submit()        
            self.ep3in_transfer.append(t)

        th3out = usb1.USBTransferHelper()
        th3out.setEventCallback(usb1.TRANSFER_COMPLETED, self.usbtransfer_ep3out_callback)
        self.ep3out_transfer = []
        for _ in range(4):
            t = self.usbhandle.getTransfer()
            t.setBulk(0x3, 128 * 512, th3out)
            t.inuse = False
            self.ep3out_transfer.append(t)

        self.statusreport_listeners = []
        th1in = usb1.USBTransferHelper()
        th1in.setEventCallback(usb1.TRANSFER_COMPLETED, self.usbtransfer_ep1in_callback)
        self.ep1in_transfer = []
        for _ in range(2):
            t = self.usbhandle.getTransfer()
            t.setInterrupt(0x81, 64, th1in)
            t.submit()
            self.ep1in_transfer.append(t)        

        self._is_filtered = False
        super().__init__(
            channel=str(self.deviceid),
            can_filters=can_filters,
            **kwargs,
        )

    def timestamp_convert(self, procts, hwts):
        """
        Generate timestamp from processing- and hardware-timestamp
        :param procts: Processing timestamp (integer)
        :param hwts: Hardware timestamp (integer)
        :return: Calculated timestamp (float)
        """
        d = (procts & 0xF) - (hwts >> 12)
        if d < 0:
            d = d + 0x10
        ts = ((procts - d) << 12) | (hwts & 0xfff)

        return self.timestamp_offset + (ts /  self.TIMESTAMP_FACTOR)

    def get_usb_device(self, serialnumber = None):
        """
        Get USB device matching VID and PID and if given also check the USB serial number.
        :param serialnumber: USB serial number
        :return USB device if found, otherwise None
        """
        for device in self.ctx.getDeviceIterator():
            if device.getVendorID() == self.USB_VID and device.getProductID() == self.USB_PID:
            
                if serialnumber is None:
                    return device

                try:
                    if device.getSerialNumber() == str(serialnumber):
                        return device
                except usb1.USBErrorAccess:
                    pass                

    def shutdown(self) -> None:
        """
        Cleanup and shutdown bus.
        """
        super().shutdown()
        self.recording_stop()
        self.running = False
        self.eventthread.stop()
        self.command_write(self.CMD_SET_MODE, 0)
        self.eventthread.join()
        self.usbdev.close()
        self.ctx.close()            

    def send(self, msg: Message, timeout: Optional[float] = None) -> None:
        """
        Send out given message to CAN bus.
        :param msg: Message to send out
        :param timeout: Timeout in seconds. None blocks, 0 returns immediately
        """
        msg.is_rx = False
        
        if not msg.is_remote_frame:
            msg.dlc = dlc2len(len2dlc(msg.dlc))
            while len(msg.data) < msg.dlc:
                msg.data.append(0x00)        
        
        if timeout == 0 and not self.receive_own_messages:
            self.tx_queue.put_nowait(msg)
        else:
            txconfirmation = TXConfirmation(msg)
            self.tx_queue.put_nowait(txconfirmation)

        for t in self.ep3out_transfer:
            if not t.inuse:
                buffer = self.tx_prepareBuffer()
                if len(buffer) > 0:
                    t.inuse = True
                    t.setBuffer(buffer)
                    t.submit()                        
                    break

        if timeout != 0:
            if not txconfirmation.wait_confirmed(timeout):
                raise CanTimeoutError()


    def _recv_internal(self, timeout: Optional[float]) -> Tuple[Optional[Message], bool]:
        """
        Read message from CAN bus.
        :param timeout: Timeout in seconds. None blocks.
        """
        try:
            if timeout == 0:
                msg = self.rx_queue.get(False)
            else:
                msg = self.rx_queue.get(timeout = timeout)
        except queue.Empty:
            return None, False
            
        return msg, self._is_filtered

    def usbtransfer_ep3in_callback(self, t):
        """
        Complete callback for EP3IN bulk transfers. This is the receiving pipe.
        :param t: USB transfer
        """
        packet = t.getBuffer()[:t.getActualLength()]

        while len(packet) >= 4:
            msgtype, msglength = struct.unpack("<BBxx", packet[:4])

            if msgtype == self.RXMSG_TYPE_CAN:
                procts, msgid, msgtimestamp, msgdlc, msgfilteridx  = struct.unpack("<IIHBB", packet[4:16])

                is_extended_id = (packet[11] & (1 << 6)) != 0
                is_remote_frame = (packet[11] & (1 << 5)) != 0
                error_state_indicator = (packet[11] & (1 << 7)) != 0
                is_fd = (packet[14] & (1 << 5)) != 0
                bitrate_switch = (packet[14] & (1 << 4)) != 0
                
                if is_extended_id:
                    msgid = msgid & 0x1fffffff
                else:
                    msgid = (msgid >> 18) & 0x7ff
                
                length = 0
                dlc = dlc2len(msgdlc & 0xf)
                if is_remote_frame == False:
                    length = dlc
                    
                msg = Message(
                    arbitration_id = msgid,
                    is_extended_id = is_extended_id,
                    is_remote_frame = is_remote_frame,
                    error_state_indicator = error_state_indicator,
                    is_fd = is_fd,
                    bitrate_switch = bitrate_switch,
                    timestamp = self.timestamp_convert(procts, msgtimestamp),
                    data=packet[16:16+length],
                    dlc=dlc)
                self.rx_queue.put_nowait(msg)
            
            if msgtype == self.RXMSG_TYPE_TXEVENT and msglength == 3:
                procts, msgid, msgtimestamp, msgdlc, msgmm  = struct.unpack("<IIHBB", packet[4:16])
                try:
                    txconfirmation = self.txconfirmation_queue.get_nowait()

                    while txconfirmation.txmm != msgmm:
                        
                        # The marker ids do not match. This should not happen. Somewhere a packet has been lost.
                        # We try to solve the problem by estimating where the packet loss occurred (usb or internal fifo).
                        
                        diff = msgmm - txconfirmation.txmm
                        if diff < 0:
                            diff = diff + 0x100

                        if diff > 128:
                            # ignore this incoming tx event.
                            break
                        
                        # try next item in the fifo
                        txconfirmation = self.txconfirmation_queue.get_nowait()
                    
                    if txconfirmation.txmm == msgmm:
                        txconfirmation.message.timestamp = self.timestamp_convert(procts, msgtimestamp)
                        txconfirmation.confirm()
                        if self.receive_own_messages == True:
                            self.rx_queue.put_nowait(txconfirmation.message)
                        
                except queue.Empty as e:
                    # TX confirmation queue empty! We got confirmation, but have no request for it.
                    pass
                

            bytelen = (1 + msglength) * 4
            packet = packet[bytelen:]

        return True

    def usbtransfer_ep3out_callback(self, t):
        """
        Complete callback for EP3OUT USB transfers. This is the sending pipe.
        """
        buffer = self.tx_prepareBuffer()
        if len(buffer) > 0:
            t.setBuffer(buffer)
            return True

        t.inuse = False
        return False

    def tx_prepareBuffer(self):
        """
        Prepare buffer for sending. Collects pending messages and fill buffer for USB transfer.
        """
        try:
            packet = bytearray()
            current_usbpacketremaining = 512
            while len(packet) < 128 * 512:
                
                item = self.tx_queue.get_nowait()
                  
                if isinstance(item, TXConfirmation):
                    msg = item.message
                    get_tx_confirmation = True
                else:
                    msg = item
                    get_tx_confirmation = False

                # add padding bytes if needed to get 32bit aligned
                padlength = (4 - (len(msg.data) % 4)) % 4
                datalength = len(msg.data) + padlength
                    
                # prepare header
                messagetype = 0x01
                messagesize = int(2 + datalength / 4)
                mid = msg.arbitration_id
                if not msg.is_extended_id:
                    mid = mid << 18

                mid = mid | (int(msg.is_remote_frame) << 29)
                mid = mid | (int(msg.is_extended_id) << 30)
                mid = mid | (int(msg.error_state_indicator) << 31)

                mdlc = len2dlc(msg.dlc)
                mdlc = mdlc | int(msg.is_fd) << 5
                mdlc = mdlc | int(msg.bitrate_switch) << 4
                mdlc = mdlc | int(get_tx_confirmation) << 7

                header = struct.pack("<BBxxIxxBB", messagetype, messagesize, mid, mdlc, self.txmm)

                # check if this message fits completely into this usb packet, if not add padding bytes to start new usb packet
                size = len(header) + datalength
                if (size > current_usbpacketremaining):
                    while current_usbpacketremaining > 0:
                        packet.append(0x00)
                        current_usbpacketremaining = current_usbpacketremaining - 1
                    current_usbpacketremaining = 512

                # add header and data to packet
                packet.extend(header)
                packet.extend(bytes(msg.data))
                packet.extend(bytes([0x00] * padlength))
                current_usbpacketremaining = current_usbpacketremaining - size

                # if we have requested tx event, add it to fifo
                if get_tx_confirmation:
                    item.txmm = self.txmm
                    self.txconfirmation_queue.put_nowait(item)

                # increment transmit maker
                self.txmm = (self.txmm + 1) % 0x100
                        
        except queue.Empty as e:
            pass

        return packet

    def command_write(self, command, value = 0, index = 0, data = []):
        """
        Write command to USBtingo device.
        """
        self.usbhandle.controlWrite(usb1.TYPE_VENDOR, command, value, index, data)

    def mcan_register_read(self, address):
        """
        Read register from MCAN module.
        """
        response = self.usbhandle.controlRead(usb1.TYPE_VENDOR, self.CMD_MCAN_REG_READ, address, 0, 4)
        return response[0] | response[1] << 8 | response[2] << 16 | response[3] << 24

    def mcan_register_write(self, address, value):
        """
        Write register of MCAN module.
        :param address: Offset address of MCAN register to write to
        :param value: Value (uint32) to write to given MCAN register
        """
        response = self.usbhandle.controlWrite(usb1.TYPE_VENDOR, self.CMD_MCAN_REG_WRITE, address, 0, struct.pack("<I", value))

    def deviceinfo_read(self):
        """
        Acquire device information
        """
        devinfo = self.usbhandle.controlRead(usb1.TYPE_VENDOR, self.CMD_GET_DEVICEINFO, 0, 0, 8)
        self.firmware_version_minor, self.firmware_version_major, self.hardware_model_id, self.deviceid = struct.unpack("<BBBxI", devinfo)
        

    def recording_start(self, filename = None, samplingrate = None):
        """
        Start logic level recording. The logic levels of CAN-RX is sampled and stored to sigrok/Pulseview compatible file.
        :param filename: File to which sampling data are written
        :param Samplingrate: sampling rate in Hz. If None, a sampling rate suitable for the CAN bit rate is set.
        """

        if self.recordingActive:
        	raise CanOperationError("Logic recording is already running. It cannot be started more than once.")

        # if no samplingrate given, determine one depending bitrate
        if samplingrate is None:
            if self.cprotocol == self.PROTOCOL_CAN_20:
                samplingrate = self.bitrate * 10
            else:
                samplingrate = self.data_bitrate * 10
                
        # calculate prescaler for sampling clock
        scaler = int(120000000 / samplingrate)
        if scaler < 3:
            scaler = 3
        if scaler > 0xff: 
            scaler = 0xff
        realrate = int(120000000 / scaler)
        # if realrate != samplingrate: cannot set samplingrate to given value. Next possible value would be: realrate
        self.samplingrate = realrate

        if filename is None:
            filename = "usbtingo_capture.sr"
        self.logicoutfilename = filename    
        self.LR_LUT = [bytearray([(byte & (1 << (7 - i))) and 0x4F or 0x2E for i in range(8)]) for byte in range(256)]
        self.logicoutfile = open("logic-1-1", "wb")
        self.recordingActive = True
        self.recordingShutdown = None
        for t in self.ep2in_transfer:
            t.submit()
        self.command_write(self.CMD_LOGIC_SETCONFIG, (scaler << 8) | 0)


    def recording_stop(self):
        """
        Stop logic level recording and generate output file with sampled data.
        """
        if not self.recordingActive:
            return
        self.recordingShutdown = threading.Event()
        self.recordingActive = False
        
        self.command_write(self.CMD_LOGIC_SETCONFIG, 0)

        # wait until last packet was received
        self.recordingShutdown.wait()
        self.recordingShutdown = None

        for transfer in self.ep2in_transfer:
            try:
                transfer.cancel()
            except usb1.USBErrorNotFound:
                pass

        txerrors, = struct.unpack("<I", self.usbhandle.controlRead(usb1.TYPE_VENDOR, self.CMD_LOGIC_GETTXERRORS, 0, 0, 4))
        if txerrors > 0:
            raise CanOperationError("TX errors while logic recording. The sampled data will be corrupted. Please, try to decrease sampling rate!", txerrors)
        
        self.logicoutfile.close()
        with zipfile.ZipFile(self.logicoutfilename, 'w', zipfile.ZIP_DEFLATED) as zipped_f:
            zipped_f.write("logic-1-1")
            zipped_f.writestr("version", "2")
            zipped_f.writestr("metadata", f"[device 1]\ncapturefile=logic-1\ntotal probes=1\nsamplerate={self.samplingrate} Hz\ntotal analog=0\nprobe1=CAN\nunitsize=1")

    def usbtransfer_ep2in_callback(self, t):
        """
        Complete callback for EP2IN USB transfers. This is the logic level pipe.
        :param t: USB transfer object
        """
        data = t.getBuffer()
        length = t.getActualLength()

        # check if this is the last packet from the device
        if self.recordingShutdown is not None and length < len(data):
            self.recordingShutdown.set()
        
        startposition = 0

        # write raw recording data to out file
        for i in range(startposition, length):
            self.logicoutfile.write(self.LR_LUT[data[i]])
                
        return True

    def usbtransfer_ep1in_callback(self, t):
        """
        Complete callback for EP1IN USB transfers. This is the statusreport pipe.
        :param t: USB transfer object
        """
        data = t.getBuffer()
        if data[0] != 0x80:
            return True

        report = USBtingoStatusreport(data[:32])
        for listener in self.statusreport_listeners:
            listener(report)
                
        return True

    def statusreport_listener_add(self, func):
        """
        Add a statusreport listener (callback)
        :param func: Function to add to listener list
        :type func: function
        """
        self.statusreport_listeners.append(func)

    def statusreport_listener_remove(self, func):
        """
        Remove given function from listeners list
        :param func: Function to remove from listener list
        :type func: function
        """
        if func in self.statusreport_listeners:
            self.statusreport_listeners.remove(func)
        else:
            raise CanOperationError("Failed to remove status report listener")

    def _apply_filters(self, filters):
        """
        Apply given filters. Try to set hardware filter (there are 32 each for standard and extended frames)
        :param filters: List of filters
        """
        self.command_write(self.CMD_FILTER_DISABLE_ALL)

        if filters:
            filters_std = [can_filter for can_filter in filters if not can_filter.get("extended")]
            filters_ext = [can_filter for can_filter in filters if can_filter.get("extended")]

            if len(filters_std) <= 32 and len(filters_ext) <= 32:
                for i, can_filter in enumerate(filters_std):
                    self.command_write(self.CMD_FILTER_SET_STD, i, 0, struct.pack("<III", 1, can_filter["can_id"], can_filter["can_mask"]))

                for i, can_filter in enumerate(filters_ext):
                    self.command_write(self.CMD_FILTER_SET_EXT, i, 0, struct.pack("<III", 1, can_filter["can_id"], can_filter["can_mask"]))
            
                self._is_filtered = True
                return
            else:
                log.warning("Only 32 hardware filters possible. Fallback to software filtering.")

        # accept all
        self.command_write(self.CMD_FILTER_SET_STD, 0, 0, struct.pack("<III", 1, 0, 0))
        self.command_write(self.CMD_FILTER_SET_EXT, 0, 0, struct.pack("<III", 1, 0, 0))
        self._is_filtered = False
        log.info("Hardware filtering has been disabled")

    @property
    def state(self):
        """
        Bus state currently set (ACTIVE or PASSIVE allowed).
        """
        return self._state
        
    @state.setter
    def state(self, new_state):
        """
        Bus state currently set (ACTIVE or PASSIVE allowed).
        """    
        if new_state not in [BusState.ACTIVE, BusState.PASSIVE]:
            raise ValueError("BusState must be Active or Passive")
        mode = self.MODE_ACTIVE if new_state == BusState.ACTIVE else self.MODE_LISTENONLY
        self.command_write(self.CMD_SET_MODE, mode)
        self._state = new_state
    

    @staticmethod
    def _detect_available_configs():
        """
        Detect USBtingo devices
        """
        ctx = usb1.USBContext()
        ctx.open()
        channels = []        
        for device in ctx.getDeviceIterator():
            if device.getVendorID() == USBtingoBus.USB_VID and device.getProductID() == USBtingoBus.USB_PID:                
                try:
                    channels.append(device.getSerialNumber())
                except usb1.USBErrorAccess:
                    pass
        ctx.close()

        return [
            {"interface": "usbtingo", "channel": channel}
            for channel in channels
        ]

class USBtingoStatusreport(object):

    def __init__(self, rawdata):
        self.cmd, self.mode, self.overflow, self.procts, self.errorcount, self.protocolstatus, self.stats_std, self.stats_ext, self.stats_data, self.stats_dataBRS  = struct.unpack("<BBBxIIIIIII", rawdata)        

    def getTEC(self):
        return self.errorcount & 0xff

    def getREC(self):
        return (self.errorcount >> 8) & 0x7f

    def isReceiveErrorPassive(self):
        return (self.errorcount >> 15 & 0x1) == 1

    def isErrorPassive(self):
        return (self.protocolstatus >> 5 & 0x1) == 1

    def isBusOff(self):
        return (self.protocolstatus >> 7 & 0x1) == 1

    def isWarningStatus(self):
        return (self.protocolstatus >> 6 & 0x1) == 1

class TXConfirmation(object):
    
    def __init__(self, message):
        self.message = message
        self.confirmed = threading.Event()

    def confirm(self):
        self.confirmed.set()

    def wait_confirmed(self, timeout):
        return self.confirmed.wait(timeout)

        
class USBtingoUSBEventHandler(threading.Thread):
    def __init__(self, tingo):
        threading.Thread.__init__(self)
        self.tingo = tingo
        self.running = True

    def run(self):
        while self.running:
            self.tingo.ctx.handleEvents()
    def stop(self):
        self.running = False





