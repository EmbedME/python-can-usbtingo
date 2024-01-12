# This file is part of the python-can-usbtingo project.
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

import pathlib
from setuptools import setup

# The directory containing this file
HERE = pathlib.Path(__file__).parent

# The text of the README file
README = (HERE / "README.md").read_text()

setup(
    name="python-can-usbtingo",
    version="1.0.2",
    author="Thomas Fischl",
    author_email="tfischl@gmx.de",
    description="Python-can USBtingo",
    long_description=README,
    long_description_content_type="text/markdown",
    url="https://github.com/EmbedME/python-can-usbtingo",
    py_modules = ["usbtingobus"],
    python_requires=">=3.8",
    install_requires=[
        "python-can",
        "libusb1"
    ],
    entry_points = {
        'can.interface': [
            'usbtingo = usbtingobus:USBtingoBus'
        ]
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)",
        "Operating System :: OS Independent",
    ]
)
