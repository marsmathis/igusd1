#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This file is part of the igusD1 repository (https://github.com/marsmathis/igusd1).
# Copyright 2022 Mathis Reuß-Hennschen.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, version 3.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

"""
Library for igus Dryve D1 motor controller
"""

import socket
import struct
import time

__author__ = "Mathis Reuß-Hennschen"
__copyright__ = "Copyright 2022 Mathis Reuß-Hennschen"
__license__ = "GPL"
__version__ = "1.0"


# =========================================================================== #
# Example file structure:
# --------------------------------------------------------------------------- #
# import igusd1
#
# dryve = igusd1.D1("[IP]", [Port])
# dryve.init()
# dryve.set_home("LSN", find_velocity, zero_velocity, acceleration)
# [miscellaneous moves]
#
# =========================================================================== #


def make_bytearray(read_write, obj_ind, sub_ind, data_len, temp_data=[-1, -1, -1, -1]):
    """
    Create a bytearray which corresponds to an instruction set for the igus D1.

    Parameters
    ----------
    read_write : int
        Select read or write mode (0 [read] / 1 [write]).
    obj_ind : list[2]
        List of two bytes to specify the request function.
    sub_ind : ind
        Sub-index for certain functions; if no subfunctions: 0
    data_len : int
        Amount of data bytes to send or receive.
    temp_data : list[1 to 4], optional
        Data to send (only applicable if read_write == 1).

    Returns
    -------
    array : bytes
        Answer of the device as bytes object.

    """

    # [transaction_id_1, transaction_id_2,      always 0, 0
    #  protocol_id_1, protocol_id_2, null,      always 0, 0, 0
    #  length,                                  amount of bytes to be sent after b5
    #  null, function_code, mei_type,           always 0, 43, 13
    #  rw_byte,                                 0 for read, 1 for write
    #  null, null,                              always 0, 0
    #  object_index_1, object_index_2,          set the function
    #  sub_index,                               set sub-index of function
    #  null, null, null,                        always 0, 0, 0
    #  bytes_amount,                            amount of data bytes to be sent/received
    #  data_1, data_2, data_3, data_4]          data bytes

    data = []
    for i in temp_data:
        if i != -1:
            data.append(i)

    array = bytearray([0, 0,
                       0, 0, 0,
                       0,
                       0, 43, 13,
                       read_write,
                       0, 0]
                       + obj_ind +
                       [sub_ind,
                       0, 0, 0,
                       data_len]
                       + data)

    array[5] = len(array) - 6

    return array


def get_array(selector):
    """
    Fetch one of the init or status arrays.

    Parameters
    ----------
    selector : str
        Select array ("status" / "shutdown" / "switch_on" / "enable_operation").

    Returns
    -------
    sel : bytearray
        Corresponding array to the selector.

    """
    if selector == "status":
        sel = make_bytearray(0, [96, 65], 0, 2)
    elif selector == "shutdown":
        sel = make_bytearray(1, [96, 64], 0, 2, [6, 0])
    elif selector == "switch_on":
        sel = make_bytearray(1, [96, 64], 0, 2, [7, 0])
    elif selector == "enable_operation":
        sel = make_bytearray(1, [96, 64], 0, 2, [15, 0])

    return sel


class D1:
    """
    Class for the igus D1 motor driver. The methods are used to control the device.
    """
    def __init__(self,
                 ip_address,
                 port=502):
        """
        Create socket and connect to IP and port.

        Parameters
        ----------
        ip : str
            IP of the device.
        port : int
            TCP port (default is 502).

        Returns
        -------
        None.

        """
        try:
            self.sock = socket.socket(socket.AF_INET,
                                      socket.SOCK_STREAM)

        except:
            pass

        try:
            self.sock.connect((ip_address, port))

        except:
            pass


    def send_command(self,
                     data):
        """
        Send a command to the device.

        Parameters
        ----------
        data : bytearray
            Bytearray to send to the device.

        Returns
        -------
        bytes
            Returned data as a bytes object.

        """
        self.sock.send(data)
        res = self.sock.recv(24)

        return res


    def set_shutdown(self):
        """
        Tell the device to shutdown (set status to 00000000 00000110).

        Returns
        -------
        None.

        """
        self.send_command(get_array("shutdown"))

        ba_1 = make_bytearray(0, [96, 65], 0, 2, [33, 6])
        ba_2 = make_bytearray(0, [96, 65], 0, 2, [33, 22])
        ba_3 = make_bytearray(0, [96, 65], 0, 2, [33, 2])

        while (self.send_command(get_array("status")) !=
               ba_1
               and
               self.send_command(get_array("status")) !=
               ba_2
               and
               self.send_command(get_array("status")) !=
               ba_3):
            print('Waiting for shutdown...')
            time.sleep(1)


    def set_switch_on(self):
        """
        Tell the device to switch on (set status to 00000000 00000111).

        Returns
        -------
        None.

        """
        self.send_command(get_array("switch_on"))

        while (self.send_command(get_array("status")) !=
               make_bytearray(0, [96, 65], 0, 2, [35, 6])
               and
               self.send_command(get_array("status")) !=
               make_bytearray(0, [96, 65], 0, 2, [35, 22])
               and
               self.send_command(get_array("status")) !=
               make_bytearray(0, [96, 65], 0, 2, [35, 2])):
            print('Waiting for switch-on...')
            time.sleep(1)


    def set_enable_operation(self):
        """
        Tell the device to enable operation (set status to 00000000 00001111).

        Returns
        -------
        None.

        """
        self.send_command(get_array("enable_operation"))

        while (self.send_command(get_array("status")) !=
               make_bytearray(0, [96, 65], 0, 2, [39, 6])
               and
               self.send_command(get_array("status")) !=
               make_bytearray(0, [96, 65], 0, 2, [39, 22])
               and
               self.send_command(get_array("status")) !=
               make_bytearray(0, [96, 65], 0, 2, [39, 2])):
            print('Waiting for enabling operation...')
            time.sleep(1)


    def init(self):
        """
        Attempt to enable the device (only works if ???)

        Returns
        -------
        None.

        """
        self.set_shutdown()
        self.set_switch_on()
        self.set_enable_operation()


    def set_feedrate(self, feedrate):
        """
        Set the feedrate to specified value.

        Parameters
        ----------
        feedrate : int
            Feedrate in steps per revolution.

        Returns
        -------
        None.

        """
        feedrate_bytes = feedrate.to_bytes(4, "little")

        self.send_command(make_bytearray(1, [96, 146], 1, 2, [feedrate_bytes[0],
                                                              feedrate_bytes[1]]))

        self.send_command(make_bytearray(1, [96, 146], 2, 1, [1]))


    def set_mode(self, mode):
        """
        Set the movement mode of the device.

        Parameters
        ----------
        mode : int
            Selected mode (1 [move] / 6 [home]).

        Returns
        -------
        None.

        """
        self.send_command(make_bytearray(1, [96, 96], 0, 1, [mode]))

        while (self.send_command(make_bytearray(0, [96, 97], 0, 1))
               !=
               make_bytearray(0, [96, 97], 0, 1, [mode])):

            time.sleep(1)


    def set_homing(self, method, find_velocity, zero_velocity, acceleration):
        """
        Find reference point (home) [method doesn't work yet I guess?].

        Parameters
        ----------
        method : str
            Homing method ("LSN" [limit switch negative] / ???).
        find_velocity : int
            Velocity when searching for switch (upper limit: 50000).
        zero_velocity : int
            Velocity when zeroing away from switch after pressing it.
        acceleration : int
            Acceleration/deceleration when starting/stopping move.

        Returns
        -------
        None.

        """
        self.set_mode(6)
        methods = {
            "LSN": 17,
            "LSP": 18,
            "IEN": 33,
            "IEP": 34,
            "SCP": 37,
            "AAF": 255
        }

        selected_method = methods[method]

        # homing method
        self.send_command(make_bytearray(1, [96, 152], 1, 1, [selected_method]))

        self.set_feedrate(6000)

        # homing velocity – max search velocity
        find_velocity_bytes = find_velocity.to_bytes(4, "little")
        self.send_command(make_bytearray(1, [96, 153], 1, 2, [find_velocity_bytes[0],
                                                              find_velocity_bytes[1]]))

        # zeroing velocity – velocity after contact
        zero_velocity_bytes = zero_velocity.to_bytes(4, "little")
        self.send_command(make_bytearray(1, [96, 153], 2, 2, [zero_velocity_bytes[0],
                                                              zero_velocity_bytes[1]]))

        # homing acceleration
        acceleration_bytes = acceleration.to_bytes(4, "little")
        self.send_command(make_bytearray(1, [96, 154], 0, 2, [acceleration_bytes[0],
                                                              acceleration_bytes[1]]))

        # start movement
        print(list(self.send_command(make_bytearray(1, [96, 64], 0, 2, [31, 0]))))

        # reset start bit
        self.send_command(make_bytearray(1, [96, 64], 0, 2, [15, 0]))

        while (self.send_command(get_array("status"))
               !=
               make_bytearray(0, [96, 65], 0, 2, [39, 22])):
            print("wait for Homing to end")
            print(list(self.send_command(get_array("status"))))
            time.sleep(1)

        self.send_command(get_array("enable_operation"))


    def move(self,
             velocity,
             acceleration,
             target_position):
        """
        Move the sled to an arbitrary position within the limits of the rod.

        Parameters
        ----------
        velocity : int
            Velocity when moving.
        acceleration : int
            Acceleration/deceleration when starting/stopping move.
        target_position : int
            Target position in steps.

        Returns
        -------
        None.

        """
        self.set_mode(1)

        velocity_bytes = velocity.to_bytes(4, "little")
        self.send_command(make_bytearray(1, [96, 129], 0, 4, [velocity_bytes[0],
                                                              velocity_bytes[1],
                                                              velocity_bytes[2],
                                                              velocity_bytes[3]]))

        acceleration_bytes = acceleration.to_bytes(4, "little")
        self.send_command(make_bytearray(1, [96, 131], 0, 4, [acceleration_bytes[0],
                                                              acceleration_bytes[1],
                                                              acceleration_bytes[2],
                                                              acceleration_bytes[3]]))

        target_position_bytes = target_position.to_bytes(4, "little")
        self.send_command(make_bytearray(1, [96, 122], 0, 4, [target_position_bytes[0],
                                                              target_position_bytes[1],
                                                              target_position_bytes[2],
                                                              target_position_bytes[3]]))

        print(list(self.send_command(make_bytearray(1, [96, 64], 0, 2, [31, 0]))))

        self.send_command(make_bytearray(1, [96, 64], 0, 2, [15, 0]))

        while (self.send_command(get_array("status"))
               !=
               make_bytearray(0, [96, 65], 0, 2, [39, 6])):

            time.sleep(0.1)
        status = []
        actual_position_bytes = self.send_command(make_bytearray(0, [96, 100], 0, 4))
        status.append(struct.unpack("<xxxxxxxxxxxxxxxxxxxi", actual_position_bytes)[0])
        actual_velocity_bytes = self.send_command(make_bytearray(0, [96, 108], 0, 4))
        status.append(struct.unpack("<xxxxxxxxxxxxxxxxxxxi", actual_velocity_bytes)[0])
        print(status)
        self.send_command(get_array("enable_operation"))


    def staggered_move(self,
                       velocity,
                       acceleration,
                       start_position,
                       iterations,
                       step_width,
                       wait_time,
                       go_back
                       ):
        """
        Move the sled to a start position, then move a specified amount of times
        by a specified distance and wait for a specified interval.

        Parameters
        ----------
        velocity : int
            Velocity when moving.
        acceleration : int
            Acceleration/deceleration when starting/stopping move.
        start_position : int
            Initial position to move to.
        iterations : int
            Amount of times to increment the position by step_width.
        step_width : int
            Distance between iterations.
        wait_time : float
            Wait time between moves.
        go_back : bool
            Go back to start_position at the end? (True [yes] / False [no]).

        Returns
        -------
        None.

        """
        self.set_mode(1)

        velocity_bytes = velocity.to_bytes(4, "little")
        self.send_command(make_bytearray(1, [96, 129], 0, 4, [velocity_bytes[0],
                                                              velocity_bytes[1],
                                                              velocity_bytes[2],
                                                              velocity_bytes[3]]))

        acceleration_bytes = acceleration.to_bytes(4, "little")
        self.send_command(make_bytearray(1, [96, 131], 0, 4, [acceleration_bytes[0],
                                                              acceleration_bytes[1],
                                                              acceleration_bytes[2],
                                                              acceleration_bytes[3]]))

        self.move(velocity, acceleration, start_position)

        for i in range(iterations):
            self.move(velocity, acceleration, start_position + step_width * (i + 1))
            time.sleep(wait_time)

        if go_back is True:
            self.move(velocity, acceleration, start_position)


    def get_status(self):
        """
        Receive current position and velocity.

        Returns
        -------
        status : list[2]
            Receive status list [current position, current velocity].

        """
        status = []

        actual_position_bytes = self.send_command(make_bytearray(0, [96, 100], 0, 4))
        status.append(struct.unpack("<xxxxxxxxxxxxxxxxxxxi", actual_position_bytes)[0])

        actual_velocity_bytes = self.send_command(make_bytearray(0, [96, 108], 0, 4))
        status.append(struct.unpack("<xxxxxxxxxxxxxxxxxxxi", actual_velocity_bytes)[0])

        return status

    def close(self):
        """
        Closes the socket.

        Returns
        -------
        None.

        """
        self.sock.close()
