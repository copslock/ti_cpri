#!/usr/bin/env python
#
# Copyright (c) 2018-2019, Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# usage: ./sysfw_trace_parser.py [-h] (-l LOG_FILE | -d SERIAL_PORT)
#                                (-o OUTPUT_FILE | -O) [-t] [-r RULES_FILE]
#                                [-Pm]
# 
# System Firmware Log Parse utility URL: http://software-
# dl.ti.com/tisci/esd/latest/4_trace/trace.html
# 
# Required arguments - Choose one of the inputs:
#   -l LOG_FILE, --log_file LOG_FILE
#                         Log File provided as input (default: None)
#   -d SERIAL_PORT, --serial_port SERIAL_PORT
#                         Provide Device as input: Requires pyserial package
#                         installed: See https://pyserial.readthedocs.io/
#                         (default: None)
# 
# Required arguments - Choose one of the outputs:
#   -o OUTPUT_FILE, --output_file OUTPUT_FILE
#                         Parse out the output to a file (default: None)
#   -O, --output_console  Log File to parse and report results to console
#                         (default: False)
# 
# optional arguments:
#   -h, --help            show this help message and exit
#   -t, --time_stamp_relative
#                         Add TimeStamp to output in relative milliseconds(this
#                         is approximation ONLY) (default: False)
#   -r RULES_FILE, --rules_file RULES_FILE
#                         Alternate Rules file (default: ./sysfw_trace_rules.json)
#   -Pm, --print_match_only
#                         Print just decoded data, (default: False)

import os
import io
import sys
import multiprocessing
import json
import re
import argparse
import logging
import string
import time

# Python 3 Hack
try:
    unichr
    _control_chars = ''.join(map(unichr, range(0, 32) + range(127, 160)))
except NameError:
    unichr = chr
    _control_chars = ''.join(map(unichr, list(range(0, 32)) + list(range(127, 160))))

_control_char_re = re.compile('[%s]' % re.escape(_control_chars))

# Python 3 Hack
try:
    xrange
    _range = xrange
except NameError:
    _range = range


def create_mask(highest_bit, lowest_bit):
    """ Return a bit  mask using a start and end bit
    """

    m = 1 << (highest_bit + 1)
    m = m - 1

    m_c = 0
    if lowest_bit > 0:
        m_c = 1 << lowest_bit
        m_c = m_c - 1

    return m & (~m_c)


def cleanup_string(s):
    """ A magical combination of regex and string squashing to get a clean output
    """
    s2 = ''.join([x for x in s if x in string.printable])
    s = _control_char_re.sub('', s2)
    return s.strip()


class sysfw_trace_rules:

    """ Class for processing the trace rules
    """
    dir_location = os.path.dirname(os.path.realpath(__file__))
    rules_file_name = 'sysfw_trace_rules.json'

    def get_rules_file(self):
        """ Return the current rules file
        """
        return self.rules_file

    def set_rules_file(self, fname):
        """ Set up a new rules file
        """
        try:
                self.fh = io.open(
                    fname,
                    mode='r',
                    encoding="utf8",
                    errors='ignore')
        except Exception as ex:
                raise Exception(fname + ":  File not readable?:" + str(ex))
        # Make sure rules is a JSON file
        self.json_rules = json.load(self.fh)

        # XXX: Introduce schema check later?

        self.primary_match_re = re.compile(
            r"" + self.json_rules['description_trace']['detect_regex'])
        trace_struct = self.json_rules['description_trace']

        self.domain_mask = create_mask(
            trace_struct['domain']['higher_bit'],
            trace_struct['domain']['lower_bit'])
        self.domain_shift = trace_struct['domain']['lower_bit']

        self.action_mask = create_mask(
            trace_struct['action_id']['higher_bit'],
            trace_struct['action_id']['lower_bit'])
        self.action_shift = trace_struct['action_id']['lower_bit']

        self.msg_mask = create_mask(
            trace_struct['message_data']['higher_bit'],
            trace_struct['message_data']['lower_bit'])
        self.msg_shift = trace_struct['message_data']['lower_bit']

        self.decoder_ring = self.json_rules['message_decode']

        self.rules_file = fname
        return

    def primary_decode_string(self, in_str):
        x = int(in_str, 16)
        domain = (x & self.domain_mask) >> self.domain_shift
        action_id = (x & self.action_mask) >> self.action_shift
        message_data = (x & self.msg_mask) >> self.msg_shift
        domain_decode = None
        try:
            domain_decode = self.decoder_ring["0x%02X" % domain]
        except Exception:
            s = "0x%08X: Unknown DOMAIN:0x%02X Action: 0x%02X MSG:0x%06X " % (
                x, domain, action_id, message_data)

        action_decode = None
        modifier_string = ""
        if domain_decode is not None:
            action_domain_name = domain_decode['action_domain_name']
            action_modifiers = None
            try:
                action_modifiers = domain_decode['action_modifiers']
            except:
                pass
            if action_modifiers is not None:
                for idx in _range(len(action_modifiers)):
                    modifier = action_modifiers[idx]
                    modifier_mask = int(modifier['mask'], 16)
                    s1 = ''
                    if (modifier_mask & action_id):
                        s1 = "%s(%s)" % (
                            modifier['modifier_short'],
                            modifier['modifier_long'])
                        modifier_string = modifier_string + "/" + s1
                        # Make sure to clear the action_id mask
                        action_id = action_id & (~modifier_mask)

            try:
                action_decode = domain_decode['actions']["0x%02X" % action_id]
            except Exception:
                s = "0x%08X: %10s%s: Unknown Action: 0x%02X MSG:0x%06X " % (
                    x, action_domain_name, modifier_string, action_id, message_data)

        if action_decode is not None:
            action_short = action_decode['action_short']
            action_long = action_decode['action_long']
            action_str = "%s(%s)" % (action_short, action_long)
            s = "0x%08X: %10s%s: %40s:" % (
                x,
                action_domain_name, modifier_string,
                action_str)
            try:
                msg_data_decode = action_decode['msg_data']
                for idx in _range(len(msg_data_decode)):
                    msg = msg_data_decode[idx]
                    msk = create_mask(msg['higher_bit'], msg['lower_bit'])
                    v = (message_data & msk) >> msg['lower_bit']
                    s1 = msg['name'] + ': ' + msg['fmt'] % v
                    s = s + ' ' + s1
            except Exception:
                s = s + " MSG:0x%06X" % message_data

        return cleanup_string(s)

    def process_data(self, input_class, output_class, match_only):
        self.input_class = input_class
        self.output_class = output_class
        self.match_only = match_only

        while self.input_class.is_eof() is not True:
            in_str = self.input_class.get_next_line()
            print_it = False

            pmatch = self.primary_match_re.match(in_str)

            decode_string = in_str

            if match_only is False:
                print_it = True

            if pmatch is not None:
                print_it = True
                decode_string = self.primary_decode_string(in_str)

            if print_it is True:
                self.output_class.send_next_line(decode_string)

    def __init__(self):
        self.rules_file = os.path.join(self.dir_location, self.rules_file_name)
        return


class sysfw_trace_input_file:

        """ Input Class: File
        """

        def __init__(self, fname=None):
            try:
                self.fh = io.open(
                    fname,
                    mode='r',
                    encoding="utf8",
                    errors='ignore')
            except Exception as ex:
                raise Exception(fname + ":  File not found?:" + str(ex))
            self.fsize = os.fstat(self.fh.fileno()).st_size
            return

        def is_eof(self):
            return self.fh.tell() == self.fsize

        def get_next_line(self):
            ln = self.fh.readline()
            return cleanup_string(ln)


class sysfw_trace_input_serial:

        """ Input Class: Serial port read
        """

        def __init__(self, fname=None):
            try:
                import serial
            except ImportError as err:
                raise Exception('Please install python package pyserial')

            self.ser = serial.Serial()
            self.ser.port = fname
            self.ser.baudrate = 115200
            self.ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
            self.ser.parity = serial.PARITY_NONE  # set parity check: no parity
            self.ser.stopbits = serial.STOPBITS_ONE  # number of stop bits
            self.ser.timeout = None  # block read
            self.ser.xonxoff = False  # disable software flow control
            self.ser.rtscts = False  # disable hardware (RTS/CTS) flow control
            self.ser.dsrdtr = False  # disable hardware (DSR/DTR) flow control
            self.ser.writeTimeout = 2  # timeout for write

            try:
                self.ser.open()
            except Exception as e:
                raise Exception(
                    "error open serial port(" + fname + "): " + str(e) + ': Try: `python -m serial.tools.list_ports -v`')

            # We use \r in SYSFW.. so, wrap it back up:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser_io = io.TextIOWrapper(
                io.BufferedRWPair(self.ser, self.ser, 1),
                newline='\n',
                line_buffering=True, encoding="ascii", errors='ignore')
            return

        def is_eof(self):
            # Assume always open unless otherwise..
            return False

        def get_next_line(self):
            ln = self.ser_io.readline()
            return cleanup_string(ln)


class sysfw_trace_output_console:

        """ Input Class: print to console
        """

        def __init__(self, fname=None, ts=None):
            self.ts = ts
            return

        def send_next_line(self, s):
            if self.ts is not None:
                s = self.ts.get_ts() + s
            print(s)


class sysfw_trace_output_file:

        """ Input Class: print to console
        """

        def __init__(self, fname=None, ts=None):
            try:
                self.fh = io.open(
                    fname,
                    mode='w',
                    encoding="utf8",
                    errors='ignore')
            except Exception as ex:
                raise Exception(fname + ":  File not writable?:" + str(ex))
            self.ts = ts
            return

        def send_next_line(self, s):
            if self.ts is not None:
                s = self.ts.get_ts() + s
            self.fh.write(s + '\n')


class ts:

    def _get_time_ms(self):
        return int(round(time.time() * 1000))

    def get_ts(self):
        self.c_time = self._get_time_ms()
        return "%10s: " % str(self.c_time - self.epoch)

    def reset_ts(self):
        self.epoch = self._get_time_ms()

    def __init__(self):
        self.epoch = self._get_time_ms()


class sysfw_trace_cli:

    """ This is the Base Command Line interface Class
    """

    def __init__(self):
        self.rules = sysfw_trace_rules()
        self.ts = ts()
        return

    def parse_args(self):
        """ Helper to parse the command line arguments
        """
        help_text = "System Firmware Log Parse utility "
        help_text = help_text + "URL: "
        help_text = help_text + \
            "http://software-dl.ti.com/tisci/esd/latest/4_trace/trace.html"

        parser = argparse.ArgumentParser(prog=__file__,
                                         description=help_text,
                                         formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        optional = parser._action_groups.pop()
        ig = parser.add_argument_group(
            'Required arguments - Choose one of the inputs')
        input_group = ig.add_mutually_exclusive_group(required=True)

        input_group.add_argument(
            '-l',
            '--log_file',
            help="Log File provided as input",
            action="store")
        input_group.add_argument(
            '-d',
            '--serial_port',
            help="Provide Device as input: " +
            "Requires pyserial package installed: " +
            "See https://pyserial.readthedocs.io/",
            action="store")

        og = parser.add_argument_group('Required arguments - Choose one of the outputs')
        output_group = og.add_mutually_exclusive_group(required=True)

        output_group.add_argument(
            '-o',
            '--output_file',
            help="Parse out the output to a file",
            action="store")

        output_group.add_argument(
            '-O',
            '--output_console',
            help="Log File to parse and report results to console",
            action="store_true")

        optional.add_argument(
            '-t',
            '--time_stamp_relative',
            help="Add TimeStamp to output in relative milliseconds(this is approximation ONLY)",
            action="store_true")

        optional.add_argument(
            '-r',
            '--rules_file',
            help="Alternate Rules file",
            action="store",
            default=self.rules.get_rules_file())

        optional.add_argument(
            '-Pm',
            '--print_match_only',
            help="Print just decoded data,",
            action="store_true",
            default=False)

        parser._action_groups.append(optional)
        self.cmd_args = parser.parse_args()

        if self.cmd_args.rules_file is not None:
            self.rules.set_rules_file(self.cmd_args.rules_file)

        if self.cmd_args.log_file is not None:
            self.input_class = sysfw_trace_input_file(self.cmd_args.log_file)

        if self.cmd_args.serial_port is not None:
            self.input_class = sysfw_trace_input_serial(
                self.cmd_args.serial_port)

        timestamp = None
        if self.cmd_args.time_stamp_relative is True:
            timestamp = self.ts
        if self.cmd_args.output_console is True:
            self.output_class = sysfw_trace_output_console(ts=timestamp)

        if self.cmd_args.output_file is not None:
            self.output_class = sysfw_trace_output_file(
                self.cmd_args.output_file, ts=timestamp)

    def process_data(self):
        self.ts.reset_ts()
        self.rules.process_data(
            self.input_class,
            self.output_class,
            self.cmd_args.print_match_only)


def sysfw_cli_job():
    cli = sysfw_trace_cli()
    cli.parse_args()
    cli.process_data()


def sysfw_cli_wrapper():
    """ If we make this a pypi package eventually, this would be the entry point
    """

    source_debug = 0

    if source_debug == 1:
        sysfw_cli_job()
    else:
        try:
            sysfw_cli_job()
        except Exception as e:
            print (str(e))
            sys.exit(1)

if __name__ == '__main__':
    sysfw_cli_wrapper()

# Format via !autopep8 -i -a %
# vim: et:ts=4
