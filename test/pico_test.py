#! /usr/bin/env python3
# -*- coding: utf8 -*-

''' Tests AMC-Pico8 driver functionality '''

import argparse
import fcntl
import struct
import numpy as np
import os

import picodefs

class PicoTest(object):
    """docstring for PicoTest"""

    def __init__(self, filename, debug=True):
        super(PicoTest, self).__init__()
        self.debug = debug
        self.f = os.open(filename, os.O_RDWR)

        if self.debug:
            print('Opened', filename, 'with fileno', self.f)

    def __del__(self):
        os.close(self.f)
        if self.debug:
            print('Closed', self.f)

    def get_range(self):
        ''' Gets picoammeter range '''
        buf = fcntl.ioctl(self.f, picodefs.GET_RANGE, ' ')
        rng = struct.unpack('B', buf)[0]
        if self.debug:
            print('get_range():', '0b{0:08b}'.format(rng))
        return rng

    def set_range(self, rng):
        ''' Sets picoammeter range '''
        if self.debug:
            print('set_range(', '0b{0:08b}'.format(rng), ')')
        buf = struct.pack('B', rng)
        fcntl.ioctl(self.f, picodefs.SET_RANGE, buf)

    def get_fsamp(self):
        ''' Gets picoammeter sampling frequency '''
        buf = fcntl.ioctl(self.f, picodefs.GET_FSAMP, '    ')
        fsamp = struct.unpack('I', buf)[0]
        if self.debug:
            print('get_fsamp():', str(fsamp))
        return fsamp

    def set_fsamp(self, fsamp):
        ''' Sets picoammeter sampling frequency '''
        if self.debug:
            print('set_fsamp(', str(fsamp), ')')

        buf = struct.pack('I', int(fsamp))
        fcntl.ioctl(self.f, picodefs.SET_FSAMP, buf)

    def get_b_trans(self):
        ''' Gets number of bytes transfered in previous read() '''
        buf = fcntl.ioctl(self.f, picodefs.GET_B_TRANS, '    ')
        b_trans = struct.unpack('I', buf)[0]
        if self.debug:
            print('get_b_trans():', str(b_trans))
        return b_trans

    def set_trigger(self, limit, ch_sel, nr_samp=1000, mode='POS EDGE'):
        ''' Sets trigger control '''
        if self.debug:
            print('set_trigger(', str(limit), ',', str(nr_samp), ',', str(mode), ')')

        _MODE = ['DISABLED', 'POS EDGE', 'NEG EDGE', 'BOTH EDGES']
        buf = bytes()
        buf += struct.pack('f', limit)
        buf += struct.pack('I', nr_samp)
        buf += struct.pack('I', ch_sel)
        if isinstance(mode, int):
            buf += struct.pack('I', mode)
        else:
            buf += struct.pack('I', _MODE.index(mode))

        fcntl.ioctl(self.f, picodefs.SET_TRG, buf)

    def set_ring_buf(self, nr_samp):
        ''' Sets number of bytes in ring buffer (pre-trigger condition) '''
        if self.debug:
            print('set_ring_buf(', str(nr_samp), ')')

        buf = struct.pack('I', nr_samp)
        fcntl.ioctl(self.f, picodefs.SET_RING_BUF, buf)

    def set_gate_mux(self, sel):
        ''' Sets gate mux '''
        if self.debug:
            print('set_gate_mux(', str(sel), ')')

        buf = struct.pack('I', sel)
        fcntl.ioctl(self.f, picodefs.SET_GATE_MUX, buf)

    def set_conv_mux(self, sel):
        ''' Sets convert mux '''
        if self.debug:
            print('set_conv_mux(', str(sel), ')')

        buf = struct.pack('I', sel)
        fcntl.ioctl(self.f, picodefs.SET_CONV_MUX, buf)

    def read(self, nr_samp, print_data=True):
        ''' Reads picoammeter data'''
        if self.debug:
            print('read(', str(nr_samp), ')')

        buf = os.read(self.f, nr_samp*8*4)
        formatStr = '<' + 'f'*nr_samp*8
        chs = struct.unpack(formatStr, buf)
        chs = np.array(chs)
        chs = chs.reshape((nr_samp, 8))

        if print_data:
            print(chs)


def main():
    ''' Performs AMC-Pico8 driver test '''

    parser = argparse.ArgumentParser(description='Performs tests on AMC-Pico8 driver')
    parser.add_argument('--filename', dest='file', action='store')

    args = parser.parse_args()

    pico_test = PicoTest(args.file)
    pico_test.get_range()
    pico_test.set_range(0b1110000)
    pico_test.get_range()
    pico_test.get_fsamp()
    pico_test.set_fsamp(500e3)
    pico_test.get_fsamp()

    pico_test.get_b_trans()

    pico_test.set_trigger(limit=5e-8, ch_sel=0, nr_samp=2000, mode='POS EDGE')
    pico_test.set_ring_buf(100)

    pico_test.set_conv_mux(0)
    pico_test.set_gate_mux(0)

    pico_test.read(1)
    pico_test.read(10)
    pico_test.read(100000)

    pico_test.set_conv_mux(0)
    [pico_test.set_gate_mux(i) for i in range(8)]

    # Clean-up
    pico_test.set_conv_mux(0)
    pico_test.set_gate_mux(0)
    pico_test.set_trigger(limit=5e-8, ch_sel=0, nr_samp=2000, mode='DISABLED')
    pico_test.set_fsamp(1e6)
    pico_test.set_ring_buf(0)

    print('***************************************')
    print(' OK: AMC-Pico8 test succesfully passed ')
    print('***************************************')

if __name__ == '__main__':
    main()
