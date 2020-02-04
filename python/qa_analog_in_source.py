#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2011 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.


from gnuradio import gr, gr_unittest, blocks
import m2k
import libm2k


class test_analog_in_source(gr_unittest.TestCase):

    def setUp(self):
        self.tb = gr.top_block()
        self.ctx = libm2k.m2kOpen()
        self.ctx.calibrateDAC()

        analog_out = self.ctx.getAnalogOut()
        analog_out.setSampleRate(0, 75000)
        analog_out.setSampleRate(1, 75000)
        analog_out.enableChannel(0, True)
        analog_out.enableChannel(1, True)

        buffer1 = [1] * 1024
        buffer2 = [3] * 1024

        self.buffer = [buffer1, buffer2]

        analog_out.setCyclic(True)
        analog_out.push(self.buffer)

    def tearDown(self):
        self.tb = None
        libm2k.contextClose(self.ctx)

    def test_processed_signal(self):
        nb_samples = 10
        tb = self.tb
        expected_result1 = [1.0] * nb_samples
        expected_result2 = [3.0] * nb_samples
        src1 = m2k.analog_in_source('ip:192.168.2.1', 0x4000, [1, 1], [0, 0], 10000, 1, 4, False, True, [0, 0], [0, 0], 0, 0, [0, 0])
        op1 = blocks.head(gr.sizeof_float, nb_samples)
        op2 = blocks.head(gr.sizeof_float, nb_samples)

        msg = blocks.message_debug()

        dst1 = blocks.vector_sink_f()
        dst2 = blocks.vector_sink_f()

        tb.connect((src1, 0), op1)
        tb.connect((src1, 1), op2)
        tb.msg_connect((src1, 'msg'), (msg, 'print'))
        tb.connect(op1, dst1)
        tb.connect(op2, dst2)
        tb.start()

        dst_data1 = dst1.data()
        dst_data2 = dst2.data()
        map(lambda x, y: self.assertAlmostEqual(x, y, 0.1), expected_result1, dst_data1)
        map(lambda x, y: self.assertAlmostEqual(x, y, 0.1), expected_result2, dst_data2)

    def test_raw_signal(self):
        analog_in = self.ctx.getAnalogIn()
        nb_samples = 10
        tb = self.tb
        expected_result1 = [1.0] * nb_samples
        expected_result2 = [3.0] * nb_samples

        src1 = m2k.analog_in_source('ip:192.168.2.1', 1024, [1, 1], [0, 0], 10000, 1, 4, False, False, [0, 0], [0, 0], 0, 0, [0, 0])

        op1 = blocks.head(gr.sizeof_short, nb_samples)
        op2 = blocks.head(gr.sizeof_short, nb_samples)

        dst1 = blocks.vector_sink_s()
        dst2 = blocks.vector_sink_s()

        tb.connect((src1, 0), op1)
        tb.connect((src1, 1), op2)
        tb.connect(op1, dst1)
        tb.connect(op2, dst2)
        tb.start()

        dst_data1 = dst1.data()
        dst_data2 = dst2.data()
        map(lambda x, y: self.assertAlmostEqual(x, analog_in.convertRawToVolts(0, y), 0.1), expected_result1, dst_data1)
        map(lambda x, y: self.assertAlmostEqual(x, analog_in.convertRawToVolts(1, y), 0.1), expected_result2, dst_data2)


if __name__ == '__main__':
    gr_unittest.run(test_analog_in_source, "test_analog_in_source.xml")
