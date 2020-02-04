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


from gnuradio import gr, gr_unittest, analog, blocks
import m2k
import libm2k


class test_analog_out_sink(gr_unittest.TestCase):

    def setUp(self):
        self.tb = gr.top_block()
        self.ctx = libm2k.m2kOpen()
        self.ctx.calibrateDAC()

        self.analog_in = self.ctx.getAnalogIn()
        self.analog_in.enableChannel(0, True)
        self.analog_in.enableChannel(1, True)
        self.analog_in.setSampleRate(100000)
        self.analog_in.setRange(0, -10, 10)

    def tearDown(self):
        self.tb = None
        libm2k.contextClose(self.ctx)

    def test_processed_signal(self):
        nb_samples = 16
        tb = self.tb
        expected_result1 = [1.0] * nb_samples
        expected_result2 = [3.0] * nb_samples

        dst1 = m2k.analog_out_sink('ip:192.168.2.1', 1024, [750000, 750000],
                                   [1, 1], [4, 4], [0, 0], False, True)

        src1 = analog.sig_source_f(750000, analog.GR_CONST_WAVE, 1000, 1, 0, 0)
        src2 = analog.sig_source_f(750000, analog.GR_CONST_WAVE, 1000, 3, 0, 0)

        tb.connect((src1, 0), (dst1, 0))
        tb.connect((src2, 0), (dst1, 1))

        tb.start()

        samples = self.analog_in.getSamples(nb_samples)

        map(lambda x, y: self.assertAlmostEqual(x, y, 0.1), expected_result1, samples[0])
        map(lambda x, y: self.assertAlmostEqual(x, y, 0.1), expected_result2, samples[1])

    def test_raw_signal(self):
        analog_out = self.ctx.getAnalogOut()
        nb_samples = 16
        tb = self.tb
        expected_result1 = [1.0] * nb_samples
        expected_result2 = [3.0] * nb_samples

        dst1 = m2k.analog_out_sink('ip:192.168.2.1', 1024, [750000, 750000],
                                   [1, 1], [4, 4], [0, 0], False, False)

        src1 = analog.sig_source_s(750000, analog.GR_CONST_WAVE, 1000, analog_out.convertVoltsToRaw(0, 1), 0, 0)
        src2 = analog.sig_source_s(750000, analog.GR_CONST_WAVE, 1000, analog_out.convertVoltsToRaw(1, 3), 0, 0)

        tb.connect((src1, 0), (dst1, 0))
        tb.connect((src2, 0), (dst1, 1))

        tb.start()

        samples = self.analog_in.getSamples(nb_samples)

        map(lambda x, y: self.assertAlmostEqual(x, y, 0.1), expected_result1, samples[0])
        map(lambda x, y: self.assertAlmostEqual(x, y, 0.1), expected_result2, samples[1])


if __name__ == '__main__':
    gr_unittest.run(test_analog_out_sink, "test_analog_out_sink.xml")
