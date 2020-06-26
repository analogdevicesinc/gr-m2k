/*
 * Copyright 2020 Analog Devices Inc.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_MIXED_SIGNAL_SOURCE_IMPL_H
#define INCLUDED_MIXED_SIGNAL_SOURCE_IMPL_H

#include <m2k/mixed_signal_source.h>
#include <libm2k/analog/m2kanalogin.hpp>
#include <libm2k/digital/m2kdigital.hpp>


namespace gr {
    namespace m2k {
        class mixed_signal_source_impl : public mixed_signal_source {
        private:
            libm2k::analog::M2kAnalogIn *d_analog_in;
            libm2k::digital::M2kDigital *d_digital;

            const std::string d_uri;
            int d_kernel_buffers;

            unsigned int d_buffer_size;
            unsigned int d_sample_index;
            unsigned long d_items_in_buffer;
            const short *d_analogical_raw_samples;
            const unsigned short *d_digital_raw_samples;

        public:
            mixed_signal_source_impl(const std::string &uri,
                                  int buffer_size,
                                  double sampling_frequency,
                                  int oversampling_ratio,
                                  int kernel_buffers,
                                  int trigger_condition,
                                  int trigger_mode,
                                  int trigger_source,
                                  int trigger_delay,
                                  double trigger_level);

            ~mixed_signal_source_impl();

            int work(int noutput_items,
                     gr_vector_const_void_star &input_items,
                     gr_vector_void_star &output_items);

            unsigned short get_channel_value(unsigned int channel, unsigned short sample);
        };

    } // namespace m2k
} // namespace gr

#endif /* INCLUDED_MIXED_SIGNAL_SOURCE_IMPL_H */
