/*
 * Copyright 2019 Analog Devices Inc.
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

#ifndef INCLUDED_M2K_ANALOG_OUT_SINK_IMPL_H
#define INCLUDED_M2K_ANALOG_OUT_SINK_IMPL_H

#include <m2k/analog_out_sink.h>
#include <libm2k/analog/m2kanalogout.hpp>


namespace gr {
namespace m2k {

class analog_out_sink_impl : public analog_out_sink {
private:
    libm2k::analog::M2kAnalogOut *d_analog_out;
    const std::string d_uri;
    const int d_buffer_size;
    bool d_stream_voltage_values;
    std::vector<bool> d_cyclic_buffer;
    bool d_first_iteration;

public:
    analog_out_sink_impl(const std::string &uri,
                         const int buffer_size,
                         std::vector<double> sampling_frequency,
                         std::vector<int> oversampling_ratio,
                         const std::vector<int> &kernel_buffers,
                         const std::vector<int> &cyclic,
                         bool calibrate_DAC,
                         bool input_voltage);

    ~analog_out_sink_impl();

    void set_params(std::vector<double> sampling_frequency,
                    std::vector<int> oversampling_ratio);

    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);
};

} // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_ANALOG_OUT_SINK_IMPL_H */
