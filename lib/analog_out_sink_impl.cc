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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "analog_out_sink_impl.h"
#include "analog_in_source_impl.h"
#include <gnuradio/io_signature.h>
#include <libm2k/m2k.hpp>
#include <boost/lexical_cast.hpp>


namespace gr {
namespace m2k {

analog_out_sink::sptr
analog_out_sink::make(const std::string &uri,
                      const int buffer_size,
                      std::vector<double> sampling_frequency,
                      std::vector<int> oversampling_ratio,
                      const std::vector<int> &kernel_buffers,
                      const std::vector<int> &cyclic,
                      bool calibrate_DAC,
                      bool input_voltage)
{
    return gnuradio::get_initial_sptr
        (new analog_out_sink_impl(uri, buffer_size, sampling_frequency, oversampling_ratio, kernel_buffers,
                                  cyclic, calibrate_DAC, input_voltage));
}

analog_out_sink_impl::analog_out_sink_impl(const std::string &uri,
                                           const int buffer_size,
                                           std::vector<double> sampling_frequency,
                                           std::vector<int> oversampling_ratio,
                                           const std::vector<int> &kernel_buffers,
                                           const std::vector<int> &cyclic,
                                           bool calibrate_DAC,
                                           bool input_voltage)
    : gr::sync_block("analog_out_sink",
                     gr::io_signature::make(2, 2, sizeof(float)),
                     gr::io_signature::make(0, 0, 0)),
    d_uri(uri),
    d_buffer_size(buffer_size),
    d_stream_voltage_values(input_voltage)
{
    libm2k::context::M2k *context = analog_in_source_impl::get_context(uri);
    d_analog_out = context->getAnalogOut();

    d_cyclic_buffer = std::vector<bool>(2);
    d_first_iteration = true;

    d_analog_out->stop();
    if (calibrate_DAC) {
        context->calibrateDAC();
    }

    for (int i = 0; i < kernel_buffers.size(); ++i) {
        d_analog_out->setKernelBuffersCount(i, kernel_buffers[i]);
        d_analog_out->setCyclic(i, cyclic[i]);
        d_cyclic_buffer.at(i) = cyclic[i];
        d_analog_out->enableChannel(i, true);
    }

    set_params(sampling_frequency, oversampling_ratio);

    if (!input_voltage) {
        set_input_signature(gr::io_signature::make(2, 2, sizeof(short)));
    }

    set_output_multiple(buffer_size);
}

analog_out_sink_impl::~analog_out_sink_impl()
{
    d_analog_out->stop();
    analog_in_source_impl::remove_contexts(d_uri);
}

void analog_out_sink_impl::set_params(std::vector<double> sampling_frequency,
                                      std::vector<int> oversampling_ratio)
{
    d_analog_out->setSyncedDma(true);
    for (int i = 0; i < 2; ++i) {
        d_analog_out->setSampleRate(i, sampling_frequency[i]);
        d_analog_out->setOversamplingRatio(i, oversampling_ratio[i]);
    }
    d_analog_out->setSyncedDma(false);
}

int analog_out_sink_impl::work(int noutput_items,
                               gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items)
{
    std::vector <std::vector<short>> samples;

    for (unsigned int i = 0; i < input_items.size(); i++) {
        samples.push_back(std::vector<short>());
        if (d_stream_voltage_values) {
            float *temp_samples = (float *) input_items[i];
            for (int j = 0; j < d_buffer_size; j++) {
                samples[i].push_back(d_analog_out->convertVoltsToRaw(0, temp_samples[j]));
            }
        } else {
            short *temp_samples = (short *) input_items[i];
            for (int j = 0; j < d_buffer_size; j++) {
                samples[i].push_back(temp_samples[j]);
            }
        }
    }

    for (unsigned int i = 0; i < input_items.size(); i++) {
        if (d_cyclic_buffer.at(i)) {
            if (d_first_iteration) {
                d_analog_out->pushRaw(i, samples.at(i));
            }
        } else {
            d_analog_out->pushRaw(i, samples.at(i));
        }
    }

    if ((d_cyclic_buffer.at(0) || d_cyclic_buffer.at(1)) && d_first_iteration) {
        d_first_iteration = false;
    }

    consume_each(d_buffer_size);
    return 0;
}

void analog_out_sink_impl::forecast(int noutput_items,
                                    gr_vector_int &ninput_items_required)
{
    for (unsigned int i = 0; i < ninput_items_required.size(); i++)
        ninput_items_required[i] = noutput_items;
}

} // namespace m2k
} // namespace gr
