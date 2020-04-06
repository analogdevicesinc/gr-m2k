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

#include "analog_in_source_impl.h"
#include <gnuradio/io_signature.h>
#include <libm2k/m2k.hpp>
#include <libm2k/contextbuilder.hpp>
#include <libm2k/analog/m2kanalogin.hpp>
#include <boost/lexical_cast.hpp>


namespace gr {
namespace m2k {

analog_in_source::sptr
analog_in_source::make(const std::string &uri,
                       int buffer_size,
                       const std::vector<int> &channels,
                       std::vector<int> ranges,
                       double sampling_frequency,
                       int oversampling_ratio,
                       int kernel_buffers,
                       bool calibrate_ADC,
                       bool stream_voltage_values,
                       std::vector<int> trigger_condition,
                       std::vector<int> trigger_mode,
                       int trigger_source,
                       int trigger_delay,
                       std::vector<double> trigger_level)
{
    return gnuradio::get_initial_sptr
        (new analog_in_source_impl(uri, buffer_size, channels, ranges, sampling_frequency, oversampling_ratio,
                                   kernel_buffers,
                                   calibrate_ADC, stream_voltage_values, trigger_condition, trigger_mode,
                                   trigger_source,
                                   trigger_delay, trigger_level));
}

analog_in_source_impl::analog_in_source_impl(const std::string &uri,
                                             int buffer_size,
                                             const std::vector<int> &channels,
                                             std::vector<int> ranges,
                                             double sampling_frequency,
                                             int oversampling_ratio,
                                             int kernel_buffers,
                                             bool calibrate_ADC,
                                             bool stream_voltage_values,
                                             std::vector<int> trigger_condition,
                                             std::vector<int> trigger_mode,
                                             int trigger_source,
                                             int trigger_delay,
                                             std::vector<double> trigger_level)
    : gr::sync_block("analog_in_source",
                     gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1, 2, sizeof(float))),
    d_uri(uri),
    d_buffer_size(buffer_size),
    d_channels(channels),
    d_stream_voltage_values(stream_voltage_values)
{
    libm2k::context::M2k *context = analog_in_source_impl::get_context(uri);
    d_analog_in = context->getAnalogIn();

    d_analog_in->setKernelBuffersCount(kernel_buffers);
    set_params(ranges, sampling_frequency, oversampling_ratio);
    set_trigger(trigger_condition, trigger_mode, trigger_source, trigger_delay, trigger_level);

    if (calibrate_ADC) {
        context->calibrateADC();
    }

    if (!stream_voltage_values) {
        set_output_signature(gr::io_signature::make(1, 2, sizeof(short)));
    }

    d_items_in_buffer = 0;
    set_output_multiple(0x400);
}

analog_in_source_impl::~analog_in_source_impl()
{
    remove_contexts(d_uri);
}

void analog_in_source_impl::set_params(std::vector<int> ranges,
                                       double sampling_frequency,
                                       int oversampling_ratio)
{
    for (int i = 0; i < d_channels.size(); i++) {
        if (d_channels.at(i)) {
            d_analog_in->enableChannel(i, true);
            d_analog_in->setRange(static_cast<libm2k::analog::ANALOG_IN_CHANNEL>(i),
                                  (static_cast<libm2k::analog::M2K_RANGE>(ranges[i])));
        }
    }

    d_analog_in->setSampleRate(sampling_frequency);
    d_analog_in->setOversamplingRatio(oversampling_ratio);
}

void analog_in_source_impl::set_trigger(std::vector<int> trigger_condition,
                                        std::vector<int> trigger_mode,
                                        int trigger_source,
                                        int trigger_delay,
                                        std::vector<double> trigger_level)
{
    libm2k::M2kHardwareTrigger *trigger = d_analog_in->getTrigger();

    for (int i = 0; i < d_channels.size(); ++i) {
        if (d_channels.at(i)) {
            trigger->setAnalogCondition(i, static_cast<libm2k::M2K_TRIGGER_CONDITION_ANALOG>(trigger_condition[i]));
            trigger->setAnalogMode(i, static_cast<libm2k::M2K_TRIGGER_MODE>(trigger_mode[i]));
            trigger->setAnalogLevel(i, trigger_level[i]);
        }
    }
    trigger->setAnalogSource(static_cast<libm2k::M2K_TRIGGER_SOURCE_ANALOG>(trigger_source));
    trigger->setAnalogDelay(trigger_delay);
}

libm2k::context::M2k *analog_in_source_impl::get_context(const std::string &uri)
{
    auto element = s_contexts.find(uri);
    if (element == s_contexts.end()) {
	libm2k::context::M2k *ctx = libm2k::context::m2kOpen(uri.c_str());
        if (ctx == nullptr) {
            throw std::runtime_error("Unable to create the context!");
        }
	s_contexts.insert(std::pair<std::string, libm2k::context::M2k *>(ctx->getUri(), ctx));
        return ctx;
    }
    return element->second;
}

void analog_in_source_impl::remove_contexts(const std::string &uri)
{
    boost::lock_guard <boost::mutex> lock(s_ctx_mutex);
    auto element = s_contexts.find(uri);
    if (element != s_contexts.end()) {
	libm2k::context::contextClose(element->second, true);
        s_contexts.erase(element);
    }
}


int analog_in_source_impl::work(int noutput_items,
                                gr_vector_const_void_star &input_items,
                                gr_vector_void_star &output_items)
{
    if (!d_items_in_buffer) {
        try {
            d_raw_samples = d_analog_in->getSamplesRawInterleaved(d_buffer_size);
        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
            return -1;
        }
        d_items_in_buffer = (unsigned long) d_buffer_size;
        d_sample_index = 0;
    }

    unsigned long nb_samples = std::min(d_items_in_buffer, (unsigned long) noutput_items);
    unsigned int sample_index, channel_index, out_stream_index;

    for (out_stream_index = 0, channel_index = 0;
         out_stream_index < output_items.size(); out_stream_index++, channel_index++) {
        while (!d_channels[channel_index]) {
            channel_index++;
        }

        if (!d_sample_index) {
            tag_t tag;
            tag.value = pmt::from_long(d_items_in_buffer);
            tag.offset = nitems_written(out_stream_index);
            tag.key = pmt::intern("buffer_start");
            tag.srcid = alias_pmt();

            add_item_tag(out_stream_index, tag);
        }

        if (d_stream_voltage_values) {
            float *out = (float *) output_items[out_stream_index];
            for (sample_index = 0; sample_index < nb_samples; sample_index++) {
                out[sample_index] = boost::lexical_cast<float>(d_analog_in->convertRawToVolts(channel_index, d_raw_samples[(2*(sample_index + d_sample_index)) + out_stream_index]));

            }
        } else {
            short *out = (short *) output_items[out_stream_index];
            for (sample_index = 0; sample_index < nb_samples; sample_index++) {
                out[sample_index] = d_raw_samples[(2*(sample_index + d_sample_index)) + out_stream_index];
            }
        }
    }
    d_items_in_buffer -= nb_samples;
    d_sample_index += nb_samples;

    return (int) nb_samples;
}

} /* namespace m2k */
} /* namespace gr */
