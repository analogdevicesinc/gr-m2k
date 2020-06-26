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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mixed_signal_source_impl.h"
#include "analog_in_source_impl.h"
#include <gnuradio/io_signature.h>
#include <libm2k/m2k.hpp>
#include <boost/lexical_cast.hpp>

namespace gr {
    namespace m2k {

        mixed_signal_source::sptr
        mixed_signal_source::make(const std::string &uri,
                               int buffer_size,
                               double sampling_frequency,
                               int oversampling_ratio,
                               int kernel_buffers,
                               int trigger_condition,
                               int trigger_mode,
                               int trigger_source,
                               int trigger_delay,
                               double trigger_level)
        {
            return gnuradio::get_initial_sptr
                    (new mixed_signal_source_impl(uri, buffer_size, sampling_frequency, oversampling_ratio,
                                               kernel_buffers, trigger_condition, trigger_mode,
                                               trigger_source, trigger_delay, trigger_level));
        }

        mixed_signal_source_impl::mixed_signal_source_impl(const std::string &uri,
                                                           int buffer_size,
                                                           double sampling_frequency,
                                                           int oversampling_ratio,
                                                           int kernel_buffers,
                                                           int trigger_condition,
                                                           int trigger_mode,
                                                           int trigger_source,
                                                           int trigger_delay,
                                                           double trigger_level)
                : gr::sync_block("mixed_signal_source",
                                 gr::io_signature::make(0, 0, 0),
                                 gr::io_signature::make(2, 2, sizeof(float))),
                  d_uri(uri),
                  d_buffer_size(buffer_size),
                  d_kernel_buffers(kernel_buffers)
        {
            libm2k::context::M2k *context = analog_in_source_impl::get_context(uri);
            d_analog_in = context->getAnalogIn();
            d_digital = context->getDigital();

            d_analog_in->setKernelBuffersCount(kernel_buffers);
	        d_digital->setKernelBuffersCountIn(kernel_buffers);

            d_analog_in->enableChannel(0, true);
            d_analog_in->enableChannel(1, true);

            d_analog_in->setSampleRate(sampling_frequency);
            d_analog_in->setOversamplingRatio(oversampling_ratio);
            d_digital->setSampleRateIn(sampling_frequency);

            libm2k::M2kHardwareTrigger *trigger = d_analog_in->getTrigger();

            trigger->setAnalogCondition(0, static_cast<libm2k::M2K_TRIGGER_CONDITION_ANALOG>(trigger_condition));
            trigger->setAnalogMode(0, static_cast<libm2k::M2K_TRIGGER_MODE>(trigger_mode));
            trigger->setAnalogLevel(0, trigger_level);

            trigger->setAnalogSource(static_cast<libm2k::M2K_TRIGGER_SOURCE_ANALOG>(trigger_source));
            trigger->setAnalogDelay(trigger_delay);
            trigger->setAnalogStreamingFlag(true);

            context->calibrateADC();

            d_items_in_buffer = 0;
            set_output_multiple(0x400);
        }

        mixed_signal_source_impl::~mixed_signal_source_impl()
        {
            analog_in_source_impl::remove_contexts(d_uri);
        }

        int mixed_signal_source_impl::work(int noutput_items,
                                        gr_vector_const_void_star &input_items,
                                        gr_vector_void_star &output_items)
        {
            if (!d_items_in_buffer) {
                try {
                    d_digital_raw_samples = d_digital->getSamplesP(d_buffer_size);
                    d_analogical_raw_samples = d_analog_in->getSamplesRawInterleaved(d_buffer_size);
                } catch (std::exception &e) {
                    std::cout << e.what() << std::endl;
                    return -1;
                }
                d_items_in_buffer = (unsigned long) d_buffer_size;
                d_sample_index = 0;
            }

            unsigned long nb_samples = std::min(d_items_in_buffer, (unsigned long) noutput_items);
            unsigned int sample_index, channel_index, out_stream_index;

            float *out_ain = (float *) output_items[0];
            float *out_dig = (float *) output_items[1];
            for (int i = 0; i < nb_samples; ++i) {
                out_ain[i] = boost::lexical_cast<float>(d_analog_in->convertRawToVolts(0, d_analogical_raw_samples[(2*(i + d_sample_index))]));
                out_dig[i] = get_channel_value(0, d_digital_raw_samples[d_sample_index + i]);
            }

            d_items_in_buffer -= nb_samples;
            d_sample_index += nb_samples;
            return (int) nb_samples;
        }

unsigned short mixed_signal_source_impl::get_channel_value(unsigned int channel, unsigned short sample)
{
	return (sample & (1u << channel )) >> channel;
}
    } /* namespace m2k */
} /* namespace gr */
