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

#include "digital_in_source_impl.h"
#include "analog_in_source_impl.h"
#include <gnuradio/io_signature.h>
#include <libm2k/m2k.hpp>
#include <boost/lexical_cast.hpp>

#include "block_communication.h"

namespace gr {
namespace m2k {

digital_in_source::sptr
digital_in_source::make(const std::string &uri,
			int buffer_size,
			const int channel,
			double sampling_frequency,
			int kernel_buffers, bool sync)
{
	return gnuradio::get_initial_sptr
		(new digital_in_source_impl(uri, buffer_size, channel, sampling_frequency, kernel_buffers, sync));
}

digital_in_source_impl::digital_in_source_impl(const std::string &uri,
					       int buffer_size,
					       const int channel,
					       double sampling_frequency,
					       int kernel_buffers, bool sync)
	: gr::sync_block("digital_in_source",
			 gr::io_signature::make(0, 0, 0),
			 gr::io_signature::make(1, 2, sizeof(unsigned short))),
	d_uri(uri),
	d_buffer_size(buffer_size),
	d_channel(channel)
{
	libm2k::context::M2k *context = analog_in_source_impl::get_context(uri);
	d_digital = context->getDigital();

	d_digital->setKernelBuffersCountIn(kernel_buffers);
	set_params(sampling_frequency);

	d_items_in_buffer = 0;
	set_output_multiple(0x400);

	set_sync_with_analog(sync);
}

digital_in_source_impl::~digital_in_source_impl()
{
	analog_in_source_impl::remove_contexts(d_uri);
}

void digital_in_source_impl::set_params(double sampling_frequency)
{
	d_digital->setSampleRateIn(sampling_frequency);
}

int digital_in_source_impl::work(int noutput_items,
				gr_vector_const_void_star &input_items,
				gr_vector_void_star &output_items)
{

//	static int attempt = 0;
//	if (attempt < 5) {
//		attempt++;
//		std::cerr << "Trying again next work!!!";
//		return 0;
//	}

//	attempt = 0;

//	std::cerr << "d_items_in_buffer: " << d_items_in_buffer << std::endl;
	if (!d_items_in_buffer) {
		try {
			if (block_communication::get_instance().is_sync_requested(d_uri)) {
				if (block_communication::get_instance().can_capture(d_uri, block_communication::SYNC_DEVICE::DIGITAL)) {
					d_raw_samples = d_digital->getSamplesP(d_buffer_size);
					block_communication::get_instance().data_captured(d_uri, block_communication::SYNC_DEVICE::DIGITAL);
					std::cerr << "Captured data LOGIC!" << std::endl;
				} else {
					/* Can't capture yet return 0 items produced,
					 work will be called again */
//					std::cerr << "Waiting for analog to capture!";
					return 0;
				}
			} else {
				d_raw_samples = d_digital->getSamplesP(d_buffer_size);
			}
		} catch (std::exception &e) {
			std::cout << e.what() << std::endl;
			return -1;
		}
		d_items_in_buffer = (unsigned long) d_buffer_size;
		d_sample_index = 0;

//		std::cerr << "d_items_in_buffer: " << d_items_in_buffer << std::endl;
	}

	unsigned long nb_samples = std::min(d_items_in_buffer, (unsigned long) noutput_items);
	unsigned int sample_index, channel_index, out_stream_index;

	
	if (!d_sample_index) {
		tag_t tag;
		tag.value = pmt::from_long(d_items_in_buffer);
		tag.offset = nitems_written(out_stream_index);
		tag.key = pmt::intern("buffer_start");
		tag.srcid = alias_pmt();

		add_item_tag(out_stream_index, tag);
	}
	
	short *out = (short *) output_items[0];
	memcpy(out, d_raw_samples + d_sample_index, sizeof(short) * nb_samples);
	
	d_items_in_buffer -= nb_samples;
	d_sample_index += nb_samples;

	return (int) nb_samples;
}

unsigned short digital_in_source_impl::get_channel_value(unsigned int channel, unsigned short sample)
{
	return (sample & (1u << channel )) >> channel;
}

void digital_in_source_impl::set_sync_with_analog(bool sync)
{
	block_communication::get_instance().request_sync_between_analog_digital(d_uri, true, d_buffer_size);
}

} /* namespace m2k */
} /* namespace gr */
