/*
 * Copyright 2021 Analog Devices Inc.
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

#include "digital_out_sink_impl.h"
#include "analog_in_source_impl.h"
#include <gnuradio/io_signature.h>
#include <libm2k/m2k.hpp>
#include <boost/lexical_cast.hpp>


namespace gr {
namespace m2k {

digital_out_sink::sptr
digital_out_sink::make(const std::string &uri,
		       int buffer_size,
		       const std::vector<int> channels,
		       double sampling_frequency,
		       int kernel_buffers,
		       bool cyclic)
{
	return gnuradio::get_initial_sptr
		(new digital_out_sink_impl(analog_in_source_impl::get_context(uri), buffer_size, channels, sampling_frequency, kernel_buffers, cyclic));
}

digital_out_sink::sptr
digital_out_sink::make_from(libm2k::context::M2k *context,
			    int buffer_size,
			    const std::vector<int> channels,
			    double sampling_frequency,
			    int kernel_buffers,
			    bool cyclic)
{
	return gnuradio::get_initial_sptr
		(new digital_out_sink_impl(context, buffer_size, channels, sampling_frequency, kernel_buffers, cyclic));
}

digital_out_sink_impl::digital_out_sink_impl(libm2k::context::M2k *context,
					     int buffer_size,
					     const std::vector<int> channels,
					     double sampling_frequency,
					     int kernel_buffers,
					     bool cyclic)
	: gr::sync_block("digital_out_sink",
			 gr::io_signature::make(1, 1, sizeof(unsigned short)),
			 gr::io_signature::make(0, 0, 0)),
	  d_uri(context->getUri()),
	  d_buffer_size(buffer_size)
{
	d_digital = context->getDigital();

	d_digital->stopBufferOut();

	d_digital->setKernelBuffersCountOut(kernel_buffers);
	d_digital->setCyclic(cyclic);

	for (auto ch : channels) {
		d_digital->setDirection(ch, libm2k::digital::DIO_OUTPUT);
		d_digital->enableChannel(ch, true);
	}

	set_params(sampling_frequency);

	set_output_multiple(buffer_size);
}

digital_out_sink_impl::~digital_out_sink_impl()
{
	d_digital->stopBufferOut();
	analog_in_source_impl::remove_contexts(d_uri);
}

void digital_out_sink_impl::set_params(double sampling_frequency)
{
	d_digital->setSampleRateOut(sampling_frequency);
}

int digital_out_sink_impl::work(int noutput_items,
			       gr_vector_const_void_star &input_items,
			       gr_vector_void_star &output_items)
{
	unsigned short *temp_samples = (unsigned short *) input_items[0];
	int consumed_data = 0;
	while (m_samples.size() < d_buffer_size || consumed_data < noutput_items) {
		m_samples.push_back(temp_samples[consumed_data]);
		consumed_data++;
	}

	if (m_samples.size() == d_buffer_size) {
		d_digital->push(m_samples);
		m_samples.clear();
	}

	consume_each(consumed_data);
	return 0;
}

void digital_out_sink_impl::forecast(int noutput_items,
				    gr_vector_int &ninput_items_required)
{
	for (unsigned int i = 0; i < ninput_items_required.size(); i++)
		ninput_items_required[i] = noutput_items;
}

} // namespace m2k
} // namespace gr
