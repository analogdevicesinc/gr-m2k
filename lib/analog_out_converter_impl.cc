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

#include "analog_out_converter_impl.h"
#include "analog_in_source_impl.h"
#include <boost/lexical_cast.hpp>


namespace gr {
namespace m2k {

analog_out_converter::sptr
analog_out_converter::make(const std::string &uri)
{
	return gnuradio::get_initial_sptr
			(new analog_out_converter_impl(analog_in_source_impl::get_context(uri)));
}

analog_out_converter::sptr
analog_out_converter::make_from(libm2k::context::M2k *context)
{
	return gnuradio::get_initial_sptr
			(new analog_out_converter_impl(context));
}

analog_out_converter_impl::analog_out_converter_impl(libm2k::context::M2k *context)
	: gr::sync_block("analog_out_converter",
					 gr::io_signature::make(1, 1, sizeof(float)),
					 gr::io_signature::make(1, 1, sizeof(short)))
{
	d_analog_out = context->getAnalogOut();
}


analog_out_converter_impl::~analog_out_converter_impl()
{
}


int analog_out_converter_impl::work(int noutput_items,
									gr_vector_const_void_star &input_items,
									gr_vector_void_star &output_items)
{
	int sample_index, channel;
	float *in;
	short *out;
	for (channel = 0; channel < output_items.size(); channel++) {
		in = (float *) input_items[channel];
		out = (short *) output_items[channel];
		for (sample_index = 0; sample_index < noutput_items; sample_index++) {
			out[sample_index] = boost::lexical_cast<short>(
						d_analog_out->convertVoltsToRaw(channel, in[sample_index]));
		}
	}
	return noutput_items;
}

} // namespace m2k
} // namespace gr
