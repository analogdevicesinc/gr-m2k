/* -*- c++ -*- */
/*
 * Copyright 2022 Analog Devices Inc..
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "analog_in_converter_impl.h"
#include "analog_in_source_impl.h"
#include <gnuradio/io_signature.h>
#include <boost/lexical_cast.hpp>

namespace gr {
  namespace m2k {

using input_type = short;
using output_type = float;

analog_in_converter::sptr
analog_in_converter::make(const std::string &uri)
{
  return gnuradio::get_initial_sptr
	  (new analog_in_converter_impl(analog_in_source_impl::get_context(uri)));
}

analog_in_converter::sptr
analog_in_converter::make_from(libm2k::context::M2k *context)
{
  return gnuradio::get_initial_sptr
	  (new analog_in_converter_impl(context));
}

  /*
     * The private constructor
     */

analog_in_converter_impl::analog_in_converter_impl(libm2k::context::M2k *context)
	: gr::sync_block("analog_in_converter",
					 gr::io_signature::make(1, 1, sizeof(short)),
					 gr::io_signature::make(1, 1, sizeof(float)))
{
	d_analog_in = context->getAnalogIn();
}


analog_in_converter_impl::~analog_in_converter_impl()
{
}

int analog_in_converter_impl::work(int noutput_items,
								   gr_vector_const_void_star &input_items,
								   gr_vector_void_star &output_items)
{
	int sample_index, channel;
	short *in;
	float *out;
	for (channel = 0; channel < output_items.size(); channel++) {
		in = (short *) input_items[channel];
		out = (float *) output_items[channel];
		for (sample_index = 0; sample_index < noutput_items; sample_index++) {
			out[sample_index] = boost::lexical_cast<float>(
						d_analog_in->convertRawToVolts(channel, in[sample_index]));
		}
	}
	return noutput_items;
}
  } /* namespace m2k */
} /* namespace gr */
