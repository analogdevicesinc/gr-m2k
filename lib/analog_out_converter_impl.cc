/* -*- c++ -*- */
/*
 * Copyright 2022 Analog Devices Inc..
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "analog_out_converter_impl.h"
#include "analog_in_source_impl.h"
#include <boost/lexical_cast.hpp>

namespace gr {
  namespace m2k {

    using input_type = float;
    using output_type = short;

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
  } /* namespace m2k */
} /* namespace gr */
