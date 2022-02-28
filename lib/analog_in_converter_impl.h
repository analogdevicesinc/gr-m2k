/* -*- c++ -*- */
/*
 * Copyright 2022 Analog Devices Inc..
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_M2K_ANALOG_IN_CONVERTER_IMPL_H
#define INCLUDED_M2K_ANALOG_IN_CONVERTER_IMPL_H

#include <gnuradio/m2k/analog_in_converter.h>
#include <libm2k/m2k.hpp>
#include <libm2k/analog/m2kanalogin.hpp>

namespace gr {
  namespace m2k {

    class analog_in_converter_impl : public analog_in_converter
    {
    private:
	    libm2k::analog::M2kAnalogIn *d_analog_in;

    public:
	    analog_in_converter_impl(libm2k::context::M2k *context);

	    ~analog_in_converter_impl();

	    int work(int noutput_items,
			     gr_vector_const_void_star &input_items,
			     gr_vector_void_star &output_items);
    };

  } // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_ANALOG_IN_CONVERTER_IMPL_H */
