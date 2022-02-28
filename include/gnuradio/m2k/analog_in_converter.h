/* -*- c++ -*- */
/*
 * Copyright 2022 Analog Devices Inc..
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_M2K_ANALOG_IN_CONVERTER_H
#define INCLUDED_M2K_ANALOG_IN_CONVERTER_H

#include <gnuradio/m2k/api.h>
#include <gnuradio/sync_block.h>
#include <libm2k/m2k.hpp>

namespace gr {
  namespace m2k {


  /*!
   * \brief Raw ADC samples converter
   * \ingroup m2k
   *
   * \details
   * This block converts raw samples of ADALM2000's ADC into voltage
   */
  class M2K_API analog_in_converter : virtual public gr::sync_block {
  public:
	  typedef std::shared_ptr <analog_in_converter> sptr;

	  /*!
	   * \brief Return a shared_ptr to a new instance of m2k::analog_in_converter.
	   *
	   * \param uri String of the context uri
	   */
	  static sptr make(const std::string &uri);

	  static sptr make_from(libm2k::context::M2k *context);

  };

  } // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_ANALOG_IN_CONVERTER_H */
