/* -*- c++ -*- */
/*
 * Copyright 2022 Analog Devices Inc..
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_M2K_DIGITAL_IN_SOURCE_H
#define INCLUDED_M2K_DIGITAL_IN_SOURCE_H

#include <gnuradio/m2k/api.h>
#include <gnuradio/sync_block.h>
#include <libm2k/m2k.hpp>

namespace gr {
  namespace m2k {

  /*!
   * \brief Source for ADALM2000 with buffered output channels
   * \ingroup m2k
   *
   * \details
   * This block allows for streaming digital data from ADALM2000
   */
  class M2K_API digital_in_source : virtual public gr::sync_block {
  public:
	  typedef std::shared_ptr <digital_in_source> sptr;

	  /*!
   * \brief Return a shared_ptr to a new instance of m2k::digital_in_source.
   */
	  static sptr make(const std::string &uri,
					   int buffer_size,
					   int channel,
					   double sampling_frequency,
					   int kernel_buffers,
					   bool streaming,
					   bool deinit = true,
					   double data_rate = 0);

	  static sptr make_from(libm2k::context::M2k *context,
						    int buffer_size,
						    int channel,
						    double sampling_frequency,
						    int kernel_buffers,
						    bool streaming,
						    bool deinit = true,
						    double data_rate = 0);

	  virtual void set_params(double sampling_frequency, bool streaming) = 0;

	  virtual void set_data_rate(double rate) = 0;

	  virtual void set_timeout_ms(unsigned int timeout) = 0;

	  virtual void set_buffer_size(int buffer_size) = 0;

  };
  } // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_DIGITAL_IN_SOURCE_H */
