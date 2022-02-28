/* -*- c++ -*- */
/*
 * Copyright 2022 Analog Devices Inc..
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_M2K_MIXED_SIGNAL_SOURCE_H
#define INCLUDED_M2K_MIXED_SIGNAL_SOURCE_H


#include <gnuradio/m2k/api.h>
#include <gnuradio/sync_block.h>
#include <libm2k/m2k.hpp>


namespace gr {
  namespace m2k {

  class M2K_API mixed_signal_source : virtual public gr::sync_block {
  public:
	  typedef std::shared_ptr <mixed_signal_source> sptr;

	  static sptr make(libm2k::context::M2k *context, int buffer_size, double data_rate = 0, int kb = 64);
	  static sptr make_from(libm2k::context::M2k *context, int buffer_size, double data_rate = 0, int kb = 64);

	  virtual void set_timeout_ms(unsigned int timeout) = 0;
	  virtual void set_data_rate(double rate) = 0;
	  virtual void set_buffer_size(int buffer_size) = 0;
  };

  } // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_MIXED_SIGNAL_SOURCE_H */
