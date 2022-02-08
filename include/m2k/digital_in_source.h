/* -*- c++ -*- */
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

#ifndef INCLUDED_M2K_DIGITAL_IN_SOURCE_H
#define INCLUDED_M2K_DIGITAL_IN_SOURCE_H

#include <m2k/api.h>
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
	typedef boost::shared_ptr <digital_in_source> sptr;

	/*!
 * \brief Return a shared_ptr to a new instance of m2k::digital_in_source.
 */
	static sptr make(const std::string &uri,
					 int buffer_size,
					 int channel,
					 double sampling_frequency,
					 int kernel_buffers,
					 bool streaming,
					 bool deinit = true);

	static sptr make_from(libm2k::context::M2k *context,
						  int buffer_size,
						  int channel,
						  double sampling_frequency,
						  int kernel_buffers,
						  bool streaming,
						  bool deinit = true);

	virtual void set_params(double sampling_frequency, bool streaming) = 0;

	virtual  void set_timeout_ms(unsigned int timeout) = 0;

	virtual void set_buffer_size(int buffer_size) = 0;

};


} // namespace m2k
} // namespace gr

#endif //INCLUDED_M2K_DIGITAL_IN_SOURCE_H
