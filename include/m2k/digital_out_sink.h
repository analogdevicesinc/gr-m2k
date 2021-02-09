/* -*- c++ -*- */
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

#ifndef INCLUDED_M2K_DIGITAL_OUT_SINK_H
#define INCLUDED_M2K_DIGITAL_OUT_SINK_H

#include <m2k/api.h>
#include <gnuradio/sync_block.h>
#include <libm2k/m2k.hpp>

namespace gr {
namespace m2k {

/*!
 * \brief Sink for ADALM2000 with buffered output channels
 * \ingroup m2k
 *
 * \details
 * This block allows for streaming digital data from ADALM2000
 */
class M2K_API digital_out_sink : virtual public gr::sync_block {
public:
typedef boost::shared_ptr <digital_out_sink> sptr;

/*!
 * \brief Return a shared_ptr to a new instance of m2k::digital_out_sink.
 */
static sptr make(const std::string &uri,
		 int buffer_size,
		 const std::vector<int> channels,
		 double sampling_frequency,
		 int kernel_buffers,
		 bool cyclic);

static sptr make_from(libm2k::context::M2k *context,
		      int buffer_size,
		      const std::vector<int> channels,
		      double sampling_frequency,
		      int kernel_buffers,
		      bool cyclic);

virtual void set_params(double sampling_frequency) = 0;

};


} // namespace m2k
} // namespace gr

#endif //INCLUDED_M2K_DIGITAL_OUT_SINK_H
