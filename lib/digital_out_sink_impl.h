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

#ifndef INCLUDED_M2K_DIGITAL_OUT_SINK_IMPL_H
#define INCLUDED_M2K_DIGITAL_OUT_SINK_IMPL_H

#include <m2k/digital_out_sink.h>
#include <libm2k/digital/m2kdigital.hpp>


namespace gr {
namespace m2k {

class digital_out_sink_impl : public digital_out_sink {
private:
	libm2k::digital::M2kDigital *d_digital;
	const std::string d_uri;
	const int d_buffer_size;
	std::vector<unsigned short> m_samples;

public:
	digital_out_sink_impl(libm2k::context::M2k *context,
			       int buffer_size,
			       const std::vector<int> channels,
			       double sampling_frequency,
			       int kernel_buffers,
			       bool cyclic);

	~digital_out_sink_impl();


	int work(int noutput_items,
		 gr_vector_const_void_star &input_items,
		 gr_vector_void_star &output_items);

	void forecast(int noutput_items, gr_vector_int &ninput_items_required);

	void set_params(double sampling_frequency);
};

} // namespace m2k
} // namespace gr

#endif //INCLUDED_M2K_DIGITAL_OUT_SINK_IMPL_H
