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

#ifndef INCLUDED_M2K_DIGITAL_IN_SOURCE_IMPL_H
#define INCLUDED_M2K_DIGITAL_IN_SOURCE_IMPL_H

#include <m2k/digital_in_source.h>
#include <libm2k/digital/m2kdigital.hpp>


namespace gr {
namespace m2k {

class digital_in_source_impl : public digital_in_source {
private:
	libm2k::digital::M2kDigital *d_digital;
	const std::string d_uri;
	int d_channel;

	unsigned int d_timeout;
	bool d_deinit;

	unsigned int d_buffer_size;
	unsigned int d_sample_index;
	unsigned long d_items_in_buffer;
	const unsigned short *d_raw_samples;
	double d_data_rate;

	pmt::pmt_t d_port_id;

	gr::thread::mutex d_mutex;
	gr::thread::condition_variable d_cond_wait;
	gr::thread::thread d_refill_thread;
	volatile bool d_empty_buffer{}, d_thread_stopped{};

public:
	digital_in_source_impl(libm2k::context::M2k *context,
						   int buffer_size,
						   const int channel,
						   double sampling_frequency,
						   int kernel_buffers,
						   bool streaming,
						   bool deinit,
						   double data_rate);

	~digital_in_source_impl();

	void refill_buffer();

	int work(int noutput_items,
			 gr_vector_const_void_star &input_items,
			 gr_vector_void_star &output_items);

	bool start() override;
	bool stop() override;

	void set_params(double sampling_frequency, bool streaming) override;

	void set_data_rate(double data) override;

	void set_timeout_ms(unsigned int timeout) override;

	void set_buffer_size(int buffer_size);
};

} // namespace m2k
} // namespace gr

#endif //INCLUDED_M2K_DIGITAL_IN_SOURCE_IMPL_H
