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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "digital_in_source_impl.h"
#include "analog_in_source_impl.h"
#include <gnuradio/io_signature.h>
#include <libm2k/m2k.hpp>
#include <boost/lexical_cast.hpp>
#include <libm2k/m2kexceptions.hpp>


namespace gr {
namespace m2k {

digital_in_source::sptr
digital_in_source::make(const std::string &uri,
						int buffer_size,
						const int channel,
						double sampling_frequency,
						int kernel_buffers,
						bool streaming,
						bool deinit,
						double data_rate)
{
	return gnuradio::get_initial_sptr
			(new digital_in_source_impl(analog_in_source_impl::get_context(uri), buffer_size, channel, sampling_frequency, kernel_buffers, streaming, deinit, data_rate));
}

digital_in_source::sptr
digital_in_source::make_from(libm2k::context::M2k *context,
							 int buffer_size,
							 const int channel,
							 double sampling_frequency,
							 int kernel_buffers,
							 bool streaming,
							 bool deinit,
							 double data_rate)
{
	return gnuradio::get_initial_sptr
			(new digital_in_source_impl(context, buffer_size, channel, sampling_frequency, kernel_buffers, streaming, deinit, data_rate));
}

digital_in_source_impl::digital_in_source_impl(libm2k::context::M2k *context,
											   int buffer_size,
											   const int channel,
											   double sampling_frequency,
											   int kernel_buffers,
											   bool streaming,
											   bool deinit,
											   double data_rate)
	: gr::sync_block("digital_in_source",
					 gr::io_signature::make(0, 0, 0),
					 gr::io_signature::make(1, 1, sizeof(uint16_t))),
	  d_uri(context->getUri()),
	  d_buffer_size(buffer_size),
	  d_channel(channel),
	  d_timeout(100),
	  d_deinit(deinit),
	  d_port_id(pmt::mp("msg")),
	  d_data_rate(data_rate)
{
	analog_in_source_impl::add_context(context);
	d_digital = context->getDigital();

	d_digital->setKernelBuffersCountIn(kernel_buffers);
	set_params(sampling_frequency, streaming);

	d_items_in_buffer = 0;
	set_output_multiple(0x400);
	message_port_register_out(d_port_id);
}

digital_in_source_impl::~digital_in_source_impl()
{
	if (d_deinit) {
		analog_in_source_impl::remove_contexts(d_uri);
	}
}

void digital_in_source_impl::set_params(double sampling_frequency, bool streaming)
{
	d_digital->setSampleRateIn(sampling_frequency);
	auto trigger = d_digital->getTrigger();
	trigger->setDigitalStreamingFlag(streaming);
}

void digital_in_source_impl::refill_buffer()
{
	boost::unique_lock <boost::mutex> lock(d_mutex);

	while (true) {
		if (d_thread_stopped) {
			break;
		}
		d_cond_wait.wait(lock, [&] { return d_empty_buffer; });

		try {
			lock.unlock();
			boost::chrono::high_resolution_clock::time_point t1 ;
			if(d_data_rate) {
				t1 = boost::chrono::high_resolution_clock::now();
			}

			d_raw_samples = d_digital->getSamplesP(d_buffer_size);

			if(d_data_rate) {
				boost::chrono::duration<double> sec = boost::chrono::high_resolution_clock::now() - t1; // compute getSamplesDuration
				unsigned int frameTime = 1000000.0/d_data_rate;
				unsigned int acquisitionTime = (1000000.0)* sec.count();
				unsigned int diff = (frameTime > acquisitionTime) ? frameTime - acquisitionTime: 0;
				boost::this_thread::sleep_for(boost::chrono::microseconds(diff)); // wait remainder
			}
			lock.lock();
		} catch (libm2k::m2k_exception &e) {
			if (e.iioCode() != -EBADF) {
				std::cout << e.what() << std::endl;
			}
			d_thread_stopped = true;
			break;
		}
		d_items_in_buffer = (unsigned long) d_buffer_size;
		d_sample_index = 0;
		d_empty_buffer = false;
		d_cond_wait.notify_one();
	}
}

int digital_in_source_impl::work(int noutput_items,
								 gr_vector_const_void_star &input_items,
								 gr_vector_void_star &output_items)
{
	boost::unique_lock <boost::mutex> lock(d_mutex);
	if (d_thread_stopped) {
		return -1;
	}
	if (!d_items_in_buffer) {
		d_empty_buffer = true;
		d_cond_wait.notify_one();
	}

	while (d_empty_buffer) {
		// use wait_for to avoid permanent blocking in the work function
		bool buffer_refilled = d_cond_wait.wait_for(lock, boost::chrono::milliseconds(d_timeout),
													[&] { return !d_empty_buffer; });
		if (d_thread_stopped) {
			return -1;
		}

		if (!buffer_refilled) {
			message_port_pub(d_port_id, pmt::mp("timeout"));
			return 0;
		}
	}

	unsigned long nb_samples = std::min(d_items_in_buffer, (unsigned long) noutput_items);
	unsigned int out_stream_index = 0;

	if (!d_sample_index) {
		tag_t tag;
		tag.value = pmt::from_long(d_items_in_buffer);
		tag.offset = nitems_written(out_stream_index);
		tag.key = pmt::intern("buffer_start");
		tag.srcid = alias_pmt();

		add_item_tag(out_stream_index, tag);
	}
	uint16_t *out = (uint16_t *) output_items[out_stream_index];
	memcpy(out, d_raw_samples + d_sample_index, sizeof(uint16_t) * nb_samples);
	d_items_in_buffer -= nb_samples;
	d_sample_index += nb_samples;

	return (int) nb_samples;
}

void digital_in_source_impl::set_timeout_ms(unsigned int timeout)
{
	if (d_timeout != timeout) {
		boost::unique_lock<boost::mutex> lock(d_mutex);
		d_timeout = timeout;
	}
}

void digital_in_source_impl::set_data_rate(double rate) {
	d_data_rate = rate;
}

void digital_in_source_impl::set_buffer_size(int buffer_size)
{
	if (d_buffer_size != buffer_size) {
		boost::unique_lock<boost::mutex> lock(d_mutex);

		d_items_in_buffer = 0;
		d_buffer_size = buffer_size;
	}
}

bool digital_in_source_impl::start()
{
	boost::unique_lock <boost::mutex> lock(d_mutex);

	d_items_in_buffer = 0;
	d_empty_buffer = true;
	d_thread_stopped = false;
	d_refill_thread = gr::thread::thread(boost::bind(&digital_in_source_impl::refill_buffer, this));

	return true;
}

bool digital_in_source_impl::stop()
{
	d_digital->cancelAcquisition();
	boost::unique_lock <boost::mutex> lock(d_mutex);
	d_empty_buffer = true;
	d_thread_stopped = true;
	d_cond_wait.notify_one();
	lock.unlock();
	d_refill_thread.join();
	d_digital->stopAcquisition();
	return true;
}

} /* namespace m2k */
} /* namespace gr */
