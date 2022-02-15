#include "mixed_signal_source_impl.h"

#include <libm2k/m2kexceptions.hpp>

using namespace gr::m2k;

mixed_signal_source::sptr
mixed_signal_source::make_from(libm2k::context::M2k *context, int buffer_size, double data_rate, int kb)
{
	return gnuradio::get_initial_sptr(new mixed_signal_source_impl(context, buffer_size, data_rate, kb));
}

mixed_signal_source_impl::mixed_signal_source_impl(libm2k::context::M2k *context, int buffer_size, double data_rate, int kb)
	: gr::sync_block("mixed_device_source"
	, gr::io_signature::make(0, 0, 0)
	, gr::io_signature::make3(3, -1, sizeof(short), sizeof(short), sizeof(unsigned short)))
	, d_m2k_context(context)
	, d_analog_in(context->getAnalogIn())
	, d_digital_in(context->getDigital())
	, d_analog_raw(nullptr)
	, d_digital_raw(nullptr)
	, d_timeout(100)
	, d_buffer_size(buffer_size)
	, d_items_in_buffer(0)
	, d_current_index(0)
	, d_port_id(pmt::mp("msg"))
	, d_please_refill_buffer(false)
	, d_thread_stopped(true)
	, d_current_captured_buffer(0)
	, d_data_rate(data_rate)
	, d_kernel_buffers(kb)
{
	set_output_multiple(0x400);

	message_port_register_out(d_port_id);
}

mixed_signal_source_impl::~mixed_signal_source_impl()
{
	// nothing
}

bool mixed_signal_source_impl::start()
{
	boost::unique_lock<boost::mutex> lock(d_m2k_mutex);

	d_items_in_buffer = 0;
	d_please_refill_buffer = false;
	d_thread_stopped = false;
	d_current_captured_buffer = 0;

	// TODO: take into account buffer_size and max memory on the m2k
	// and compute a number of kernel buffers, or let others set it (scopy)
	d_analog_in->setKernelBuffersCount(d_kernel_buffers);
	d_digital_in->setKernelBuffersCountIn(d_kernel_buffers);

	try {
		d_m2k_context->startMixedSignalAcquisition(d_buffer_size);
	} catch (const libm2k::m2k_exception &e) {
		return false;
	}

	d_refill_thd = boost::thread(&mixed_signal_source_impl::refill_buffer, this);

	return true;
}

bool mixed_signal_source_impl::stop()
{
	d_analog_in->cancelAcquisition();
	d_digital_in->cancelAcquisition();

	boost::unique_lock<boost::mutex> lock(d_m2k_mutex);

	d_thread_stopped = true;
	d_please_refill_buffer = true;
	d_m2k_cond.notify_all();
	lock.unlock();

	if (d_refill_thd.joinable()) {
		d_refill_thd.join();
	}

	d_m2k_context->stopMixedSignalAcquisition();

	return true;
}

int mixed_signal_source_impl::work(int noutput_items,
								   gr_vector_const_void_star &input_items,
								   gr_vector_void_star &output_items)
{
	boost::unique_lock<boost::mutex> lock(d_m2k_mutex);

	if (d_thread_stopped) {
		return -1;
	}

	if (!d_please_refill_buffer && !d_items_in_buffer) {
		d_please_refill_buffer = true;
		d_m2k_cond.notify_all();
	}

	while (d_please_refill_buffer) {
		const bool fast_enough = d_m2k_cond2.timed_wait(lock,
														boost::posix_time::milliseconds(d_timeout));

		if (d_thread_stopped) {
			return -1;
		}

		if (!fast_enough) {
			message_port_pub(d_port_id, pmt::mp("timeout"));
			
			return 0;
		}
	}

	const int items = std::min(d_items_in_buffer, noutput_items);

	for (size_t i = 0; i < output_items.size(); ++i) {

		if (!d_current_index) {
			tag_t tag;
			tag.value = pmt::from_long(d_items_in_buffer);
			tag.offset = nitems_written(i);
			tag.key = pmt::intern("buffer_start");
			tag.srcid = alias_pmt();

			add_item_tag(i, tag);
		}

		if (i <= 1) { // analog
			short *out = static_cast<short*>(output_items[i]);
			for (int j = d_current_index; j < d_current_index + items; ++j) {
				out[j - d_current_index] = d_analog_raw[2 * j + i];
			}
		} else { // digital
			unsigned short *out = static_cast<unsigned short*>(output_items[i]);
			memcpy(out, d_digital_raw + d_current_index, items * sizeof(unsigned short));
		}

	}

	d_items_in_buffer -= items;
	d_current_index += items;

	return items;
}

void mixed_signal_source_impl::set_timeout_ms(unsigned int timeout)
{
	boost::unique_lock<boost::mutex> lock(d_m2k_mutex);

	d_timeout = timeout;
}

void mixed_signal_source_impl::set_data_rate(double rate) {
	d_data_rate = rate;
}

void mixed_signal_source_impl::set_buffer_size(int buffer_size)
{	
	if (d_buffer_size != buffer_size) {
		boost::unique_lock<boost::mutex> lock(d_m2k_mutex);

		d_buffer_size = buffer_size;
		restart();
	}
}

void mixed_signal_source_impl::restart()
{
	if (d_refill_thd.joinable()) {
		d_m2k_context->stopMixedSignalAcquisition();
		d_m2k_context->startMixedSignalAcquisition(d_buffer_size);
		d_current_captured_buffer = 0;
	}
}

void mixed_signal_source_impl::refill_buffer()
{
	boost::unique_lock<boost::mutex> lock(d_m2k_mutex);

	while (!d_thread_stopped) {
		while (!d_please_refill_buffer) {
			d_m2k_cond.wait(lock);
		}

		if (d_thread_stopped) {
			break;
		}

		if (d_current_captured_buffer == d_kernel_buffers) {
			restart();
			d_current_captured_buffer = 0;
		}

		lock.unlock();

		try {
			boost::chrono::high_resolution_clock::time_point t1 ;
			if(d_data_rate) {
				t1 = boost::chrono::high_resolution_clock::now();
			}

			d_analog_raw = d_analog_in->getSamplesRawInterleaved(d_buffer_size);
			d_digital_raw = d_digital_in->getSamplesP(d_buffer_size);

			if(d_data_rate) {
				boost::chrono::duration<double> sec = boost::chrono::high_resolution_clock::now() - t1; // compute getSamplesDuration
				unsigned int frameTime = 1000000.0/d_data_rate;
				unsigned int acquisitionTime = (1000000.0)* sec.count();
				unsigned int diff = (frameTime > acquisitionTime) ? frameTime - acquisitionTime: 0;
				boost::this_thread::sleep_for(boost::chrono::microseconds(diff)); // wait remainder
			}
			lock.lock();
		} catch (const libm2k::m2k_exception &e) {
			if (e.iioCode() != -EBADF) {
				std::cerr << e.what() << std::endl;
			}

			lock.lock();
			break;
		}

		d_please_refill_buffer = false;
		d_items_in_buffer = d_buffer_size;
		d_current_index = 0;
		d_current_captured_buffer++;

		d_m2k_cond2.notify_all();
	}

	d_thread_stopped = true;
	d_m2k_cond2.notify_all();
}
