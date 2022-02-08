#ifndef MIXED_SIGNAL_SOURCE_IMPL_H
#define MIXED_SIGNAL_SOURCE_IMPL_H

#include <m2k/mixed_signal_source.h>

#include <libm2k/analog/m2kanalogin.hpp>
#include <libm2k/digital/m2kdigital.hpp>

#include <vector>
#include <ctype.h>

namespace gr {
namespace m2k {
class mixed_signal_source_impl : public mixed_signal_source
{
public: // gr block api
	mixed_signal_source_impl(libm2k::context::M2k *context, int buffer_size, double data_rate);
	~mixed_signal_source_impl();

	bool start() override;
	bool stop() override;

	int work(int noutput_items,
			 gr_vector_const_void_star &input_items,
			 gr_vector_void_star &output_items) override;

public: // mixed_signal_source api
	void set_timeout_ms(unsigned int timeout) override;
	void set_buffer_size(int buffer_size) override;
	void set_data_rate(double rate) override;

private: // private methods
	void restart();
	void refill_buffer();

private:
	libm2k::context::M2k *d_m2k_context;
	libm2k::analog::M2kAnalogIn *d_analog_in;
	libm2k::digital::M2kDigital *d_digital_in;

	const short *d_analog_raw;
	const unsigned short *d_digital_raw;
	
	int d_timeout;
	int d_buffer_size;
	int d_items_in_buffer;
	int d_current_index;
	double d_data_rate;

	pmt::pmt_t d_port_id;

	boost::mutex d_m2k_mutex;
	boost::condition_variable d_m2k_cond, d_m2k_cond2;
	bool d_please_refill_buffer, d_thread_stopped;
	boost::thread d_refill_thd;

	int d_current_captured_buffer;


};
} // namespace m2k
} // namespace gr
#endif // MIXED_SIGNAL_SOURCE_IMPL_H
