/* -*- c++ -*- */
/*
 * Copyright 2022 Analog Devices Inc..
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_M2K_ANALOG_OUT_SINK_IMPL_H
#define INCLUDED_M2K_ANALOG_OUT_SINK_IMPL_H

#include <gnuradio/m2k/analog_out_sink.h>
#include <libm2k/analog/m2kanalogout.hpp>

namespace gr {
  namespace m2k {

    class analog_out_sink_impl : public analog_out_sink
    {
    private:
	    libm2k::analog::M2kAnalogOut *d_analog_out;
	    const std::string d_uri;
	    const int d_buffer_size;
	    bool d_stream_voltage_values;
	    std::vector<bool> d_cyclic_buffer;
	    bool d_first_iteration;

    public:
	    analog_out_sink_impl(const std::string &uri,
						     const int buffer_size,
						     std::vector<double> sampling_frequency,
						     std::vector<int> oversampling_ratio,
						     const std::vector<int> &kernel_buffers,
						     const std::vector<int> &cyclic,
						     bool calibrate_DAC,
						     bool input_voltage);

	    ~analog_out_sink_impl();

	    void set_params(std::vector<double> sampling_frequency,
					    std::vector<int> oversampling_ratio);

	    int work(int noutput_items,
			     gr_vector_const_void_star &input_items,
			     gr_vector_void_star &output_items);

	    void forecast(int noutput_items, gr_vector_int &ninput_items_required);
    };

  } // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_ANALOG_OUT_SINK_IMPL_H */
