/* -*- c++ -*- */
/*
 * Copyright 2019 Analog Devices Inc.
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

#ifndef INCLUDED_M2K_ANALOG_OUT_SINK_H
#define INCLUDED_M2K_ANALOG_OUT_SINK_H

#include <m2k/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace m2k {

/*!
 * \brief Sink for ADALM2000 with buffered input channels
 * \ingroup m2k
 *
 *\details
 * This block allows for streaming data to ADALM2000
 */
class M2K_API analog_out_sink : virtual public gr::sync_block {
public:
	typedef boost::shared_ptr <analog_out_sink> sptr;

	/*!
	 * \brief Return a shared_ptr to a new instance of m2k::analog_out_sink.
	 *
	 * \param uri String of the context uri
	 * \param buffer_size Integer number of samples to be put into each buffered
	 * \param sampling_frequency List of frequency at which the hardware will output samples
	 * \param oversampling_ratio List of integer numbers representing the oversampling factors
	 * \param kernel_buffers List of integer values representing the number of the internal buffers
	 * \param cyclic List of cyclic modes (1 = cyclic, 0 = non-cyclic)
	 * \param calibrate_DAC Boolean value for deciding if both DACs should be calibrated
	 * \param input_voltage Boolean value for setting the input type: voltage/raw
	 */
	static sptr make(const std::string &uri,
					 const int buffer_size,
					 std::vector<double> sampling_frequency,
					 std::vector<int> oversampling_ratio,
					 const std::vector<int> &kernel_buffers,
					 const std::vector<int> &cyclic,
					 bool calibrate_DAC,
					 bool input_voltage);

	virtual void set_params(std::vector<double> sampling_frequency,
							std::vector<int> oversampling_ratio) = 0;
};


} // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_ANALOG_OUT_SINK_H */
