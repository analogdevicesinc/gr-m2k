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

#ifndef INCLUDED_M2K_ANALOG_IN_SOURCE_H
#define INCLUDED_M2K_ANALOG_IN_SOURCE_H

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
 * This block allows for streaming data from ADALM2000
 */
class M2K_API analog_in_source : virtual public gr::sync_block {
public:
    typedef boost::shared_ptr <analog_in_source> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of m2k::analog_in_source.
     *
     * \param uri String of the context uri
     * \param buffer_size Integer number of samples to be put into each buffered
     * \param channels List of channel's states (enable/disable)
     * \param ranges List of ranges for the given channels
     * \param sampling_frequency Frequency at which the hardware will input samples
     * \param oversampling_ratio Integer number representing the oversampling factor
     * \param kernel_buffers The number of the internal buffers
     * \param calibrate_ADC Boolean value for deciding if the ADC should be calibrated
     * \param stream_voltage_values Boolean value for setting the output type: voltage/raw
     * \param trigger_condition List of triggering conditions for the given channels
     * \param trigger_mode List of triggering modes for the given channels
     * \param trigger_source The source for the triggering event
     * \param trigger_delay The number of samples in buffer before the triggered sample
     * \param trigger_level List of triggering levels for the given channels
     */
    static sptr make(const std::string &uri,
                     int buffer_size,
                     const std::vector<int> &channels,
                     std::vector<int> ranges,
                     double sampling_frequency,
                     int oversampling_ratio,
                     int kernel_buffers,
                     bool calibrate_ADC,
                     bool stream_voltage_values,
                     std::vector<int> trigger_condition,
                     std::vector<int> trigger_mode,
                     int trigger_source,
                     int trigger_delay,
                     std::vector<double> trigger_level);

    static sptr make_from_ctx(libm2k::context::M2k *ctx,
			      int buffer_size,
			      const std::vector<int> &channels,
			      std::vector<int> ranges,
			      double sampling_frequency,
			      int oversampling_ratio,
			      int kernel_buffers,
			      bool calibrate_ADC,
			      bool stream_voltage_values,
			      std::vector<int> trigger_condition,
			      std::vector<int> trigger_mode,
			      int trigger_source,
			      int trigger_delay,
			      std::vector<double> trigger_level);

    virtual void set_params(std::vector<int> ranges,
                            double sampling_frequency,
                            int oversampling_ratio) = 0;

    virtual void set_trigger(std::vector<int> trigger_condition,
                             std::vector<int> trigger_mode,
                             int trigger_source,
                             int trigger_delay,
                             std::vector<double> trigger_level) = 0;

    virtual void set_buffer_size(int buffer_size) = 0;
};


} // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_ANALOG_IN_SOURCE_H */
