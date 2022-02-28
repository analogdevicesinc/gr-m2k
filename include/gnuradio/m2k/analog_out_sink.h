/* -*- c++ -*- */
/*
 * Copyright 2022 Analog Devices Inc..
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_M2K_ANALOG_OUT_SINK_H
#define INCLUDED_M2K_ANALOG_OUT_SINK_H

#include <gnuradio/m2k/api.h>
#include <gnuradio/sync_block.h>
#include <libm2k/m2k.hpp>


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
	  typedef std::shared_ptr <analog_out_sink> sptr;

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
