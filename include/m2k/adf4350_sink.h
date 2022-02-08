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

#ifndef INCLUDED_M2K_ADF4350_SINK_H
#define INCLUDED_M2K_ADF4350_SINK_H

#include <m2k/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace m2k {

/*!
 * \brief Sink for ADF4350
 * \ingroup m2k
 *
 * \details
 * This block allows the communication between ADALM2000 and ADF4350
 */
class M2K_API adf4350_sink : virtual public gr::block {
public:
	typedef boost::shared_ptr <adf4350_sink> sptr;

	/*!
	 * \brief Return a shared_ptr to a new instance of m2k::adf4350_sink.
	 *
	 * \param uri String of the context uri
	 * \param clock Index of any digital pin
	 * \param mosi Index of any digital pin
	 * \param miso Index of any digital pin
	 * \param chip_select Index of any digital pin
	 * \param clkin Integer - clock in
	 * \param channel_spacing Spacing of the channel
	 * \param power_up_frequency Power up frequency
	 * \param reference_div_factor Reference division factor
	 * \param reference_doubler_enable Enable doubler reference
	 * \param reference_div2_enable Enable div2 reference
	 * \param phase_detector_polarity_positive_enable Enable positive polarity of phase detector
	 * \param lock_detect_precision_6ns_enable Enable precision 6ns
	 * \param lock_detect_function_integer_n_enable Enable function integer n
	 * \param charge_pump_current Charge pump current
	 * \param muxout_select Select muxout
	 * \param low_spur_mode_enable Enable low spur mode
	 * \param cycle_slip_reduction_enable Enable cyclic slip reduction
	 * \param charge_cancellation_enable Enable charge cancellation
	 * \param anti_backlash_3ns_enable Enable anti backlash
	 * \param band_select_clock_mode_high_enable Enable clock mode high
	 * \param clk_divider_12bit Clock divider 12 bits
	 * \param clk_divider_mode Clock divider mode
	 * \param aux_output_enable Enable aux output
	 * \param aux_output_fundamental_enable Enable aux fundamental output
	 * \param mute_till_lock_enable Enable mute until lock
	 * \param output_power Output power
	 * \param aux_output_power Aux output power
	 */
	static sptr make(const std::string &uri,
					 uint8_t clock,
					 uint8_t mosi,
					 uint8_t miso,
					 uint8_t chip_select,
					 uint32_t clkin,
					 uint32_t channel_spacing,
					 uint32_t power_up_frequency,
					 uint32_t reference_div_factor,
					 uint8_t reference_doubler_enable,
					 uint8_t reference_div2_enable,

					 /* r2_user_settings */
					 uint8_t phase_detector_polarity_positive_enable,
					 uint8_t lock_detect_precision_6ns_enable,
					 uint8_t lock_detect_function_integer_n_enable,
					 uint32_t charge_pump_current,
					 uint32_t muxout_select,
					 uint8_t low_spur_mode_enable,

					 /* r3_user_settings */
					 uint8_t cycle_slip_reduction_enable,
					 uint8_t charge_cancellation_enable,
					 uint8_t anti_backlash_3ns_enable,
					 uint8_t band_select_clock_mode_high_enable,
					 uint32_t clk_divider_12bit,
					 uint32_t clk_divider_mode,

					 /* r4_user_settings */
					 uint8_t aux_output_enable,
					 uint8_t aux_output_fundamental_enable,
					 uint8_t mute_till_lock_enable,
					 uint32_t output_power,
					 uint32_t aux_output_power);

};

} // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_ADF4350_SINK_H */
