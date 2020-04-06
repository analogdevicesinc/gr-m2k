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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "adf4350_sink_impl.h"
#include <gnuradio/io_signature.h>
#include "analog_in_source_impl.h"
#include <libm2k/m2k.hpp>
#include <libm2k/contextbuilder.hpp>
#include <libm2k/tools/spi_extra.hpp>
#include <boost/lexical_cast.hpp>


namespace gr {
namespace m2k {

adf4350_sink::sptr
adf4350_sink::make(const std::string &uri,
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
                   uint32_t aux_output_power)
{
    return gnuradio::get_initial_sptr
        (new adf4350_sink_impl(uri,
                               clock,
                               mosi,
                               miso,
                               chip_select,
                               clkin,
                               channel_spacing,
                               power_up_frequency,
                               reference_div_factor,
                               reference_doubler_enable,
                               reference_div2_enable,
                               phase_detector_polarity_positive_enable,
                               lock_detect_precision_6ns_enable,
                               lock_detect_function_integer_n_enable,
                               charge_pump_current,
                               muxout_select,
                               low_spur_mode_enable,

                               cycle_slip_reduction_enable,
                               charge_cancellation_enable,
                               anti_backlash_3ns_enable,
                               band_select_clock_mode_high_enable,
                               clk_divider_12bit,
                               clk_divider_mode,

                               aux_output_enable,
                               aux_output_fundamental_enable,
                               mute_till_lock_enable,
                               output_power,
                               aux_output_power));
}

adf4350_sink_impl::adf4350_sink_impl(const std::string &uri,
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
                                     uint32_t aux_output_power)
    : gr::block("adf4350_sink",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
    d_uri(uri)
{
    libm2k::context::M2k *context = analog_in_source_impl::get_context(uri);
    m2k_spi_init m2KSpiInit;
    m2KSpiInit.clock = clock;
    m2KSpiInit.mosi = mosi;
    m2KSpiInit.miso = miso;
    m2KSpiInit.bit_numbering = MSB;
    m2KSpiInit.context = context;

    adf4350_init_param default_adf4350_init_param = {
        /* SPI */
        {
            1000000,
            chip_select,
            SPI_MODE_0,
            (void *) &m2KSpiInit
        },
        clkin,
        channel_spacing,
        power_up_frequency,
        reference_div_factor,
        reference_doubler_enable,
        reference_div2_enable,

        /* r2_user_settings */
        phase_detector_polarity_positive_enable,
        lock_detect_precision_6ns_enable,
        lock_detect_function_integer_n_enable,
        charge_pump_current,
        muxout_select,
        low_spur_mode_enable,

        /* r3_user_settings */
        cycle_slip_reduction_enable,
        charge_cancellation_enable,
        anti_backlash_3ns_enable,
        band_select_clock_mode_high_enable,
        clk_divider_12bit,
        clk_divider_mode,

        /* r4_user_settings */
        aux_output_enable,
        aux_output_fundamental_enable,
        mute_till_lock_enable,
        output_power,
        aux_output_power    // aux_output_power;
    };
    adf4350_setup(&d_adf4350_device, default_adf4350_init_param);
    message_port_register_in(pmt::mp("msg"));
    set_msg_handler(pmt::mp("msg"), boost::bind(&adf4350_sink_impl::write_attribute, this, _1));
}

adf4350_sink_impl::~adf4350_sink_impl()
{
    analog_in_source_impl::remove_contexts(d_uri);
}

void adf4350_sink_impl::write_attribute(pmt::pmt_t pdu)
{
    pmt::pmt_t key;

    if (!is_pair(pdu)) {
        throw std::runtime_error("Message not a pair!\n");
    }

    key = pmt::car(pdu);
    std::string skey = pmt::symbol_to_string(key);

    if (skey == "frequency") {
        long long frequency;
        pmt::pmt_t freq = pmt::cdr(pdu);
        std::string sfreq = pmt::symbol_to_string(freq);
        frequency = std::stoll(sfreq);
        adf4350_out_altvoltage0_frequency(d_adf4350_device, frequency);
        std::cout << "ADF4350: PLL 0 frequency = " << frequency << "Hz." << std::endl;
    }
}

} /* namespace m2k */
} /* namespace gr */
