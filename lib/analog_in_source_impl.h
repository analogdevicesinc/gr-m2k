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

#ifndef INCLUDED_M2K_ANALOG_IN_SOURCE_IMPL_H
#define INCLUDED_M2K_ANALOG_IN_SOURCE_IMPL_H

#include <m2k/analog_in_source.h>
#include <libm2k/analog/m2kanalogin.hpp>
#include <map>


namespace gr {
namespace m2k {

static std::map<std::string, libm2k::context::M2k *> s_contexts;
static boost::mutex s_ctx_mutex;

class analog_in_source_impl : public analog_in_source {
private:
    libm2k::analog::M2kAnalogIn *d_analog_in;
    const std::string d_uri;
    std::vector<int> d_channels;

    unsigned int d_timeout;
    bool d_deinit;

    unsigned int d_buffer_size;
    unsigned int d_sample_index;
    unsigned long d_items_in_buffer;
    const short *d_raw_samples;
    bool d_stream_voltage_values;

    pmt::pmt_t d_port_id;

    gr::thread::mutex d_mutex;
    gr::thread::condition_variable d_cond_wait;
    gr::thread::thread d_refill_thread;
    volatile bool d_empty_buffer{}, d_thread_stopped{};

public:
    analog_in_source_impl(libm2k::context::M2k *context,
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
                          std::vector<double> trigger_level,
                          bool streaming,
                          bool deinit);

    ~analog_in_source_impl() override;

    void refill_buffer();

    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items) override;

    bool start() override;
    bool stop() override;

    void set_params(std::vector<int> ranges,
                    double sampling_frequency,
                    int oversampling_ratio) override;

    void set_trigger(std::vector<int> trigger_condition,
                     std::vector<int> trigger_mode,
                     int trigger_source,
                     int trigger_delay,
                     std::vector<double> trigger_level,
                     bool streaming) override;

    void set_timeout_ms(unsigned int timeout) override;

    static libm2k::context::M2k *get_context(const std::string &uri);

    static void add_context(libm2k::context::M2k *context);

    static void remove_contexts(const std::string &uri);

    void set_buffer_size(int buffer_size) override;
};

} // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_ANALOG_IN_SOURCE_IMPL_H */
