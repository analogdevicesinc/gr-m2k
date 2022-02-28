/* -*- c++ -*- */
/*
 * Copyright 2022 Analog Devices Inc..
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "analog_in_source_impl.h"
#include <gnuradio/io_signature.h>
#include <libm2k/m2k.hpp>
#include <libm2k/enums.hpp>
#include <libm2k/m2kexceptions.hpp>
#include <libm2k/contextbuilder.hpp>
#include <libm2k/analog/m2kanalogin.hpp>
#include <boost/lexical_cast.hpp>

namespace gr {
  namespace m2k {

  analog_in_source::sptr
  analog_in_source::make(const std::string &uri,
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
					     bool deinit,
					     double data_rate
					     )
  {
	  return gnuradio::get_initial_sptr
			  (new analog_in_source_impl(analog_in_source_impl::get_context(uri),buffer_size, channels, ranges, sampling_frequency, oversampling_ratio,
									     kernel_buffers,
									     calibrate_ADC, stream_voltage_values, trigger_condition, trigger_mode,
									     trigger_source,
									     trigger_delay, trigger_level, streaming, deinit, data_rate));
  }

  analog_in_source::sptr
  analog_in_source::make_from(libm2k::context::M2k *context,
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
							  bool deinit,
							  double data_rate)
  {
	  return gnuradio::get_initial_sptr
			  (new analog_in_source_impl(context, buffer_size, channels, ranges, sampling_frequency, oversampling_ratio,
									     kernel_buffers,
									     calibrate_ADC, stream_voltage_values, trigger_condition, trigger_mode,
									     trigger_source,
									     trigger_delay, trigger_level, streaming, deinit, data_rate));
  }

  analog_in_source_impl::analog_in_source_impl(libm2k::context::M2k *context,
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
											   bool deinit,
											   double data_rate)
	  : gr::sync_block("analog_in_source",
					   gr::io_signature::make(0, 0, 0),
					   gr::io_signature::make(1, 2, sizeof(float))),
	  d_uri(context->getUri()),
	  d_buffer_size(buffer_size),
	  d_channels(channels),
	  d_stream_voltage_values(stream_voltage_values),
	  d_timeout(100),
	  d_deinit(deinit),
	  d_port_id(pmt::mp("msg")),
	  d_data_rate(data_rate)

  {
	  add_context(context);
	  d_analog_in = context->getAnalogIn();

	  d_analog_in->setKernelBuffersCount(kernel_buffers);
	  set_params(ranges, sampling_frequency, oversampling_ratio);
	  set_trigger(trigger_condition, trigger_mode, trigger_source, trigger_delay, trigger_level, streaming);

	  if (calibrate_ADC) {
		  context->calibrateADC();
	  }

	  if (!stream_voltage_values) {
		  set_output_signature(gr::io_signature::make(1, 2, sizeof(short)));
	  }

	  d_items_in_buffer = 0;
	  set_output_multiple(0x400);
	  message_port_register_out(d_port_id);
  }

  analog_in_source_impl::~analog_in_source_impl()
  {
	  if (d_deinit) {
		  remove_contexts(d_uri);
	  }
  }

  void analog_in_source_impl::set_params(std::vector<int> ranges,
									     double sampling_frequency,
									     int oversampling_ratio)
  {
	  for (int i = 0; i < d_channels.size(); i++) {
		  if (d_channels.at(i)) {
			  d_analog_in->enableChannel(i, true);
			  d_analog_in->setRange(static_cast<libm2k::analog::ANALOG_IN_CHANNEL>(i),
								    (static_cast<libm2k::analog::M2K_RANGE>(ranges[i])));
		  }
	  }

	  d_analog_in->setSampleRate(sampling_frequency);
	  d_analog_in->setOversamplingRatio(oversampling_ratio);
  }

  void analog_in_source_impl::set_trigger(std::vector<int> trigger_condition,
										  std::vector<int> trigger_mode,
										  int trigger_source,
										  int trigger_delay,
										  std::vector<double> trigger_level,
										  bool streaming)
  {
	  libm2k::M2kHardwareTrigger *trigger = d_analog_in->getTrigger();

	  trigger->setAnalogStreamingFlag(false);
	  for (int i = 0; i < d_channels.size(); ++i) {
		  if (d_channels.at(i)) {
			  trigger->setAnalogCondition(i, static_cast<libm2k::M2K_TRIGGER_CONDITION_ANALOG>(trigger_condition[i]));
			  trigger->setAnalogMode(i, static_cast<libm2k::M2K_TRIGGER_MODE>(trigger_mode[i]));
			  trigger->setAnalogLevel(i, trigger_level[i]);
		  }
	  }
	  trigger->setAnalogSource(static_cast<libm2k::M2K_TRIGGER_SOURCE_ANALOG>(trigger_source));
	  trigger->setAnalogDelay(trigger_delay);
	  trigger->setAnalogStreamingFlag(streaming);
  }

  void analog_in_source_impl::set_timeout_ms(unsigned int timeout)
  {
	  if (d_timeout != timeout) {
		  std::unique_lock<std::mutex> lock(d_mutex);
		  d_timeout = timeout;
	  }
  }

  libm2k::context::M2k *analog_in_source_impl::get_context(const std::string &uri)
  {
	  auto element = s_contexts.find(uri);
	  if (element == s_contexts.end()) {
		  libm2k::context::M2k *ctx = libm2k::context::m2kOpen(uri.c_str());
		  if (ctx == nullptr) {
			  throw std::runtime_error("Unable to create the context!");
		  }
		  s_contexts.insert(std::pair<std::string, libm2k::context::M2k *>(ctx->getUri(), ctx));
		  return ctx;
	  }
	  return element->second;
  }

  void analog_in_source_impl::add_context(libm2k::context::M2k *context)
  {
	  auto element = s_contexts.find(context->getUri());
	  if (element == s_contexts.end()) {
		  s_contexts.insert(std::pair<std::string, libm2k::context::M2k *>(context->getUri(), context));
	  }
  }

  void analog_in_source_impl::remove_contexts(const std::string &uri)
  {
	  std::lock_guard <std::mutex> lock(s_ctx_mutex);
	  auto element = s_contexts.find(uri);
	  if (element != s_contexts.end()) {
		  libm2k::context::contextClose(element->second, true);
		  s_contexts.erase(element);
	  }
  }

  void analog_in_source_impl::set_buffer_size(int buffer_size)
  {
	  if (d_buffer_size != buffer_size) {
		  std::unique_lock<std::mutex> lock(d_mutex);

		  d_items_in_buffer = 0;
		  d_buffer_size = buffer_size;
	  }
  }

  void analog_in_source_impl::set_data_rate(double rate) {
  #ifdef _WIN32
	  rate = 0;
  #endif
	  d_data_rate = rate;
  }

  void analog_in_source_impl::refill_buffer()
  {
	  std::unique_lock <std::mutex> lock(d_mutex);

	  while (true) {
		  if (d_thread_stopped) {
			  break;
		  }
		  d_cond_wait.wait(lock, [&] { return d_empty_buffer; });

		  try {
			  lock.unlock();
			  std::chrono::high_resolution_clock::time_point t1 ;
			  if(d_data_rate) {
				  t1 = std::chrono::high_resolution_clock::now();
			  }

			  d_raw_samples = d_analog_in->getSamplesRawInterleaved(d_buffer_size);

			  if(d_data_rate) {
				  std::chrono::duration<double> sec = std::chrono::high_resolution_clock::now() - t1; // compute getSamplesDuration
				  unsigned int frameTime = 1000000.0/d_data_rate;
				  unsigned int acquisitionTime = (1000000.0)* sec.count();
				  unsigned int diff = (frameTime > acquisitionTime) ? frameTime - acquisitionTime: 0;
				  std::this_thread::sleep_for(std::chrono::microseconds(diff)); // wait remainder
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

  int analog_in_source_impl::work(int noutput_items,
								  gr_vector_const_void_star &input_items,
								  gr_vector_void_star &output_items)
  {
	  std::unique_lock <std::mutex> lock(d_mutex);
	  if (d_thread_stopped) {
		  return -1;
	  }
	  if (!d_items_in_buffer) {
		  d_empty_buffer = true;
		  d_cond_wait.notify_one();
	  }

	  while (d_empty_buffer) {
		  // use wait_for to avoid permanent blocking in the work function
		  bool buffer_refilled = d_cond_wait.wait_for(lock, std::chrono::milliseconds(d_timeout),
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
	  unsigned int sample_index, channel_index, out_stream_index;

	  for (out_stream_index = 0, channel_index = 0;
		   out_stream_index < output_items.size(); out_stream_index++, channel_index++) {
		  while (!d_channels[channel_index]) {
			  channel_index++;
		  }

		  if (!d_sample_index) {
			  tag_t tag;
			  tag.value = pmt::from_long(d_items_in_buffer);
			  tag.offset = nitems_written(out_stream_index);
			  tag.key = pmt::intern("buffer_start");
			  tag.srcid = alias_pmt();

			  add_item_tag(out_stream_index, tag);
		  }

		  if (d_stream_voltage_values) {
			  float *out = (float *) output_items[out_stream_index];
			  for (sample_index = 0; sample_index < nb_samples; sample_index++) {
				  out[sample_index] = boost::lexical_cast<float>(d_analog_in->convertRawToVolts(channel_index, d_raw_samples[(2*(sample_index + d_sample_index)) + out_stream_index]));

			  }
		  } else {
			  short *out = (short *) output_items[out_stream_index];
			  for (sample_index = 0; sample_index < nb_samples; sample_index++) {
				  out[sample_index] = d_raw_samples[(2*(sample_index + d_sample_index)) + out_stream_index];
			  }
		  }
	  }

	  d_items_in_buffer -= nb_samples;
	  d_sample_index += nb_samples;

	  return (int) nb_samples;
  }

  bool analog_in_source_impl::start()
  {
	  std::unique_lock <std::mutex> lock(d_mutex);

	  d_items_in_buffer = 0;
	  d_empty_buffer = true;
	  d_thread_stopped = false;

	  try {
		  d_analog_in->startAcquisition(d_buffer_size);
	  } catch (const libm2k::m2k_exception &e) {
		  return false;
	  }

	  d_refill_thread = std::thread(std::bind(&analog_in_source_impl::refill_buffer, this));

	  return true;
  }

  bool analog_in_source_impl::stop()
  {
	  d_analog_in->cancelAcquisition();
	  std::unique_lock <std::mutex> lock(d_mutex);
	  d_empty_buffer = true;
	  d_thread_stopped = true;
	  d_cond_wait.notify_one();
	  lock.unlock();
	  d_refill_thread.join();
	  d_analog_in->stopAcquisition();
	  return true;
  }

  } /* namespace m2k */
} /* namespace gr */
