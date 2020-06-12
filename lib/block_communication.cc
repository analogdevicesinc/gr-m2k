#include "block_communication.h"

#include <iostream>
#include <thread>

#include "analog_in_source_impl.h"

block_communication &block_communication::get_instance()
{
	static block_communication s_instance;
	return s_instance;
}

void block_communication::request_sync_between_analog_digital(const std::string &uri, bool sync, int buffer_size)
{
	std::unique_lock<std::mutex> d_lock(d_sync_request_mutex);

	std::cerr << uri << " requested sync" << std::endl;


	d_sync_requested[uri] = sync;
	if (sync) {
		d_sync_state[uri] = {true, true};
		d_buffer_size = buffer_size;
		libm2k::context::M2k *context = gr::m2k::analog_in_source_impl::get_context(uri);
		context->startMixedSignalAcquisition(d_buffer_size);
	}
}

bool block_communication::is_sync_requested(const std::string &uri) const
{
	std::unique_lock<std::mutex> d_lock(d_sync_request_mutex);

	if (d_sync_requested.find(uri) != d_sync_requested.end()) {
		return d_sync_requested.at(uri);
	}

	return false;
}

bool block_communication::can_capture(const std::string &uri, block_communication::SYNC_DEVICE device)
{
	std::unique_lock<std::mutex> d_lock(d_sync_state_mutex);
	switch (device) {
	case SYNC_DEVICE::ANALOG:
		return d_sync_state[uri].first;
	case SYNC_DEVICE::DIGITAL:
		return d_sync_state[uri].second;
	}
}

void block_communication::data_captured(const std::string &uri, block_communication::SYNC_DEVICE device)
{
	std::unique_lock<std::mutex> d_lock(d_sync_state_mutex);
	switch (device) {
	case SYNC_DEVICE::ANALOG:
		d_sync_state[uri].first = false; // no longer capturing on analog
	case SYNC_DEVICE::DIGITAL:
		d_sync_state[uri].second = false; // no longer capturing on digital
	}

	if (!d_sync_state[uri].first && !d_sync_state[uri].second) {
		libm2k::context::M2k *context = gr::m2k::analog_in_source_impl::get_context(uri);
		context->startMixedSignalAcquisition(d_buffer_size);
		d_sync_state[uri] = {true, true}; // reset state, can capture on both digital and analog
	}
}

block_communication::block_communication()
	: d_buffer_size(0)
{

}
