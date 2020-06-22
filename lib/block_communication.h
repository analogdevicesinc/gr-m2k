#ifndef BLOCK_COMMUNICATION_H
#define BLOCK_COMMUNICATION_H

#include <string>
#include <map>
#include <mutex>

class block_communication
{
public:

	enum class SYNC_DEVICE {
		DIGITAL,
		ANALOG,
	};

	static block_communication& get_instance();

	void request_sync_between_analog_digital(const std::string &uri, bool sync, int buffer_size);
	bool is_sync_requested(const std::string &uri) const;
	bool can_capture(const std::string &uri, SYNC_DEVICE device);
	void data_captured(const std::string &uri, SYNC_DEVICE device, int device_kernel_buffers);

public: /* deleted functions */
	block_communication(block_communication const &) = delete;
	void operator=(block_communication const &) = delete;

private:
	block_communication();

private:
	std::map<std::string, bool> d_sync_requested;
	std::map<std::string, std::pair<bool, bool>> d_sync_state;
	std::map<std::string, std::pair<int, int>> d_current_buffers_captured;
	std::map<std::string, bool> d_kernel_buffers_full;
	mutable std::mutex d_sync_request_mutex; // [uri]
	mutable std::mutex d_sync_state_mutex;
	int d_buffer_size;
};

#endif // BLOCK_COMMUNICATION_H
