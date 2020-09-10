#ifndef MIXED_SIGNAL_SOURCE_H
#define MIXED_SIGNAL_SOURCE_H

#include <gnuradio/sync_block.h>

#include <boost/shared_ptr.hpp>

#include <m2k/api.h>

#include <libm2k/m2k.hpp>

// TODO: add params for digital, analog, make(uri...), doxygen comments

namespace gr {
namespace m2k {
class M2K_API mixed_signal_source : virtual public gr::sync_block {
public:
	typedef boost::shared_ptr <mixed_signal_source> sptr;

	static sptr make_from(libm2k::context::M2k *context, int buffer_size);

	virtual  void set_timeout_ms(unsigned int timeout) = 0;
	virtual void set_buffer_size(int buffer_size) = 0;
};
} // namespace m2k
} // namespace gr

#endif // MIXED_SIGNAL_SOURCE_H
