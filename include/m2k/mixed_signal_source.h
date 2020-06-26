/* -*- c++ -*- */
/*
 * Copyright 2020 Analog Devices Inc.
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

#ifndef INCLUDED_M2K_MIXED_SIGNAL_SOURCE_H
#define INCLUDED_M2K_MIXED_SIGNAL_SOURCE_H

#include <m2k/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
    namespace m2k {

        class M2K_API mixed_signal_source : virtual public gr::sync_block {
        public:
        typedef boost::shared_ptr <mixed_signal_source> sptr;

        static sptr make(const std::string &uri,
                         int buffer_size,
                         double sampling_frequency,
                         int oversampling_ratio,
                         int kernel_buffers,
                         int trigger_condition,
                         int trigger_mode,
                         int trigger_source,
                         int trigger_delay,
                         double trigger_level);
    };


} // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_MIXED_SIGNAL_SOURCE_H */
