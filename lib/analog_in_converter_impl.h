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

#ifndef INCLUDED_M2K_ANALOG_IN_CONVERTER_IMPL_H
#define INCLUDED_M2K_ANALOG_IN_CONVERTER_IMPL_H

#include <m2k/analog_in_converter.h>
#include <libm2k/m2k.hpp>
#include <libm2k/analog/m2kanalogin.hpp>

namespace gr {
namespace m2k {

class analog_in_converter_impl : public analog_in_converter {
private:
    libm2k::analog::M2kAnalogIn *d_analog_in;

public:
    analog_in_converter_impl(libm2k::contexts::M2k *context);

    ~analog_in_converter_impl();

    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};

} // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_ANALOG_IN_CONVERTER_IMPL_H */
