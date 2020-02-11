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

#ifndef INCLUDED_M2K_ANALOG_IN_CONVERTER_H
#define INCLUDED_M2K_ANALOG_IN_CONVERTER_H

#include <m2k/api.h>
#include <gnuradio/sync_block.h>
#include <libm2k/m2k.hpp>

namespace gr {
namespace m2k {

/*!
 * \brief Raw ADC samples converter
 * \ingroup m2k
 *
 * \details
 * This block converts raw samples of ADALM2000's ADC into voltage
 */
class M2K_API analog_in_converter : virtual public gr::sync_block {
public:
    typedef boost::shared_ptr <analog_in_converter> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of m2k::analog_in_converter.
     *
     * \param uri String of the context uri
     */
    static sptr make(const std::string &uri);

    static sptr make_from(libm2k::contexts::M2k *context);

};

} // namespace m2k
} // namespace gr

#endif /* INCLUDED_M2K_ANALOG_IN_CONVERTER_H */
