/* -*- c++ -*- */

#define M2K_API

%include "gnuradio.i"
%include "stdint.i"


%{
#include "m2k/analog_in_converter.h"
#include "m2k/analog_in_source.h"
%}

%include "m2k/analog_in_converter.h"
%include "m2k/analog_in_source.h"

GR_SWIG_BLOCK_MAGIC2(m2k, analog_in_converter);
GR_SWIG_BLOCK_MAGIC2(m2k, analog_in_source);
