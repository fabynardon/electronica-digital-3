CMSIS DSP Library
=====================
This project provides a number of different build configurations
for ARM's CMSIS DSP Library.

This project is only intended to be used if you want to produce
modified versions of the supplied CMSIS_DSPLIB_CM0, 
CMSIS_DSPLIB_CM3, CMSIS_DSPLIB_CM4 and CMSIS_DSPLIB_CM4_NoFP 
projects, which contain pre-built libraries, built with 
optimization level -O2.

In addition, if you import this project into a workspace where you
are using one of the pre-built DSP library projects, then you will
be able to carry out source level debugging of the DSP functions.
Note however that debugging code built at -O2 will give a 
sub-optimal debug view.

Build options:
-------------
The CMSIS DSP Library build options are set as following by default:

ARM_MATH_CMx:
- Defined as appropriate for build target (CM0, CM3, CM4).

ARM_MATH_BIG_ENDIAN:
- Not set. Library variants are all built little endian. Most Cortex
  based MCU do not support big endian mode.

ARM_MATH_MATRIX_CHECK: 
- Not set.
- Define macro for checking on the input and output sizes of matrices. 

ARM_MATH_ROUNDING: 
- Not set.
- Define macro for rounding on support functions

__FPU_PRESENT:
- Set for CM4 build configuration (but not for CM4_NoFP, CM0 or CM3).
- Initialize macro __FPU_PRESENT = 1 when building on FPU supported
  targets.

