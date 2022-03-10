/* FastCRC library code is placed under the MIT license
 * Copyright (c) 2014,2015,2016 Frank Bösing
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 
// CPU-specific implementations of helper functions

#if !defined(KINETISK)
#if !defined(FastCRC_cpu)
#define FastCRC_cpu

//Reverse byte order (16 bit)
#if defined(__thumb__)  
static inline __attribute__((always_inline)) 
uint32_t REV16( uint32_t value) //ARM-THUMB
{
	asm ("rev16 %0, %1" : "=r" (value) : "r" (value) );
	return(value);
}
#else
static inline __attribute__((always_inline)) 
uint32_t REV16( uint32_t value) //generic
{
	return (value >> 8) | ((value & 0xff) << 8);
}
#endif





//Reverse byte order (32 bit)
#if defined(__thumb__) 
static inline  __attribute__((always_inline))
uint32_t REV32( uint32_t value) //ARM-THUMB
{
	asm ("rev %0, %1" : "=r" (value) : "r" (value) );
	return(value);
}
#else
static inline  __attribute__((always_inline))
uint32_t REV32( uint32_t value) //generic
{
	value = (value >> 16) | ((value & 0xffff) << 16);
	return ((value >> 8) & 0xff00ff) | ((value & 0xff00ff) << 8);
}
#endif


#endif
#endif
