#include <sys/types.h>
#include <cstdlib>

#define __debugbreak()  { __asm volatile ("bkpt #0"); }

// From https://github.com/andysworkshop/stm32plus/blob/master/examples/sdio/system/LibraryHacks.cpp

/*
 * The default pulls in 70K of garbage
 */
namespace __gnu_cxx
{

void __verbose_terminate_handler()
{
	__debugbreak();
	for(;;);
}
}


/*
 * The default pulls in about 12K of garbage
 */

extern "C" void __cxa_pure_virtual()
{
	__debugbreak();
	for(;;);
}


/*
 * EABI builds can generate reams of stack unwind code for system generated exceptions
 * e.g. (divide-by-zero). Since we don't support exceptions we'll wrap out these
 * symbols and save a lot of flash space.
 */

extern "C" void __wrap___aeabi_unwind_cpp_pr0() {}
extern "C" void __wrap___aeabi_unwind_cpp_pr1() {}
extern "C" void __wrap___aeabi_unwind_cpp_pr2() {}
