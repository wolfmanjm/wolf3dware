#include "OutputStream.h"
#include <cstdarg>
#include <cstring>

int OutputStream::printf(const char *format, ...)
{
    char buffer[132+3];
    va_list args;
    va_start(args, format);

    size_t size = vsnprintf(buffer, sizeof(buffer), format, args)+1;

    va_end(args);

    if (size >= sizeof(buffer)) {
    	memcpy(&buffer[sizeof(buffer)-4], "...", 3);
    	buffer[size-1]= '\0';
    }

    oss << (const char*)buffer;

    return size;
}
