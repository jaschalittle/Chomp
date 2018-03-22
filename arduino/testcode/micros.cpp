#include <cstdint>
#include <cstddef>
#include <sys/time.h>

uint32_t micros(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec*1000000L) + tv.tv_usec;
}
