#ifndef MACROS_H
#define MACROS_H

#ifdef __unix__
    #include <sys/time.h>
    //source: https://gist.github.com/sevko/d23646ba07c77c15fde9
    #define time_us() ({ \
        struct timeval currentTime;\
        gettimeofday(&currentTime, NULL);\
        currentTime.tv_sec * (int)1e6 + currentTime.tv_usec;\
    })
#else   
    #include <Arduino.h>

    #define time_us() micros()
#endif

#define SECONDS_TO_MICROS 1000000

template <typename T> T map(T x, T x1, T x2, T y1, T y2);
template <typename T> T map(T x, T x1, T x2, T y1, T y2)
{
 return (x - x1) * (y2 - y1) / (x2 - x1) + y1;
}

#endif