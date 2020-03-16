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

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif