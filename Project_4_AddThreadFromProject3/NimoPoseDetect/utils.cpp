#include "utils.h"
#include <stdio.h>
#include <sys/time.h>

Utils::Utils()
{

}

long Utils::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
