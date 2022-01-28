#include "time.h"

static unsigned long nowTime, oldTime;
float dt;

void GetSamplingTime()
{
    nowTime = micros();
    dt = (float)(nowTime - oldTime) / 1000000.0; // sec
    oldTime = nowTime;
}
