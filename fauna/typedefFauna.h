#ifndef TYPEDEF_FAUNA
#define TYPEDEF_FAUNA

#include <vector>
#include <string>
#include <NIDAQmx.h>

typedef struct STREAMPARAM
{
  double sps;
  int spb;
};

typedef struct BUFFERINFO
{
  float64 timeOut;
  bool idxBuffer;  
  int32 numSampleRead[2];
};

#endif