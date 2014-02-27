#include <iostream>
#include <sys/time.h>
#include "TimeCalculate.h"
#include <stdlib.h>
#include <sstream>
#include <ostream>
#include <string>
using namespace std;

double get_wall_time()
{
  struct timeval time;
  if (gettimeofday(&time,NULL)){
    //  Handle error
    return 0;

  }

  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

