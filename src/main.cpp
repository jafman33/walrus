#include "wing/wing.h"
#include "AsyncUDP_Teensy41.h"    

using namespace Cyberwing;

Wing wing;

void setup()
{
  delay(3000);
  wing.init();
}

void loop()
{
  wing.update();
}
