
#include "stdhdr.h"
#include "device_detector.h"

#define POWER_BUS_EN(a)   (_TRISB0 = !a)
#define POWER_BUS_ON(a)   (_LATB0 = a)

void DeviceDetectorInit()
{
   
  POWER_BUS_ON(0);
  POWER_BUS_EN(1);
  block_ms(200);
  POWER_BUS_ON(1);


}

void DeviceDetectorProcessIO()
{



}
