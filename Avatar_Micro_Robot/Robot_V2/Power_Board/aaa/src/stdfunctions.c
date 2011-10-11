/**
 * @file stdfunctions.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 *
 */

#include "stdhdr.h"

void serial_delay_tick(ticks)
{
	while(ticks--) { };
}


void block_ms(unsigned int ms)

{

                unsigned int i;

                unsigned int j;

 

                for(i=0;i<800;i++)

 

                {

                                for(j=0;j<ms;j++);

                }

}
