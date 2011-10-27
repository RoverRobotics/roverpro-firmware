/** INCLUDES *******************************************************/
#include "include/GenericTypeDefs.h"
#include "include/Compiler.h"

#define FSM_RECEIVE 0x00
#define FSM_SEND 0x01

#define LEFT_JOYSTICK 0x00
#define RIGHT_JOYSTICK 0x01
#define HORIZONTAL_AXIS 0x00
#define VERTICAL_AXIS 0x01



//#define TEST
//#define TEST_RS485
//#define TEST_PWM
//#define TEST_DATALINK
//#define TEST_DYNAMIXEL
//#define TEST_AUDIO
//#define TEST_REMOTE_POWER
//#define TEST_MOSFETS
//#define TEST_SAFE_BOOTUP
//#define TEST_OSD
//#define TEST_VIDEO

_CONFIG1(JTAGEN_OFF & GCP_ON & GWRP_OFF & BKBUG_OFF & COE_OFF & FWDTEN_ON & ICS_PGx2 & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS1024) 
    _CONFIG2( 0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV5 & IOL1WAY_ON)


/** I N C L U D E S **********************************************************/

#include "include/GenericTypeDefs.h"
#include "include/Compiler.h"
#include "HardwareProfile.h"
#include "PWM.h"
#include "Datalink.h"
//#include "RS485.h"
#include "Messaging.h"
#include "SPI.h"
#include "Dynamixel.h"
#include "Timers.h"
#include "Audio.h"
#include "Remote_Power.h"
#include "Initialize.h"
#include "SPI.h"
#include "Video_Receiver.h"
#include "Controller.h"
#include "address_generation.h"

/** V A R I A B L E S ********************************************************/
#pragma udata
char USB_In_Buffer[64];
char USB_Out_Buffer[64];

BOOL stringPrinted;


/** P R I V A T E  P R O T O T Y P E S ***************************************/
//static void InitializeSystem(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void UserInit(void);



/** VECTOR REMAPPING ***********************************************/


#if defined(__C30__)
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        /*
         *	ISR JUMP TABLE
         *
         *	It is necessary to define jump table as a function because C30 will
         *	not store 24-bit wide values in program memory as variables.
         *
         *	This function should be stored at an address where the goto instructions 
         *	line up with the remapped vectors from the bootloader's linker script.
         *  
         *  For more information about how to remap the interrupt vectors,
         *  please refer to AN1157.  An example is provided below for the T2
         *  interrupt with a bootloader ending at address 0x1400
         */
//        void __attribute__ ((address(0x1404))) ISRTable(){
//        
//        	asm("reset"); //reset instruction to prevent runaway code
//        	asm("goto %0"::"i"(&_T2Interrupt));  //T2Interrupt's address
//        }
    #endif




#endif







/** DECLARATIONS ***************************************************/
#pragma code

/******************************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *****************************************************************************/
int main(void)

{   


	ClrWdt();

//unsigned int i;





		Datalink_Init();
		
		Init_Controller();




		Init_Audio();
		Init_Video();
		Init_Timer4();
		block_ms(500);

		Set_Video_Channel(1);
		set_stored_address();

	Controller_State = NOT_CONNECTED;
	event = NONE;
	osd_event = NONE;


/*

	while(1)
	{

		if(event == SEND_TURNON_MESSAGE)
		{
			event = NONE;
			//Send_Turnon_Message();
		}
		if(osd_event == CLEAR_LOGO)
		{
			osd_event = NONE;
		//	ghetto_clear_screen();
		//	MAX_External_Synch();
		//	OSD_display_string("Trying to connect to robot",2,1,26);
			break;
		}
		
		//block_ms(100);
		//block_ms(300);
		//Send_Turnoff_Message();
		//block_ms(300);
		

	}
	
	Controller_State = CONNECTED;

	block_ms(300);
	

*/

    while(1)
    {
	ClrWdt();
		if(osd_event == CLEAR_SCREEN)
		{
			osd_event = NONE;
		//	OSD_display_string("                          ",2,1,26);
		}
		

		Construct_Controller_Message();
		
		
		Datalink_Message_Send();
		//disable_video(250);
		//display_voltage();
		
		block_ms(100);
		
   
    }//end while
}//end main






/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
    stringPrinted = TRUE;
    //Initialize all of the LED pins

	//mInitAllLEDs();
}//end UserInit










/** EOF main.c *************************************************/
