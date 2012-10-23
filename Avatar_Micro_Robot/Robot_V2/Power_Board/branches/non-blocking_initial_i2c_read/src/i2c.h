
#ifndef I2C_H /*I2C_H  */
#define I2C_H

/******************************************************************************
 *
 *                  I2C PERIPHERAL LIBRARY HEADER FILE
 *
 ******************************************************************************
 * FileName:        i2c.h
 * Dependencies:    See include below
 * Processor:       PIC24
 * Compiler:        MPLAB C30
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************/

#include "PIC24F_periph_features.h"

//This preprocessor conditional statement is to avoid unintended linking for unsuppported devices.
#if defined (i2c_v1_1) || defined (i2c_v1_2) || defined (i2c_v1_3) || defined (LIB_BUILD)

#define I2C1RCV_VALUE               0x0000
#define I2C1TRN_VALUE               0x00FF
#define I2C1BRG_VALUE               0x0000
#define I2C1CON_VALUE               0x0000
#define I2C1STAT_VALUE              0x0000
#define I2C1ADD_VALUE               0x0000

#define I2C2RCV_VALUE               0x0000
#define I2C2TRN_VALUE               0x00FF
#define I2C2BRG_VALUE               0x0000
#define I2C2CON_VALUE               0x0000
#define I2C2STAT_VALUE              0x0000
#define I2C2ADD_VALUE               0x0000

#define I2C3RCV_VALUE               0x0000
#define I2C3TRN_VALUE               0x00FF
#define I2C3BRG_VALUE               0x0000
#define I2C3CON_VALUE               0x0000
#define I2C3STAT_VALUE              0x0000
#define I2C3ADD_VALUE               0x0000


#ifndef USE_AND_OR /* Format for AND_OR based bit setting */
/* I2CCON register Configuration bit definitions */
#define I2C_ON                      0xFFFF /*I2C module enabled */
#define I2C_OFF                     0x7FFF /*I2C module disabled */

#define I2C_IDLE_STOP               0xFFFF /*stop I2C module in Idle mode */
#define I2C_IDLE_CON                0xDFFF /*continue I2C module in Idle mode */

#define I2C_CLK_REL                 0xFFFF /*release clock */
#define I2C_CLK_HLD                 0xEFFF /*hold clock  */

#define I2C_IPMI_EN                 0xFFFF /*IPMI mode enabled */
#define I2C_IPMI_DIS                0xF7FF /*IPMI mode not enabled */

#define I2C_10BIT_ADD               0xFFFF /*I2CADD is 10-bit address */
#define I2C_7BIT_ADD                0xFBFF /*I2CADD is 7-bit address */

#define I2C_SLW_DIS                 0xFFFF /*Disable Slew Rate Control for 100KHz */
#define I2C_SLW_EN                  0xFDFF /*Enable Slew Rate Control for 400KHz */

#define I2C_SM_EN                   0xFFFF /*Enable SM bus specification */
#define I2C_SM_DIS                  0xFEFF /*Disable SM bus specification */

#define I2C_GCALL_EN                0xFFFF /*Enable Interrupt when General call address is received. */
#define I2C_GCALL_DIS               0xFF7F /*Disable General call address. */

#define I2C_STR_EN                  0xFFFF /*Enable clock stretching */
#define I2C_STR_DIS                 0xFFBF/*disable clock stretching */

#define I2C_ACK                     0xFFDF /*Transmit 0 to send ACK as acknowledge */
#define I2C_NACK                    0xFFFF /*Transmit 1 to send NACK as acknowledge*/

#define I2C_ACK_EN                  0xFFFF/*Initiate Acknowledge sequence */
#define I2C_ACK_DIS                 0xFFEF /*Acknowledge condition Idle */

#define I2C_RCV_EN                  0xFFFF /*Enable receive mode */
#define I2C_RCV_DIS                 0xFFF7 /*Receive sequence not in progress */
 
#define I2C_STOP_EN                 0xFFFF /*Initiate Stop sequence */
#define I2C_STOP_DIS                0xFFFB /*Stop condition Idle */

#define I2C_RESTART_EN              0xFFFF /*Initiate Restart sequence */
#define I2C_RESTART_DIS             0xFFFD /*Start condition Idle */

#define I2C_START_EN                0xFFFF /*Initiate Start sequence */
#define I2C_START_DIS               0xFFFE /*Start condition Idle */

/* Priority for Slave I2C1 Interrupt */
#define SI2C_INT_PRI_7              0xFFFF /*Slave I2C Interrupt Priority 0*/
#define SI2C_INT_PRI_6              0xFFFE /*Slave I2C Interrupt Priority 1*/
#define SI2C_INT_PRI_5              0xFFFD /*Slave I2C Interrupt Priority 2*/
#define SI2C_INT_PRI_4              0xFFFC /*Slave I2C Interrupt Priority 3*/
#define SI2C_INT_PRI_3              0xFFFB /*Slave I2C Interrupt Priority 4*/
#define SI2C_INT_PRI_2              0xFFFA /*Slave I2C Interrupt Priority 5*/
#define SI2C_INT_PRI_1              0xFFF9 /*Slave I2C Interrupt Priority 6*/
#define SI2C_INT_PRI_0              0xFFF8 /*Slave I2C Interrupt Priority 7*/

/* Slave I2C1 Interrupt Enable/Disable */
#define SI2C_INT_ON                 0xFFFF /*Slave I2C Interrupt enable*/
#define SI2C_INT_OFF                0xFFF7 /*Slave I2C Interrupt disable*/

/* Priority for Master I2C1 Interrupt */
#define MI2C_INT_PRI_7              0xFFFF /*Master I2C Interrupt Priority 0*/
#define MI2C_INT_PRI_6              0xFFEF /*Master I2C Interrupt Priority 1*/
#define MI2C_INT_PRI_5              0xFFDF /*Master I2C Interrupt Priority 2*/
#define MI2C_INT_PRI_4              0xFFCF /*Master I2C Interrupt Priority 3*/
#define MI2C_INT_PRI_3              0xFFBF /*Master I2C Interrupt Priority 4*/
#define MI2C_INT_PRI_2              0xFFAF /*Master I2C Interrupt Priority 5*/
#define MI2C_INT_PRI_1              0xFF9F /*Master I2C Interrupt Priority 6*/
#define MI2C_INT_PRI_0              0xFF8F /*Master I2C Interrupt Priority 7*/

/* Master I2C1 Interrupt Enable/Disable */
#define MI2C_INT_ON                 0xFFFF /*Master I2C Interrupt enabled*/
#define MI2C_INT_OFF                0xFF7F /*Master I2C Interrupt disabled*/

#else /* Format for backward compatibility (AND based bit setting). */

// I2CxCON Register Configuration Bit Definitions
#define I2C_ON           			0x8000 /*I2C module enabled */
#define I2C_OFF          			0x0000 /*I2C module disabled */
#define I2C_ON_OFF_MASK  			(~I2C_ON)

#define I2C_IDLE_STOP    			0x2000 /*stop I2C module in Idle mode */
#define I2C_IDLE_CON     			0x0000 /*continue I2C module in Idle mode */
#define I2C_IDLE_MASK    			(~I2C_IDLE_STOP)

#define I2C_CLK_REL      			0x1000 /*release clock */
#define I2C_CLK_HLD      			0x0000 /*hold clock  */
#define I2C_CLK_MASK     			(~I2C_CLK_REL)

#define I2C_IPMI_EN      			0x0800 /*IPMI mode enabled */
#define I2C_IPMI_DIS     			0x0000 /*IPMI mode not enabled */
#define I2C_IPMI_EN_DIS_MASK    	(~I2C_IPMI_EN)

#define I2C_10BIT_ADD    			0x0400 /*I2CADD is 10-bit address */
#define I2C_7BIT_ADD     			0x0000 /*I2CADD is 7-bit address */
#define I2C_10BIT_7BIT_MASK    		(~I2C_10BIT_ADD)

#define I2C_SLW_DIS       			0x0200 /*Disable Slew Rate Control for 100KHz */
#define I2C_SLW_EN        			0x0000 /*Enable Slew Rate Control for 400KHz */
#define I2C_SLW_EN_DIS_MASK     	(~I2C_SLW_DIS)

#define I2C_SM_EN        			0x0100 /*Enable SM bus specification */
#define I2C_SM_DIS       			0x0000 /*Disable SM bus specification */
#define I2C_SM_EN_DIS_MASK      	(~I2C_SM_EN)

#define I2C_GCALL_EN     			0x0080 /*Enable Interrupt when General call address is received. */
#define I2C_GCALL_DIS    			0x0000 /*Disable General call address. */
#define I2C_GCALL_EN_DIS_MASK   	(~I2C_GCALL_EN)

#define I2C_STR_EN       			0x0040 /*Enable clock stretching */
#define I2C_STR_DIS      			0x0000 /*disable clock stretching */
#define I2C_STR_EN_DIS_MASK     	(~I2C_STR_EN)

#define I2C_NACK         			0x0020 /*Transmit 0 to send ACK as acknowledge */
#define I2C_ACK         			0x0000 /*Transmit 1 to send NACK as acknowledge*/
#define I2C_ACK_MASK     			(~I2C_NACK)

#define I2C_ACK_EN       			0x0010 /*Initiate Acknowledge sequence */
#define I2C_ACK_DIS      			0x0000 /*Acknowledge condition Idle */
#define I2C_TX_ACK_MASK  			(~I2C_ACK_EN)

#define I2C_RCV_EN       			0x0008 /*Enable receive mode */
#define I2C_RCV_DIS      			0x0000 /*Receive sequence not in progress */
#define I2C_RCV_EN_DIS_MASK         (~I2C_RCV_EN)

#define I2C_STOP_EN      			0x0004 /*Initiate Stop sequence */
#define I2C_STOP_DIS    			0x0000 /*Stop condition Idle */
#define I2C_STOP_EN_DIS_MASK    	(~I2C_STOP_EN)

#define I2C_RESTART_EN   			0x0002 /*Initiate Restart sequence */
#define I2C_RESTART_DIS  			0x0000 /*Start condition Idle */
#define I2C_RESTART_MASK 			(~I2C_RESTART_EN)

#define I2C_START_EN     			0x0001 /*Initiate Start sequence */
#define I2C_START_DIS    			0x0000 /*Start condition Idle */
#define I2C_START_MASK   			(~I2C_START_EN)

#define SI2C_INT_PRI_0      		0x0000 /*Slave I2C Interrupt Priority 0*/
#define SI2C_INT_PRI_1      		0x0001 /*Slave I2C Interrupt Priority 1*/
#define SI2C_INT_PRI_2     			0x0002 /*Slave I2C Interrupt Priority 2*/
#define SI2C_INT_PRI_3      		0x0003 /*Slave I2C Interrupt Priority 3*/
#define SI2C_INT_PRI_4      		0x0004 /*Slave I2C Interrupt Priority 4*/
#define SI2C_INT_PRI_5      		0x0005 /*Slave I2C Interrupt Priority 5*/
#define SI2C_INT_PRI_6      		0x0006 /*Slave I2C Interrupt Priority 6*/
#define SI2C_INT_PRI_7      		0x0007 /*Slave I2C Interrupt Priority 7*/
#define SI2C_SRC_DIS        		SI2C_INT_PRI_0 /*Slave I2C Source is disabled*/
#define SI2C_INT_PRI_MASK  			(~SI2C_INT_PRI_7)

#define SI2C_INT_ON     			0x0008 /*Slave I2C Interrupt enable*/
#define SI2C_INT_OFF    			0x0000 /*Slave I2C Interrupt disable*/
#define SI2C_INT_MASK   			(~SI2C_INT_ON)

#define MI2C_INT_PRI_0  			0x0000 /*Master I2C Interrupt Priority 0*/
#define MI2C_INT_PRI_1  			0x0010 /*Master I2C Interrupt Priority 1*/
#define MI2C_INT_PRI_2  			0x0020 /*Master I2C Interrupt Priority 2*/
#define MI2C_INT_PRI_3  			0x0030 /*Master I2C Interrupt Priority 3*/
#define MI2C_INT_PRI_4  			0x0040 /*Master I2C Interrupt Priority 4*/
#define MI2C_INT_PRI_5  			0x0050 /*Master I2C Interrupt Priority 5*/
#define MI2C_INT_PRI_6  			0x0060 /*Master I2C Interrupt Priority 6*/
#define MI2C_INT_PRI_7  			0x0070 /*Master I2C Interrupt Priority 7*/
#define MI2C_SRC_DIS    			MI2C_INT_PRI_0 /*Master I2C Source is disabled*/
#define MI2C_INT_PRI_MASK  			(~MI2C_INT_PRI_7)

// Master I2C Interrupt Enable/Disable
#define MI2C_INT_ON     			0x0080 /*Master I2C Interrupt enabled*/
#define MI2C_INT_OFF    			0x0000 /*Master I2C Interrupt disabled*/
#define MI2C_INT_MASK   			(~MI2C_INT_ON)

#endif /* USE_AND_OR */

#if defined (i2c_v1_1) || defined (i2c_v1_2)|| defined (i2c_v1_3)||defined(LIB_BUILD)
/***********************************************************************************
Macro       : EnableIntSI2C1

Include     : i2c.h

Description : Macro enables I2C Slave Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define EnableIntSI2C1                   (IEC1bits.SI2C1IE = 1) 

/***********************************************************************************
Macro       : DisableIntSI2C1

Include     : i2c.h

Description : Macro disables I2C Slave Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define DisableIntSI2C1                  (IEC1bits.SI2C1IE = 0) 

/***********************************************************************************
Macro       : SetPriorityIntSI2C1(priority)
 
Include     : i2c.h
 
Description : Macro sets the priority level for I2C Salve interrupt.
 
Arguments   : priority - This input parameter is the level of interrupt priority.
 
Remarks     : None
***********************************************************************************/
#define SetPriorityIntSI2C1(priority)     (IPC4bits.SI2C1P = priority)

/*******************************************************************
Macro       : SI2C1_Clear_Intr_Status_Bit

Include     : i2c.h 

Description : Macro to Clear I2C Slave Interrupt Status bit 

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define SI2C1_Clear_Intr_Status_Bit     (IFS1bits.SI2C1IF = 0)

/***********************************************************************************
Macro       : EnableIntMI2C1

Include     : i2c.h

Description : Macro enables I2C Master Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define EnableIntMI2C1                   (IEC1bits.MI2C1IE = 1) 

/***********************************************************************************
Macro       : DisableIntMI2C1

Include     : i2c.h

Description : Macro disables I2C Master Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define DisableIntMI2C1                   (IEC1bits.MI2C1IE = 0) 

/***********************************************************************************
Macro       : SetPriorityIntMI2C1(priority)
 
Include     : i2c.h
 
Description : Macro sets the priority level for I2C Master interrupt.
 
Arguments   : priority - This input parameter is the level of interrupt priority.
 
Remarks     : None
***********************************************************************************/
#define SetPriorityIntMI2C1(priority)     (IPC4bits.MI2C1P = priority)

/*******************************************************************
Macro       : MI2C1_Clear_Intr_Status_Bit

Include     : i2c.h 

Description : Macro to Clear I2C Master Interrupt Status bit 

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define MI2C1_Clear_Intr_Status_Bit     (IFS1bits.MI2C1IF = 0)

/***********************************************************************************
Macro       : SlavegetcI2C1
 
Include     : i2c.h
 
Description : This function is identical to SlaveReadI2C1              
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define SlavegetcI2C1                     SlaveReadI2C1

/***********************************************************************************
Macro       : MastergetcI2C1
 
Include     : i2c.h
 
Description : This function is identical to MasterReadI2C1 
             
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define MastergetcI2C1                    MasterReadI2C1

/***********************************************************************************
Macro       : SlaveputcI2C1
 
Include     : i2c.h
 
Description : This function is identical to SlaveWriteI2C1. 
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define SlaveputcI2C1                    SlaveWriteI2C1

/***********************************************************************************
Macro       : MasterputcI2C1
 
Include     : i2c.h
 
Description : This function is identical to MasterWriteI2C1. 
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define MasterputcI2C1                  MasterWriteI2C1

/***********************************************************************
Macro       : mI2C1_MasterClearIntr()
 
Include     : i2c.h
 
Description : This macro clears the I2C Master interrupt flag
 
Arguments   : None
 
Remarks     : None
************************************************************************/
#define mI2C1_MasterClearIntr()          (IFS1bits.MI2C1IF = 0)

/***********************************************************************
Macro       : mI2C1_SlaveClearIntr()
 
Include     : i2c.h
 
Description : This macro clears the I2C Slave interrupt flag
 
Arguments   : None
 
Remarks     : None
************************************************************************/
#define mI2C1_SlaveClearIntr()           (IFS1bits.SI2C1IF = 0)

#if defined (i2c_v1_1) || defined(LIB_BUILD)
/***********************************************************************
Macro       : I2C_SMB_SDA_Delay_LONG()
 
Include     : i2c.h
 
Description : This macro sets the I2C for longer (300ns) SMBus SDA delay
 
Arguments   : None
 
Remarks     : None
************************************************************************/
#define I2C_SMB_SDA_Delay_LONG()           (PADCFG1bits.SMBUSDEL = 1)

/***********************************************************************
Macro       : I2C_SMB_SDA_Delay_LONG()
 
Include     : i2c.h
 
Description : This macro sets the I2C for legacy mode (150ns) SMBus SDA delay
 
Arguments   : None
 
Remarks     : None
************************************************************************/
#define I2C_SMB_SDA_Delay_LEGACY()           (PADCFG1bits.SMBUSDEL = 0)
#endif

void __attribute__ ((section (".libperi")))AckI2C1(void);

void __attribute__ ((section (".libperi")))CloseI2C1(void) ;

void __attribute__ ((section (".libperi")))ConfigIntI2C1(unsigned int config) ;

char __attribute__ ((section (".libperi")))DataRdyI2C1(void) ;

unsigned int __attribute__ ((section (".libperi")))SlavegetsI2C1(unsigned char *rdptr, unsigned int i2c_data_wait) ;

unsigned int __attribute__ ((section (".libperi")))MastergetsI2C1(unsigned int length, unsigned char * rdptr, unsigned int i2c_data_wait) ;

void __attribute__ ((section (".libperi")))IdleI2C1(void) ;

void __attribute__ ((section (".libperi")))NotAckI2C1(void) ;

void __attribute__ ((section (".libperi")))OpenI2C1(unsigned int config1,unsigned int config2) ;

char __attribute__ ((section (".libperi"))) MasterputsI2C1(unsigned char *wrptr) ;

unsigned int __attribute__ ((section (".libperi"))) SlaveputsI2C1(unsigned char *wrptr) ;

unsigned char __attribute__ ((section (".libperi")))SlaveReadI2C1(void) ;

unsigned char __attribute__ ((section (".libperi")))MasterReadI2C1(void) ;

void __attribute__ ((section (".libperi")))SlaveWriteI2C1(unsigned char data_out) ;

char __attribute__ ((section (".libperi")))MasterWriteI2C1(unsigned char data_out) ;

void __attribute__ ((section (".libperi")))RestartI2C1(void) ;

void __attribute__ ((section (".libperi")))StartI2C1(void) ;

void __attribute__ ((section (".libperi")))StopI2C1(void) ;

void __attribute__ ((section (".libperi")))SlaveWaitForIntrI2C1(void) ;

void __attribute__ ((section (".libperi")))MasterWaitForIntrI2C1(void) ;

#endif

#if defined (i2c_v1_2)|| defined (i2c_v1_3)||defined(LIB_BUILD)

/***********************************************************************************
Macro       : EnableIntSI2C2

Include     : i2c.h

Description : Macro enables I2C Slave Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define EnableIntSI2C2                   (IEC3bits.SI2C2IE = 1) 

/***********************************************************************************
Macro       : DisableIntSI2C2

Include     : i2c.h

Description : Macro disables I2C Slave Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define DisableIntSI2C2                   (IEC3bits.SI2C2IE = 0)

/***********************************************************************************
Macro       : SetPriorityIntSI2C2(priority)
 
Include     : i2c.h
 
Description : Macro sets the priority level for I2C Salve interrupt.
 
Arguments   : priority - This input parameter is the level of interrupt priority.
 
Remarks     : None
***********************************************************************************/
#define SetPriorityIntSI2C2(priority)     (IPC12bits.SI2C2P = priority)

/*******************************************************************
Macro       : SI2C2_Clear_Intr_Status_Bit

Include     : i2c.h 

Description : Macro to Clear I2C Slave Interrupt Status bit 

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define SI2C2_Clear_Intr_Status_Bit     (IFS3bits.SI2C2IF = 0)

/***********************************************************************************
Macro       : EnableIntMI2C1

Include     : i2c.h

Description : Macro enables I2C Master Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define EnableIntMI2C2                   (IEC3bits.MI2C2IE = 1)

/***********************************************************************************
Macro       : DisableIntMI2C2

Include     : i2c.h

Description : Macro disables I2C Master Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define DisableIntMI2C2                   (IEC3bits.MI2C2IE = 0)

/***********************************************************************************
Macro       : SetPriorityIntMI2C2(priority)
 
Include     : i2c.h
 
Description : Macro sets the priority level for I2C Master interrupt.
 
Arguments   : priority - This input parameter is the level of interrupt priority.
 
Remarks     : None
***********************************************************************************/
#define SetPriorityIntMI2C2(priority)     (IPC12bits.MI2C2P = priority)

/*******************************************************************
Macro       : MI2C2_Clear_Intr_Status_Bit

Include     : i2c.h 

Description : Macro to Clear I2C Master Interrupt Status bit 

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define MI2C2_Clear_Intr_Status_Bit     (IFS3bits.MI2C2IF = 0)

/***********************************************************************************
Macro       : SlavegetcI2C2
 
Include     : i2c.h
 
Description : This function is identical to SlaveReadI2C2. 
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define SlavegetcI2C2                      SlaveReadI2C2

/***********************************************************************************
Macro       : MastergetcI2C2
 
Include     : i2c.h
 
Description : This function is identical to MasterReadI2C2.
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define MastergetcI2C2                      MasterReadI2C2

/***********************************************************************************
Macro       : SlaveputcI2C2
 
Include     : i2c.h
 
Description : This function is identical to SlaveWriteI2C2.
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define SlaveputcI2C2                       SlaveWriteI2C2

/***********************************************************************************
Macro       : MasterputcI2C2
 
Include     : i2c.h
 
Description : This function is identical to MasterWriteI2C2. 
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define MasterputcI2C2                     MasterWriteI2C2

/***********************************************************************
Macro       : mI2C2_MasterClearIntr()
 
Include     : i2c.h
 
Description : This macro clears the I2C Master interrupt flag
 
Arguments   : None
 
Remarks     : None
************************************************************************/
#define mI2C2_MasterClearIntr()   (IFS3bits.MI2C2IF = 0)

/***********************************************************************
Macro       : mI2C2_SlaveClearIntr()
 
Include     : i2c.h
 
Description : This macro clears the I2C Slave interrupt flag
 
Arguments   : None
 
Remarks     : None
************************************************************************/
#define mI2C2_SlaveClearIntr()   (IFS3bits.SI2C2IF = 0)


void __attribute__ ((section (".libperi")))AckI2C2(void) ;

void __attribute__ ((section (".libperi")))CloseI2C2(void) ;

void __attribute__ ((section (".libperi")))ConfigIntI2C2(unsigned int config) ;

char __attribute__ ((section (".libperi")))DataRdyI2C2(void) ;

unsigned int __attribute__ ((section (".libperi")))SlavegetsI2C2(unsigned char *rdptr, unsigned int i2c_data_wait) ;

unsigned int __attribute__ ((section (".libperi")))MastergetsI2C2(unsigned int length, unsigned char * rdptr, unsigned int i2c_data_wait) ;

void __attribute__ ((section (".libperi")))IdleI2C2(void) ;

void __attribute__ ((section (".libperi")))NotAckI2C2(void) ;

void  __attribute__ ((section (".libperi")))OpenI2C2(unsigned int config1,unsigned int config2);

char __attribute__ ((section (".libperi")))MasterputsI2C2(unsigned char *wrptr) ;

unsigned int __attribute__ ((section (".libperi"))) SlaveputsI2C2(unsigned char *wrptr) ;

unsigned char __attribute__ ((section (".libperi")))SlaveReadI2C2(void) ;

unsigned char __attribute__ ((section (".libperi"))) MasterReadI2C2(void) ;

void __attribute__ ((section (".libperi")))SlaveWriteI2C2(unsigned char data_out) ;

char __attribute__ ((section (".libperi")))MasterWriteI2C2(unsigned char data_out) ;

void __attribute__ ((section (".libperi")))RestartI2C2(void) ;

void __attribute__ ((section (".libperi")))StartI2C2(void) ;

void __attribute__ ((section (".libperi")))StopI2C2(void) ;

void __attribute__ ((section (".libperi")))SlaveWaitForIntrI2C2(void) ;

void __attribute__ ((section (".libperi")))MasterWaitForIntrI2C2(void);

#endif 

#if defined (i2c_v1_3)||defined(LIB_BUILD)
/***********************************************************************************
Macro       : EnableIntSI2C3

Include     : i2c.h

Description : Macro enables I2C Slave Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define EnableIntSI2C3                   (IEC5bits.SI2C3IE = 1)

/***********************************************************************************
Macro       : DisableIntSI2C3

Include     : i2c.h

Description : Macro disables I2C Slave Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define DisableIntSI2C3                   (IEC5bits.SI2C3IE = 0)

/***********************************************************************************
Macro       : SetPriorityIntSI2C3(priority)
 
Include     : i2c.h
 
Description : Macro sets the priority level for I2C Slave interrupt.
 
Arguments   : priority - This input parameter is the level of interrupt priority.
 
Remarks     : None
***********************************************************************************/
#define SetPriorityIntSI2C3(priority)     (IPC21bits.SI2C3P = priority)

/*******************************************************************
Macro       : SI2C3_Clear_Intr_Status_Bit

Include     : i2c.h 

Description : Macro to Clear I2C Slave Interrupt Status bit 

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define SI2C3_Clear_Intr_Status_Bit     (IFS5bits.SI2C3IF = 0)

/***********************************************************************************
Macro       : EnableIntMI2C3

Include     : i2c.h

Description : Macro enables I2C Master Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define EnableIntMI2C3                   (IEC5bits.MI2C3IE = 1)

/***********************************************************************************
Macro       : DisableIntMI2C3

Include     : i2c.h

Description : Macro disables I2C Master Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define DisableIntMI2C3                   (IEC5bits.MI2C3IE = 0)

/***********************************************************************************
Macro       : SetPriorityIntMI2C3(priority)
 
Include     : i2c.h
 
Description : Macro sets the priority level for I2C Master interrupt.
 
Arguments   : priority - This input parameter is the level of interrupt priority.
 
Remarks     : None
***********************************************************************************/
#define SetPriorityIntMI2C3(priority)     (IPC21bits.MI2C3P = priority)

/*******************************************************************
Macro       : MI2C3_Clear_Intr_Status_Bit

Include     : i2c.h 

Description : Macro to Clear I2C Slave Interrupt Status bit 

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define MI2C3_Clear_Intr_Status_Bit     (IFS5bits.MI2C3IF = 0)

/***********************************************************************************
Macro       : SlavegetcI2C3
 
Include     : i2c.h
 
Description : This function is identical to SlaveReadI2C3. 
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define SlavegetcI2C3						SlaveReadI2C3

/***********************************************************************************
Macro       : MastergetcI2C3
 
Include     : i2c.h
 
Description : This function is identical to MasterReadI2C3. 
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define MastergetcI2C3						 MasterReadI2C3

/***********************************************************************************
Macro       : SlaveputcI2C3
 
Include     : i2c.h
 
Description : This function is identical to SlaveWriteI2C3 
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define SlaveputcI2C3                      SlaveWriteI2C3

/***********************************************************************************
Macro       : MasterputcI2C3
 
Include     : i2c.h
 
Description : This function is identical to MasterWriteI2C3.
                
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define MasterputcI2C3                        MasterWriteI2C3

/***********************************************************************
Macro       : mI2C3_MasterClearIntr()
 
Include     : i2c.h
 
Description : This macro clears the I2C Master interrupt flag
 
Arguments   : None
 
Remarks     : None
************************************************************************/
#define mI2C3_MasterClearIntr()   			(IFS5bits.MI2C3IF = 0)

/***********************************************************************
Macro       : mI2C3_SlaveClearIntr()
 
Include     : i2c.h
 
Description : This macro clears the I2C Slave interrupt flag
 
Arguments   : None
 
Remarks     : None
************************************************************************/
#define mI2C3_SlaveClearIntr()   			(IFS5bits.SI2C3IF = 0)

void __attribute__ ((section (".libperi")))AckI2C3(void) ;

void __attribute__ ((section (".libperi")))CloseI2C3(void) ;

void __attribute__ ((section (".libperi")))ConfigIntI2C3(unsigned int config) ;

char __attribute__ ((section (".libperi")))DataRdyI2C3(void) ;

unsigned int __attribute__ ((section (".libperi")))SlavegetsI2C3(unsigned char *rdptr, unsigned int i2c_data_wait) ;

unsigned int __attribute__ ((section (".libperi")))MastergetsI2C3(unsigned int length, unsigned char * rdptr, unsigned int i2c_data_wait) ;

void __attribute__ ((section (".libperi"))) IdleI2C3(void) ;

void __attribute__ ((section (".libperi")))NotAckI2C3(void) ;

void __attribute__ ((section (".libperi")))OpenI2C3(unsigned int config1,unsigned int config2) ;

char __attribute__ ((section (".libperi")))MasterputsI2C3(unsigned char *wrptr) ;

unsigned int __attribute__ ((section (".libperi"))) SlaveputsI2C3(unsigned char *wrptr) ;

unsigned char __attribute__ ((section (".libperi")))SlaveReadI2C3(void) ;

unsigned char __attribute__ ((section (".libperi"))) MasterReadI2C3(void) ;

void __attribute__ ((section (".libperi")))SlaveWriteI2C3(unsigned char data_out) ;

char __attribute__ ((section (".libperi"))) MasterWriteI2C3(unsigned char data_out) ;

void __attribute__ ((section (".libperi")))RestartI2C3(void) ;

void __attribute__ ((section (".libperi")))StartI2C3(void) ;

void __attribute__ ((section (".libperi")))StopI2C3(void) ;

void __attribute__ ((section (".libperi")))SlaveWaitForIntrI2C3(void) ;

void __attribute__ ((section (".libperi")))MasterWaitForIntrI2C3(void) ;

#endif /* _I2C_3 */

#else		//This preprocessor conditional statement is to avoid unintended linking for unsuppported devices.
#warning "Does not build on this target"
#endif

#endif /*I2C_H  */

