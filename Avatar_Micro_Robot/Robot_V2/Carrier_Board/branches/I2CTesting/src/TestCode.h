/*******************************************************************************
File: TestCode.h

Description: This module provides several functions to help test the Carrier
  PCBA.
*******************************************************************************/
#ifndef TESTCODE_H
#define TESTCODE_H

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: DisplayBoardNumber
Description: Displays the board number
*******************************************************************************/
void DisplayBoardNumber(void);


/*******************************************************************************
Function: SendLCDString
Parameters:
  char s[],               the string (array of characters) to print 
  unsigned char length,   the length of the string (1-based indexing)
*******************************************************************************/
void SendLCDString(char s[], unsigned char length);


void PrintLoopNumber(void);
void DisplayInt(char *description, unsigned int intToDisplay);
void InitTestCode(void);

#endif
