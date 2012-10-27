/*==============================================================================
File: XboxController.h

Description: This class organizes an interface to an Xbox controller built on
  top of Simple DirectMedia Layer library (libsdl).

Notes:
  - joystick left is negative, right is positive 
  - joystick up is negative, down is positive
  - trigger pulled is positive, unpulled is negative
  - http://en.wikipedia.org/wiki/Xbox_360_Controller#Layout

Responsible Engineer: Stellios Leventis sleventis@robotex.com
==============================================================================*/
#pragma once
/*---------------------------Dependencies-------------------------------------*/
#include "./SDL/SDL.h"  // Simple DirectMedia Layer library

/*---------------------------Type Definitions---------------------------------*/
typedef enum {
 kMinAxisValue = -32768,
 kMaxAxisValue = 32767
} kAxisLimit;

typedef enum {
  kNorth = 1,
  kNorthEast = 3, // Note: the intercardinal directions are the 
  kEast = 2,      // summation of the neighboring cardinal directions
  kSouthEast = 6,
  kSouth = 4,
  kSouthWest = 12,
  kWest = 8,
  kNorthWest = 9
} kDPadDirection;

typedef struct {
  uint8_t direction;
} XboxControllerDPad;

typedef bool XboxControllerButton;

typedef struct {
  int16_t x;
  int16_t y;
  XboxControllerButton button;  // when the joystick is depressed
} XboxControllerJoystick;

typedef struct {
  int16_t pull;                 // how far the trigger is depressed
} XboxControllerTrigger;



class XboxController {
  public:
  	/*---Public Methods-------------------------------------------------------*/
    XboxController();
	  ~XboxController();

  	/*---Public Instance Methods----------------------------------------------*/
    void getUpdatedValues(void);

    /*---Public Instance Variables--------------------------------------------*/
    XboxControllerButton A;
    XboxControllerButton B;
    XboxControllerButton X;
    XboxControllerButton Y;
    XboxControllerButton rightBumper;
    XboxControllerButton leftBumper;
    XboxControllerButton Guide;
    XboxControllerButton Start;
    XboxControllerButton Back;
    XboxControllerJoystick leftStick;
    XboxControllerJoystick rightStick;
    XboxControllerTrigger leftTrigger;
    XboxControllerTrigger rightTrigger;
    XboxControllerButton Connect;
    XboxControllerDPad dPad;
  private:
    /*---Private Methods------------------------------------------------------*/

    /*---Private Instance Variables-------------------------------------------*/
    SDL_Joystick* myJoystick;
};


