/*==============================================================================
File: XboxController.cpp
==============================================================================*/
//#define TEST_XBOX_CONTROLLER
/*---------------------------Dependencies-------------------------------------*/
#include "./XboxController.hpp"

/*---------------------------Macros-------------------------------------------*/
#define NUM_CONTROLLER_BUTTONS  11 // number of buttons on the Xbox controller

/*---------------------------Type Definitions---------------------------------*/
typedef enum {
  kAButton = 0,
  kBButton,
  kXButton,
  kYButton,
  kLeftBumper,
  kRightBumper,
  kBackButton,
  kStartButton,
  kGuideButton,
  kLeftStick,     // when left joystick is depressed
  kRightStick,    // when right joystick is depressed
  kConnectButton, // UNVALIDATED, only on wireless controllers
} XboxControllerButtonIndex;

typedef enum {
  kLeftX = 0,
  kLeftY,
  kLeftPull,
  kRightX,
  kRightY,
  kRightPull
} XboxControllerAxisIndex;

#define DPAD_HAT  0

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_XBOX_CONTROLLER
#include <unistd.h>	    // for blocking usleep() function

void PrintControllerValues(XboxController *controller) {
  printf("\r\nMy Controller\r\n");
  printf("--------------------\r\n");
  printf("A: %d\r\n", controller->A);
  printf("B: %d\r\n", controller->B);
  printf("X: %d\r\n", controller->X);
  printf("Y: %d\r\n", controller->Y);
  printf("leftBumper: %d\r\n", controller->leftBumper);
  printf("rightBumper: %d\r\n", controller->rightBumper);
  printf("Back: %d\r\n", controller->Back);
  printf("Start: %d\r\n", controller->Start);
  printf("Guide: %d\r\n", controller->Guide);
  printf("leftStick.x: %d\r\n", (controller->leftStick).x);
  printf("leftStick.y: %d\r\n", (controller->leftStick).y);
  printf("leftStick.button: %d\r\n", (controller->leftStick).button);
  printf("rightStick.x: %d\r\n", (controller->rightStick).x);
  printf("rightStick.y: %d\r\n", (controller->rightStick).y);
  printf("rightStick.button: %d\r\n", (controller->rightStick).button);
  printf("Connect: %d\r\n", controller->Connect);
  printf("lefTrigger.pull: %d\r\n", (controller->leftTrigger).pull);  
  printf("rightTrigger.pull: %d\r\n", (controller->rightTrigger).pull);  
  printf("dPad.direction: %d\r\n", (controller->dPad).direction);
}

int main(int argn, char *argc[]) {
  XboxController myController;  

  while (true) {
    myController.getUpdatedValues();
    usleep(5000);
    PrintControllerValues(&myController);
  }

  return EXIT_SUCCESS;
}
#endif
/*---------------------------Public Method Definitions------------------------*/
XboxController::XboxController() {
  if (SDL_Init(SDL_INIT_JOYSTICK) < 0) printf("Error: trouble in SDL_Init()\n");
  if (SDL_NumJoysticks() < 1) printf("Error: Couldn't find controller\n");
  SDL_JoystickEventState(SDL_IGNORE);
  myJoystick = SDL_JoystickOpen(0);
  if (!myJoystick) printf("\r\ncould NOT open joystick 0\r\n");
}

XboxController::~XboxController() {
  SDL_JoystickClose(this->myJoystick);
  // free all internal objects out
}

void XboxController::getUpdatedValues(void) {
  SDL_JoystickUpdate();

  this->A = SDL_JoystickGetButton(this->myJoystick, kAButton);
  this->B = SDL_JoystickGetButton(this->myJoystick, kBButton);
  this->X = SDL_JoystickGetButton(this->myJoystick, kXButton);
  this->Y = SDL_JoystickGetButton(this->myJoystick, kYButton);
  this->rightBumper = SDL_JoystickGetButton(this->myJoystick, kRightBumper);
  this->leftBumper = SDL_JoystickGetButton(this->myJoystick, kLeftBumper);
  this->Guide = SDL_JoystickGetButton(this->myJoystick, kGuideButton);
  this->Start = SDL_JoystickGetButton(this->myJoystick, kStartButton);
  this->Back = SDL_JoystickGetButton(this->myJoystick, kBackButton);
  (this->leftStick).x = SDL_JoystickGetAxis(this->myJoystick, kLeftX);
  (this->leftStick).y = SDL_JoystickGetAxis(this->myJoystick, kLeftY);
  (this->leftStick).button = SDL_JoystickGetButton(this->myJoystick, kLeftStick);  
  (this->rightStick).x = SDL_JoystickGetAxis(this->myJoystick, kRightX);
  (this->rightStick).y = SDL_JoystickGetAxis(this->myJoystick, kRightY);
  (this->rightStick).button = SDL_JoystickGetButton(this->myJoystick, kRightStick); 
  this->Connect = SDL_JoystickGetButton(this->myJoystick, kConnectButton);
  (this->leftTrigger).pull = SDL_JoystickGetAxis(this->myJoystick, kLeftPull);
  (this->rightTrigger).pull = SDL_JoystickGetAxis(this->myJoystick, kRightPull);
  (this->dPad).direction = SDL_JoystickGetHat(this->myJoystick, DPAD_HAT);
}

/*---------------------------Helper Function Definitions----------------------*/





