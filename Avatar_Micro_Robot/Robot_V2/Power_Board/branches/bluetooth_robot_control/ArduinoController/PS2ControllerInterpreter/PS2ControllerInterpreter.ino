/*==============================================================================
File: PS2ControllerInterpreter.ino

Description: This is the overarching file that manages the interface between a
  PS2 Controller and a BLE112 bluetooth module.  It interprets the controller
  values through an open-source arduino-based library and then outputs
  relevant data in a defined packet on the USART pins.
  
Notes:
  - see billporter.info for PS2X library updates and documentation
  - does NOT support hot-pluggable controllers, meaning you must always either
    restart your Arduino or re-initialize the gamepad pins in code after you
    connect the controller
    
TODO:
  - determine if read_gamepad() takes time or just reads values from buffers
  - find out why button presses don't seem to be working (pressure sensing in general?)
  - why does it only recognize this controller as a "DualShock" sometimes?
    
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
/*---------------------------Dependencies-------------------------------------*/
#include <Timers.h>
#include <PS2X_lib.h>
#include "Protocol.h"

/*---------------------------Macros-------------------------------------------*/
// PS2 controller pins
#define CLK_PIN        13
#define CMD_PIN        11
#define ATT_PIN        12
#define DATA_PIN       10

// for protocol
#define N_DATA_BYTES   7
#define PINK_BIT       0
#define RED_BIT        1
#define BLUE_BIT       2
#define GREEN_BIT      3

// times and timers
#define TX_TIMER       0
#define TX_TIME        2000  // [ms]
#define UPDATE_TIMER   1
#define UPDATE_TIME    50

/*---------------------------Constants----------------------------------------*/
static const uint16_t kBaudRate = 9600;

// for PSX library
static const uint8_t kControllerUnknown = 0;
static const uint8_t kControllerDualShock = 1;
static const uint8_t kControllerGuitarHero = 2;
static const bool kUsePressures = true;
static const bool kUseRumble = true;

/*---------------------------Helper Function Prototypes-----------------------*/
void PrintInitializationFeedback(PS2X ps2x, uint8_t error);
void PrintArrayAsHex(uint8_t tx_packet[], uint8_t tx_packet_length);
void PrintControllerValues(PS2X ps2x);
void PrintControllerType(PS2X ps2x);

/*---------------------------Module Variables---------------------------------*/
PS2X my_controller;
static uint8_t tx_packet[PROT_MAX_PACKET_LENGTH] = {0};
static  uint8_t tx_packet_length = 0;

/*---------------------------Module Code--------------------------------------*/
void setup() {
  Serial.begin(kBaudRate);
  int error = my_controller.config_gamepad(CLK_PIN, CMD_PIN, ATT_PIN, DATA_PIN,
                                           kUsePressures, kUseRumble);
  PrintInitializationFeedback(my_controller, error);
  
  // prime any timers that require it
  TMRS_StartTimer(TX_TIMER, 0);
  TMRS_StartTimer(UPDATE_TIMER, 0);
}

void loop() {
  // get new values from the controller
  if (TMRS_IsTimerExpired(UPDATE_TIMER)) {
    TMRS_StartTimer(UPDATE_TIMER, UPDATE_TIME);
    my_controller.read_gamepad();
    /*
    PrintControllerValues(my_controller);
    if (my_controller.NewButtonState(PSB_BLUE)) {
      Serial.println("X just changed");
      delay(20);
      while (1);
    }
    */
  }
  
  // tranmist updated controller values
  if (TMRS_IsTimerExpired(TX_TIMER)) {
    TMRS_StartTimer(TX_TIMER, TX_TIME);
    my_controller.read_gamepad();
    
    static uint8_t data_out[N_DATA_BYTES] = {0};
    data_out[0] = (uint8_t) (my_controller.Analog(PSS_LY) >> 8);
    data_out[1] = (uint8_t) (my_controller.Analog(PSS_LY) & 0xff);
    data_out[2] = (uint8_t) (my_controller.Analog(PSS_LX) >> 8);
    data_out[3] = (uint8_t) (my_controller.Analog(PSS_LX) & 0xff);
    data_out[4] = (uint8_t) my_controller.Button(PSB_L1);
    data_out[5] = (uint8_t) my_controller.Button(PSB_R1);
    data_out[6] = ((my_controller.ButtonPressed(PSB_PINK) << PINK_BIT) | 
                   (my_controller.ButtonPressed(PSB_GREEN) << GREEN_BIT) | 
                   (my_controller.ButtonPressed(PSB_RED) << RED_BIT) | 
                   (my_controller.ButtonPressed(PSB_BLUE) << BLUE_BIT));
    //PrintArrayAsHex(data_out, N_DATA_BYTES);
    PROT_BuildPacket(data_out, N_DATA_BYTES, tx_packet, &tx_packet_length);
    //Serial.write(tx_packet, tx_packet_length);  // output this to UART pins
    PrintArrayAsHex(tx_packet, tx_packet_length);
  }
  
}

/*---------------------------Helper Function Definitions----------------------*/
void PrintArrayAsHex(uint8_t tx_packet[], uint8_t tx_packet_length) {
  Serial.print("\r\n");
  for (int i = 0; i < tx_packet_length; i++) {
    Serial.print("0x");
    Serial.println(tx_packet[i], HEX);
  }
}

void PrintInitializationFeedback(PS2X ps2x, uint8_t error) {
  if (error == 0) {
    Serial.println("Found Controller, configured successful");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
  } else if (error == 1) {
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
    while (1);
  } else if (error == 2) {
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  } else if (error == 3) {
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
    switch (ps2x.readType()) {
      case kControllerUnknown: Serial.println("Unknown Controller type"); break;
      case kControllerDualShock: Serial.println("DualShock Controller Found"); break;
      case kControllerGuitarHero: Serial.println("GuitarHero Controller Found"); break;
    }
  }
}

void PrintControllerValues(PS2X ps2x) {
  // print the controller type
  PrintControllerType(ps2x);
  
  // print the current data on each input
  if (ps2x.Button(PSB_START)) Serial.println("Start is being held");
  if (ps2x.Button(PSB_SELECT)) Serial.println("Select is being held");
  if (ps2x.Button(PSB_PAD_UP)) {         //will be TRUE as long as button is pressed
    Serial.print("Up held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
  }
  if (ps2x.Button(PSB_PAD_RIGHT)){
    Serial.print("Right held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
  }
  if (ps2x.Button(PSB_PAD_LEFT)){
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
  }
  if (ps2x.Button(PSB_PAD_DOWN)){
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
  }
  if (ps2x.NewButtonState()) {
    if (ps2x.Button(PSB_L3)) Serial.println("L3 pressed");
    if (ps2x.Button(PSB_R3)) Serial.println("R3 pressed");
    if (ps2x.Button(PSB_L2)) Serial.println("L2 pressed");
    if (ps2x.Button(PSB_R2)) Serial.println("R2 pressed");
    if (ps2x.Button(PSB_GREEN)) Serial.println("Triangle pressed");
  }
  if (ps2x.ButtonPressed(PSB_RED)) Serial.println("Circle just pressed");
  if (ps2x.ButtonReleased(PSB_PINK)) Serial.println("Square just released"); 
  if (ps2x.NewButtonState(PSB_BLUE)) Serial.println("X just changed");
  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC); 
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC); 
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC); 
  }
}

void PrintControllerType(PS2X ps2x) {
  Serial.print("\r\n\r\nController Type: ");
  switch (ps2x.readType()) {
    case kControllerGuitarHero: Serial.println("Guitar Hero Controller"); break;
    case kControllerDualShock: Serial.println("Unrecognized controller type"); break;
    default: Serial.println("Unrecognized"); break;
  }
}

