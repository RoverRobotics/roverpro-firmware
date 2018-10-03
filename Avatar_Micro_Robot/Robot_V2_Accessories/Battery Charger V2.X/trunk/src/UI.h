// File: UI.h
//
// Description: This module manages the user interface (UI) of the battery
//   charger.
//
// Notes:
//   - assumes timers are already initialized
//   - consumes up to the first three timers (0, 1 and 2) of the timers library
//   - additionally provides a heartbeat debugging output on pin 1
//   - default state is kUIStateWaiting
//
// Responsible Engineer(s): Stellios Leventis (sleventis@robotex.com)
#ifndef USERINTERFACE_H
#define USERINTERFACE_H
//---------------------------Type Definitions-----------------------------------
// supported UI states
typedef enum {
  kUIStateWaiting = 0,
	kUIStateCharging,
  kUIStateDoneCharging,
  kUIStateErring
} kUIState;

//---------------------------Public Functions-----------------------------------
// Function: UI_Init
void UI_Init(void);

// Function: UI_Run
// Description: Place this in the main loop.  It manages all UI-related 
//   processes.
void UI_Run(void);


// Function: UI_set_state
// Parameters:
//   const kUIState state,  the desired state to which the UI should be updated
void UI_set_state(const kUIState state);

// Function: UI_state
// Description: Returns the current state of the UI
kUIState UI_state(void);

void UI_Deinit(void);

#endif
