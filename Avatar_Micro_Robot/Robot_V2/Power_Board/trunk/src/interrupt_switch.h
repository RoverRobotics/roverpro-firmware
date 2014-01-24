/**
 * @file interrupt_switch.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex carrier PIC firmware.
 *
 */


void InterruptDummyFunction();

extern void (*T1InterruptUserFunction)(void);
extern void (*T2InterruptUserFunction)(void);
extern void (*T3InterruptUserFunction)(void);
extern void (*T4InterruptUserFunction)(void);
extern void (*T5InterruptUserFunction)(void);
extern void (*IC1InterruptUserFunction)(void);
extern void (*IC2InterruptUserFunction)(void);
extern void (*IC3InterruptUserFunction)(void);
extern void (*IC4InterruptUserFunction)(void);
extern void (*IC5InterruptUserFunction)(void);
extern void (*IC6InterruptUserFunction)(void);
extern void (*ADC1InterruptUserFunction)(void);
extern void (*U1TXInterruptUserFunction)(void);
extern void (*U1RXInterruptUserFunction)(void);
