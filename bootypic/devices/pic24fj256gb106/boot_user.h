#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef __PIC24FJ256GB106__
#error "platform settings do not match header file"
#endif
#define PLATFORM_STRING "pic24fj256gb106"

// UART communication baud rate, in Hz
#define UART_BAUD_RATE 57600

// remappable pin for UART input
#define RX_PIN 6
// remappable pin for UART output
#define TX_PIN 7

/**
 * @brief How long may the bootloader idle before starting, in milliseconds,
 *    under normal start conditions?
 * Long enough to provide a window of opportunity for software.
 */
#define BOOTLOAD_SHORT_TIMEOUT_MS 1000

/**
 * @brief How long may the bootloader idle before starting, in milliseconds?
 * active at startup before moving on to the application if a reset was requested.
 * Long enough for the user to start manual bootload
 */
#define BOOTLOAD_LONG_TIMEOUT_MS 10000

/// Where to jump when bootloader is done
#define APPLICATION_START_ADDRESS 0x2000

/// Instruction clock frequency, in HZ
#define FCY (16000000UL)

/// instructions per erase
#define _FLASH_PAGE 512
/// instructions per row write
#define _FLASH_ROW 64

/** @brief this is the maximum size that can be programmed into the microcontroller
 * as part of one transaction using the CMD_WRITE_MAX_PROG_SIZE command
 *
 * A value of 0x80 should work on all microcontrollers.  Larger values will
 * allow faster programming operations, but will consume more RAM.
 */
#define MAX_PROG_SIZE (2 * _FLASH_ROW)

/**
 * @brief run the very first initialization
 */
void pre_bootload_hook();

/**
 * @brief hook to run when the app is requested to start
 *
 * This function should either return, handing control back to the bootloader, or end the bootloader
 * and start the firmware.
 */
void try_start_app_hook();

/**
 * @brief hook to run inside bootload main loop
 *
 * This function should run any loop checks and return if the bootload should continue to run.
 */
void bootload_loop_hook();

/**
 * @brief reads instructions from the given address
 *
 * @param words buffer to put the data into
 * @param start_address first address to read
 * @param n_words how many instructions to read
 */
void read_words(uint32_t *words, uint32_t start_address, unsigned n_words);

/**
 * @brief erases the flash page starting at the address
 *
 * @param address
 */
void erase_page(uint32_t address);

/**
 * @brief write instructions to the given address, using the most appropriate NVM operations
 * available
 *
 * @param words buffer containing the data to write
 * @param start_address first address to write
 * @param n_words how many instructions to write
 */
void write_words(const uint32_t *words, uint32_t start_address, unsigned n_words);
