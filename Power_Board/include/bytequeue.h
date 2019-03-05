#ifndef BYTEQUEUE_N
#define BYTEQUEUE_N

#include "stddef.h"
#include "stdbool.h"
#include "stdint.h"

/// A ring queue used to buffer incoming and outgoing data
/// It is safe for data to be enqueued by one function and dequeued by an interrupt or vice versa.
typedef struct ByteQueue {
    /// physical buffer that the data will be stored in
    uint8_t *buffer;
    /// size of the buffer that data will be stored in.
    size_t capacity;
    /// number of bytes that have been enqueued over the lifetime of this queue, mod 2*capacity
    size_t n_enqueued;
    /// number of bytes that have been dequeued over the lifetime of this queue, mod 2*capacity
    size_t n_dequeued;
} ByteQueue;

/// A valid initialization value for an empty ByteQueue.
#define BYTE_QUEUE_NULL                                                                            \
    { 0 }
/// Resize the ByteQueue.
/// @returns true if the operation is successful.
///          false if not, in which case the byte queue will be set to @ref BYTE_QUEUE_NULL
bool bq_try_resize(volatile ByteQueue *bq, size_t new_capacity);
/// Add the given data to the ByteQueue if there is space available
bool bq_try_push(volatile ByteQueue *bq, size_t n_bytes, void *data_in);
/// Get data from the ByteQueue if available
bool bq_try_pop(volatile ByteQueue *bq, size_t n_bytes, void *data_out);
size_t bq_count(volatile ByteQueue *bq);
#endif