/// @file
/// A FIFO data structure for arbitrary binary data.
/// Designed for single-producer, single-consumer use.

#ifndef BYTEQUEUE_N
#define BYTEQUEUE_N

#include "stddef.h"
#include "stdbool.h"
#include "stdint.h"

/// A ring queue used to buffer incoming and outgoing data
/// It is safe for data to be enqueued by one function and dequeued by an interrupt or vice versa.
typedef struct ByteQueue {
    /// physical buffer that the data will be stored in
    volatile void *buffer;
    /// allocated size of the buffer that data will be stored in
    size_t capacity;
    /// number of bytes that have been enqueued over the lifetime of this queue, mod 2*capacity
    volatile size_t n_enqueued;
    /// number of bytes that have been dequeued over the lifetime of this queue, mod 2*capacity
    volatile size_t n_dequeued;
} ByteQueue;

/// A valid initialization value for an empty ByteQueue.
#define BYTE_QUEUE_NULL {0}
    
/// Clear the contents and resize the ByteQueue. A new, empty buffer will be created on the heap.
/// @returns true if the operation is successful.
///          false if not, in which case the byte queue will be set to @ref BYTE_QUEUE_NULL
bool bq_try_resize(ByteQueue *self, size_t new_capacity);

/// If the ByteQueue has at least n bytes of free space, append the data from data_in.
/// Otherwise, the ByteQueue is unmodified.
/// It is safe to call bq_try_push and bq_try_pop concurrently.
/// @param self the ByteQueue to operate on
/// @param n the number of bytes to write.
/// @param data_in where the new data will be read from.
/// @return true if the operation is successful.
///         false if not enough bytes were present.
bool bq_try_push(ByteQueue *self, size_t n, void *data_in);

/// If the ByteQueue has at least n bytes of data, remove them from the ByteQueue and write them to data_out.
/// Otherwise, the ByteQueue and data_out is unmodified.
/// It is safe to call bq_try_push and bq_try_pop concurrently.
/// @param self the ByteQueue to operate on
/// @param n the number of bytes to retrieve
/// @param data_out where resulting data will be written. If unsuccessful, contents will be unchanged.
/// @return true if the data was successfully retrieved into data_out.
///         false if not enough bytes were present.
bool bq_try_pop(ByteQueue *self, size_t n, void *data_out);

bool bq_can_push(ByteQueue *self, size_t n);
bool bq_can_pop(ByteQueue *self, size_t n);

#endif