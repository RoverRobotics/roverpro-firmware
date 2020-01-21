#include "bytequeue.h"
#include "stdhdr.h"
#include "stdlib.h"

bool bq_try_resize(ByteQueue *self, size_t new_capacity) {
    ByteQueue new_q = BYTE_QUEUE_NULL;
    if (new_capacity != 0) {
        new_q.buffer = calloc(new_capacity, 1);
        if (new_q.buffer == 0) {
            return false;
        }
    }
    new_q.capacity = new_capacity;
    free((void *)self->buffer);
    *self = new_q;
    return true;
}

bool bq_try_push(ByteQueue *self, size_t n_data, void *data) {
    if (!bq_can_push(self, n_data))
        return false;

    size_t capacity = self->capacity;
    size_t n_enqueued = self->n_enqueued;
    size_t i;
    for (i = 0; i < n_data; i++) {
        ((uint8_t *)self->buffer)[(n_enqueued + i) % capacity] = ((uint8_t *)data)[i];
    }

    self->n_enqueued = (n_enqueued + n_data) % (2 * capacity);
    return true;
}

bool bq_try_pop(ByteQueue *self, size_t n_data, void *data_out) {
    if (!bq_can_pop(self, n_data))
        return false;

    size_t capacity = self->capacity;
    size_t n_dequeued = self->n_dequeued;
    size_t i;
    for (i = 0; i < n_data; i++) {
        ((uint8_t *)data_out)[i] = ((uint8_t *)self->buffer)[(n_dequeued + i) % capacity];
        ((uint8_t *)self->buffer)[(n_dequeued + i) % capacity] = 0;
    }

    self->n_dequeued = (n_dequeued + n_data) % (2 * capacity);
    return true;
}

bool bq_can_push(ByteQueue *self, size_t n) {
    BREAKPOINT_IF(self->capacity < n);
    size_t count =
        (self->n_enqueued - self->n_dequeued + 2 * self->capacity) % (2 * self->capacity);
    return n + count <= self->capacity;
}

bool bq_can_pop(ByteQueue *self, size_t n) {
    BREAKPOINT_IF(self->capacity < n);
    size_t count =
        (self->n_enqueued - self->n_dequeued + 2 * self->capacity) % (2 * self->capacity);
    return n <= count;
}
