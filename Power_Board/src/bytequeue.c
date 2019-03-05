#include "bytequeue.h"
#include "stdhdr.h"
#include "string.h"

#define min(a, b) (((a) < (b)) ? (a) : (b))

bool bq_try_resize(volatile ByteQueue *self, size_t new_capacity) {
    ByteQueue new_q = BYTE_QUEUE_NULL;
    if (new_capacity == 0) {
        free(self->buffer);
    } else {
        new_q.buffer = realloc(self->buffer, new_capacity);
        // uh oh - this would mean we're out of heap!
        BREAKPOINT_IF(new_q.buffer == 0);
        memset(new_q.buffer, 0, new_capacity);
    }
    new_q.capacity = new_q.buffer == 0 ? 0 : new_capacity;

    *self = new_q;
    return new_capacity == 0 || new_q.buffer != 0;
}

bool bq_try_push(volatile ByteQueue *self, size_t n_data, void *data) {
    size_t capacity = self->capacity;
    size_t n_enqueued = self->n_enqueued;
    void *buffer = self->buffer;

    BREAKPOINT_IF(n_data > capacity);

    if (bq_count(self) + n_data > capacity) {
        return false;
    }

    size_t i_dest1 = n_enqueued % capacity;
    size_t ncpy1 = min(n_data, capacity - i_dest1);
    size_t ncpy2 = n_data - ncpy1;

    memcpy(buffer + i_dest1, data, ncpy1);
    memcpy(buffer, data + ncpy1, ncpy2);

    self->n_enqueued = (n_enqueued + n_data)%(capacity*2);
    return true;
}

size_t bq_count(volatile ByteQueue *self) {
    size_t capacity = self->capacity;
    size_t n_enqueued = self->n_enqueued;
    size_t n_dequeued = self->n_dequeued;
    return (n_enqueued - n_dequeued + 2 * capacity) % (2 * capacity);
}

bool bq_try_pop(volatile ByteQueue *self, size_t n_data, void *data_out) {
    size_t n_dequeued = self->n_dequeued;
    void *buffer = self->buffer;
    size_t capacity = self->capacity;
    BREAKPOINT_IF(n_data > capacity);

    if (bq_count(self) < n_data) {
        return false;
    }
	
    size_t i_src1 = n_dequeued % capacity;
    size_t n_cpy1 = min(n_data, capacity - i_src1);
    size_t n_cpy2 = n_data - n_cpy1;

    memcpy(data_out, buffer + i_src1, n_cpy1);
    memset(buffer + i_src1, 0, n_cpy1);
    memcpy(data_out + i_src1, buffer, n_cpy2);
    memset(buffer, 0, n_cpy2);
    
    self->n_dequeued = (n_dequeued + n_data) % (capacity*2);
    return true;
}
