/**
 * Lock-Free Single-Producer Single-Consumer (SPSC) RAM FIFO
 * (C) Juan Schiavoni 2021-2026
 *
 * Implements an interrupt-safe and thread-safe circular buffer without 
 * requiring hardware spinlocks or interrupt disabling.
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "ram_fifo.h"

static uint32_t *capture_buf = NULL;
static volatile size_t capture_head = 0; // Written ONLY by Producer (ISR / Core 0)
static volatile size_t capture_tail = 0; // Written ONLY by Consumer (Core 1 / Main Loop)
static size_t capture_size = 0;

/**
 * @brief Initializes the RAM FIFO buffer.
 * @param size Desired capacity (elements count).
 * @return true if memory allocation succeeded, false otherwise.
 */
bool ram_fifo_init(size_t size) {
    // Allocate size + 1 to distinguish between Full and Empty states
    capture_buf = malloc((size + 1) * sizeof(uint32_t));
    capture_size = size + 1;
    capture_head = 0;
    capture_tail = 0;

    return (capture_buf != NULL);
}

/**
 * @brief Checks if the FIFO buffer is empty.
 * @return true if empty, false otherwise.
 */
bool ram_fifo_is_empty(void) {
    return (capture_head == capture_tail);
}

/**
 * @brief Inserts an item into the FIFO buffer (Producer context).
 * @param item 32-bit data word to enqueue.
 * @return true on success, false if the buffer is full.
 */
bool ram_fifo_set(uint32_t item) {
    size_t next_head = (capture_head + 1) % capture_size;

    // If next_head reaches tail, the FIFO is full
    if (next_head == capture_tail) {
        return false; // Retain current pointers intact on overflow
    }

    capture_buf[capture_head] = item;
    capture_head = next_head; // Atomic single-word write operation
    return true;
}

/**
 * @brief Retrieves and removes an item from the FIFO buffer (Consumer context).
 * @return Dequeued 32-bit data word, or 0 if the FIFO is empty.
 */
uint32_t ram_fifo_get(void) {
    // Strict underflow prevention
    if (ram_fifo_is_empty()) {
        return 0; 
    }

    uint32_t item = capture_buf[capture_tail];
    capture_tail = (capture_tail + 1) % capture_size; // Atomic single-word write operation
    return item;
}

/**
 * @brief Calculates current number of items stored in the FIFO buffer.
 * @return Number of elements currently enqueued.
 */
uint32_t ram_fifo_get_level(void) {
    size_t head = capture_head;
    size_t tail = capture_tail;
    
    if (head >= tail) {
        return head - tail;
    }
    return capture_size + head - tail;
}
