
#ifndef _PICO_RDM630_PIO_H
#define _PICO_RDM630_PIO_H

#include <stdint.h>

#include "pico/async_context_freertos.h"
#include "pico/util/queue.h"
#include "hardware/pio.h"
#include "hardware/irq.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RDM630_BAUDRATE                (9600)
#define RDM630_PACKET_SIZE             (14)
#define RDM630_PACKET_BEGIN            (0x02)
#define RDM630_PACKET_END              (0x03)
#define RDM630_DEFAULT_TAG_TIMEOUT_MS  (300)
#define RDM630_READ_TIMEOUT            (20)
#define RMD630_FIFO_SIZE               (RDM630_PACKET_SIZE * 3)

// can't use this due to forward delectation issue ??
// typedef void (*rdm630_pio_callback_t)(rdm630_pio_t *rdm630_pio, uint32_t tag_id);
typedef void (*rdm630_pio_callback_t)(void *rdm630_pio, uint32_t tag_id);

typedef struct {
    PIO pio;
    int pio_sm;
    int rx_pin;
    rdm630_pio_callback_t callback;

    unsigned int pio_offset;
    int8_t pio_irq;
    irq_handler_t pio_irq_func;
    queue_t fifo;
    async_when_pending_worker_t irq_worker;
    async_at_time_worker_t poll_worker;

    char buffer[RDM630_PACKET_SIZE];
    size_t chars_received;

    uint32_t _current_tag_id;
    uint32_t _tag_id;
    uint32_t _new_tag_id;
    uint32_t _last_tag_id;
    uint32_t _last_tag_ms;
    uint32_t _tag_timeout_ms;
} rdm630_pio_t;


bool rdm630_pio_init(rdm630_pio_t *rdm630_pio, PIO pio, int sm, int rx_pin, rdm630_pio_callback_t new_tag_callback);

bool rdm630_pio_deinit(rdm630_pio_t *rdm630_pio);

void rdm630_pio_set_tag_timeout(rdm630_pio_t *rdm630_pio, uint32_t tag_timeout_ms);

uint32_t rdm630_pio_get_tag_id(rdm630_pio_t *rdm630_pio);
uint32_t rdm630_pio_get_new_tag_id(rdm630_pio_t *rdm630_pio);

#ifdef __cplusplus
}
#endif

#endif
