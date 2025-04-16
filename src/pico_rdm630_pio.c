
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/async_context_freertos.h"
#include "pico/util/queue.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/dma.h"

#include "pico_rdm630_pio.pio.h"

#include "pico_rdm630_pio.h"

int32_t _rdm630_shared_pio_program_offset[NUM_PIOS] = {-1, -1};
int8_t _rdm630_shared_pio_irq[NUM_PIOS] = {-1, -1};
static async_context_freertos_t _rdm630_share_async_context;
bool _rdm630_share_async_context_initalized = false;
bool _rdm630_shared_dma_handleder_initalized = false;

rdm630_pio_t *_rdm630_pio_instances[NUM_PIOS][NUM_PIO_STATE_MACHINES];

void _rmd630_dma_complete_handler(void) {
    for (int pio_count = 0; pio_count < NUM_PIOS; ++pio_count) {
        for (int sm_count = 0; sm_count < NUM_PIO_STATE_MACHINES; ++sm_count) {
            if (_rdm630_pio_instances[pio_count][sm_count] == NULL) {
                continue;
            }

            rdm630_pio_t *self = _rdm630_pio_instances[pio_count][sm_count];
            if (dma_irqn_get_channel_status(CONFIG_RDM630_DMA_IRQ_TO_USE, self->dma_ch)) {
                dma_irqn_acknowledge_channel(CONFIG_RDM630_DMA_IRQ_TO_USE, self->dma_ch);

                // restart dma
                dma_channel_set_write_addr(self->dma_ch, &self->buffer, true);

                // work the buffer on the thread
                async_context_set_work_pending(&_rdm630_share_async_context.core, &self->dma_worker);
            }
        }
    }
}

uint32_t _rdm630_dma_init(void *param) {
    rdm630_pio_t *self = (rdm630_pio_t *) param;
#ifndef NDEBUG
    printf("_rdm630_dma_init: %d\n", self->rx_pin);
#endif
    self->dma_ch = dma_claim_unused_channel(true);

    dma_channel_config channel_config = dma_channel_get_default_config(self->dma_ch); /* get default configuration */
    channel_config_set_dreq(&channel_config, pio_get_dreq(self->pio, self->pio_sm, false)); /* configure data request. false: receiving data from the PIO state machine */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_8); /* data transfer size is 8 bits */
    channel_config_set_read_increment(&channel_config, false); /* each read of the data will NOT increase the read pointer */
    channel_config_set_write_increment(&channel_config, true); /* each read of the data will increase the write pointer */

    dma_channel_configure(self->dma_ch,
        &channel_config,
        &self->buffer, /* write address: write to instance buffer */
        (io_rw_8*) &self->pio->rxf[self->pio_sm] + 3, /* don't provide a read address yet */
        RDM630_PACKET_SIZE, /* number of transfers */
        false); /* don't start yet */

    if (! _rdm630_shared_dma_handleder_initalized) {
        irq_add_shared_handler(CONFIG_RDM630_DMA_IRQ_TO_USE, &_rmd630_dma_complete_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY); /* after DMA all data, raise an interrupt */
        _rdm630_shared_dma_handleder_initalized = true;
    }

    dma_irqn_set_channel_enabled(CONFIG_RDM630_DMA_IRQ_TO_USE, self->dma_ch, true); /* map DMA channel to interrupt */
    irq_set_enabled(CONFIG_RDM630_DMA_IRQ_TO_USE, true); /* enable interrupt */

    // start the dma
    dma_channel_set_write_addr(self->dma_ch, &self->buffer, true);

    return 0;
}

static void _rdm630_async_dma_worker_func(async_context_t *async_context, async_when_pending_worker_t *worker) {
    rdm630_pio_t *self = worker->user_data;
    #ifndef NDEBUG
    printf("_rdm630_async_dma_worker_func: %d\n", self->rx_pin);
    #endif

    if (RDM630_PACKET_BEGIN != self->buffer[0]
        || RDM630_PACKET_END != self->buffer[13]) {
        printf("_rdm630_async_dma_worker_func: %d, reset DMA\n", self->rx_pin);
        // reset dma read in hope of fixing alignment
        // Tag is resent every ~50ms, ao good chance we are running in a silent gap

        dma_irqn_set_channel_enabled(CONFIG_RDM630_DMA_IRQ_TO_USE, self->dma_ch, false);
        dma_channel_abort(self->dma_ch);
        dma_irqn_acknowledge_channel(CONFIG_RDM630_DMA_IRQ_TO_USE, self->dma_ch);
        dma_irqn_set_channel_enabled(CONFIG_RDM630_DMA_IRQ_TO_USE, self->dma_ch, true);

        // restart the dma
        dma_channel_set_write_addr(self->dma_ch, &self->buffer, true);

        return;
    }

    uint8_t checksum;
    uint32_t tag_id;

    /* add null and parse checksum */
    self->buffer[13] = 0;
    checksum = strtol(self->buffer + 11, NULL, 16);
    /* add null and parse tag_id */
    self->buffer[11] = 0;
    tag_id = strtol(self->buffer + 3, NULL, 16);
    /* add null and parse version (needs to be xored with checksum) */
    self->buffer[3] = 0;
    checksum ^= strtol(self->buffer + 1, NULL, 16);

    /* xore the tag_id and validate checksum */
    for (uint8_t i = 0; i < 32; i += 8) {
        checksum ^= ((tag_id >> i) & 0xFF);
    }

    if (checksum) {
        // bad
        tag_id = 0;
    }

    self->_current_tag_id = tag_id;
}

uint32_t _rdm630_irq_init(void *param) {
    rdm630_pio_t *self = (rdm630_pio_t *) param;
#ifndef NDEBUG
    assert(get_core_num() == async_context_core_num(&_rdm630_share_async_context.core));
#endif
    irq_add_shared_handler(self->pio_irq, self->pio_irq_func, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY); // Add a shared IRQ handler
    irq_set_enabled(self->pio_irq, true); // Enable the IRQ
    const unsigned int irq_index = self->pio_irq - ((self->pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0); // Get index of the IRQ
    pio_set_irqn_source_enabled(self->pio, irq_index, pis_sm0_rx_fifo_not_empty + self->pio_sm, true); // Set pio to tell us when the FIFO is NOT empty

    return 0;
}

static void _rdm630_shared_pio_irq_func(rdm630_pio_t *self) {
    while(!pio_sm_is_rx_fifo_empty(self->pio, self->pio_sm)) {
        char c = uart_rx_program_getc(self->pio, self->pio_sm);
        if (!queue_try_add(&self->fifo, &c)) {
            panic("fifo full, rx_pin: %d", self->rx_pin);
        }
    }

    // Tell the async worker that there are some characters waiting for us
    if (queue_get_level(&self->fifo) >= RDM630_PACKET_SIZE) {
        async_context_set_work_pending(&_rdm630_share_async_context.core, &self->irq_worker);
    }
}

// little messy but could not come up with a better way in a hurry
static void _rdm630_shared_pio_irq_func_pio0_sm0(void) {
    _rdm630_shared_pio_irq_func(_rdm630_pio_instances[0][0]);
}

static void _rdm630_shared_pio_irq_func_pio0_sm1(void) {
    _rdm630_shared_pio_irq_func(_rdm630_pio_instances[0][1]);
}

static void _rdm630_shared_pio_irq_func_pio0_sm2(void) {
    _rdm630_shared_pio_irq_func(_rdm630_pio_instances[0][2]);
}

static void _rdm630_shared_pio_irq_func_pio0_sm3(void) {
    _rdm630_shared_pio_irq_func(_rdm630_pio_instances[0][3]);
}

static void _rdm630_shared_pio_irq_func_pio1_sm0(void) {
    _rdm630_shared_pio_irq_func(_rdm630_pio_instances[1][0]);
}

static void _rdm630_shared_pio_irq_func_pio1_sm1(void) {
    _rdm630_shared_pio_irq_func(_rdm630_pio_instances[1][1]);
}

static void _rdm630_shared_pio_irq_func_pio1_sm2(void) {
    _rdm630_shared_pio_irq_func(_rdm630_pio_instances[1][2]);
}

static void _rdm630_shared_pio_irq_func_pio1_sm3(void) {
    _rdm630_shared_pio_irq_func(_rdm630_pio_instances[1][3]);
}

static void _rdm630_async_irq_worker_func(async_context_t *async_context, async_when_pending_worker_t *worker) {
    rdm630_pio_t *self = worker->user_data;
    #ifndef NDEBUG
    printf("_rdm630_async_irq_worker_func: %d\n", self->rx_pin);
    #endif

    while(! queue_is_empty(&self->fifo)) {
        char c;
        if (!queue_try_remove(&self->fifo, &c)) {
            panic("fifo empty, rx_pin: %d", self->rx_pin);
        }
        if (self->chars_received == RDM630_PACKET_SIZE) {
            self->chars_received = 0;
        }

        // looking for the start of a packet
        if (RDM630_PACKET_BEGIN != c && self->chars_received == 0) {
            continue;
        } else if (RDM630_PACKET_BEGIN == c) {
            self->chars_received = 0;
        } else if (RDM630_PACKET_END == c && self->chars_received != (RDM630_PACKET_SIZE - 1)) {
            // got end before expecting it
            self->chars_received = 0;
            continue;
        }

        self->buffer[self->chars_received++] = c;

        if (self->chars_received != RDM630_PACKET_SIZE) {
            // don't yet have a full packet
            continue;
        }

        self->chars_received = 0;

        uint8_t checksum;
        uint32_t tag_id;

        /* add null and parse checksum */
        self->buffer[13] = 0;
        checksum = strtol(self->buffer + 11, NULL, 16);
        /* add null and parse tag_id */
        self->buffer[11] = 0;
        tag_id = strtol(self->buffer + 3, NULL, 16);
        /* add null and parse version (needs to be xored with checksum) */
        self->buffer[3] = 0;
        checksum ^= strtol(self->buffer + 1, NULL, 16);

        /* xore the tag_id and validate checksum */
        for (uint8_t i = 0; i < 32; i += 8) {
            checksum ^= ((tag_id >> i) & 0xFF);
        }

        if (checksum) {
            // bad
            tag_id = 0;
        }

        self->_current_tag_id = tag_id;
    }
}

void _rdm630_update(rdm630_pio_t *self) {
    #ifndef NDEBUG
    self->xLastWakeTime = xTaskGetTickCount();
    if (self->xLastWakeTime - self->xLastWatchdog > pdMS_TO_TICKS(1000)) {
        self->xLastWatchdog = self->xLastWakeTime;
        printf("Watchdog: RDM630 %d\n", self->rx_pin);
    }
    #endif

    uint32_t cur_ms = to_ms_since_boot(get_absolute_time());
    uint32_t tag_id = self->_current_tag_id;
    self->_current_tag_id = 0;

    /* if a new tag appears- return it */
    if (tag_id) {
        self->_tag_id = tag_id;
        if (self->_last_tag_id != tag_id) {
            self->_last_tag_id = tag_id;
            self->_new_tag_id = tag_id;
            if (self->callback) {
                self->callback(self, tag_id);
            }
        }
        self->_last_tag_ms = cur_ms;

        return;
    }

    /* if the old tag id is still valid- levee it */
    if (! self->_tag_id || (cur_ms - self->_last_tag_ms < self->_tag_timeout_ms)) {
        return;
    }

    self->_tag_id = self->_last_tag_id = self->_new_tag_id = 0;
    if (self->callback) {
        self->callback(self, self->_new_tag_id);
    }
}

void _rdm630_async_poll_worker_func(async_context_t *context, struct async_work_on_timeout *timeout) {
    _rdm630_update((rdm630_pio_t *) timeout->user_data);

    async_context_add_at_time_worker_in_ms(&_rdm630_share_async_context.core, timeout, RDM630_READ_TIMEOUT / 2);
}

bool rdm630_pio_init(rdm630_pio_t *rdm630_pio, PIO pio, int sm, int rx_pin, rdm630_pio_callback_t new_tag_callback) {
    rdm630_pio_t *self = rdm630_pio;
    self->pio = pio;
    self->pio_sm = sm;
    self->rx_pin = rx_pin;
    self->callback = new_tag_callback;
    self->_tag_timeout_ms = RDM630_DEFAULT_TAG_TIMEOUT_MS;
    self->_current_tag_id = 0;

    _rdm630_pio_instances[pio_get_index(self->pio)][self->pio_sm] = self;

    #if CONFIG_RDM630_USE_DMA != 1
    queue_init(&self->fifo, 1, RDM630_FIFO_SIZE);
    #endif

    if (! _rdm630_share_async_context_initalized) {
        async_context_freertos_config_t config = async_context_freertos_default_config();
        #ifdef CONFIG_RDM630_TASK_PRIORITY
            config.task_priority = CONFIG_RDM630_TASK_PRIORITY;
        #endif
        #ifdef CONFIG_RDM630_TASK_STACK_SIZE
            config.task_stack_size = CONFIG_RDM630_TASK_STACK_SIZE;
        #endif

        if (! async_context_freertos_init(&_rdm630_share_async_context, &config)) {
            panic("failed to setup context");
        }

        _rdm630_share_async_context_initalized = true;
    }

    #if CONFIG_RDM630_USE_DMA
    self->dma_worker.do_work = _rdm630_async_dma_worker_func;
    self->dma_worker.user_data = rdm630_pio;
    async_context_add_when_pending_worker(&_rdm630_share_async_context.core, &self->dma_worker);
    #else
    self->irq_worker.do_work = _rdm630_async_irq_worker_func;
    self->irq_worker.user_data = rdm630_pio;
    async_context_add_when_pending_worker(&_rdm630_share_async_context.core, &self->irq_worker);
    #endif
    self->poll_worker.do_work = _rdm630_async_poll_worker_func;
    self->poll_worker.user_data = rdm630_pio;
    async_context_add_at_time_worker_in_ms(&_rdm630_share_async_context.core, &self->poll_worker, RDM630_READ_TIMEOUT);

    // add program
    if (-1 == _rdm630_shared_pio_program_offset[pio_get_index(self->pio)]) {
        if (! pio_can_add_program(self->pio, &uart_rx_program)) {
            panic("failed to add program to pio");
        }

        _rdm630_shared_pio_program_offset[pio_get_index(self->pio)] = pio_add_program(self->pio, &uart_rx_program);
    }
    self->pio_offset = _rdm630_shared_pio_program_offset[pio_get_index(self->pio)];

    uart_rx_program_init(self->pio, self->pio_sm, self->pio_offset, self->rx_pin, RDM630_BAUDRATE);

    #if CONFIG_RDM630_USE_DMA
    // Enable DMA
    _rdm630_dma_init(self);
    #else
    // Find a free irq
    if (-1 == _rdm630_shared_pio_irq[pio_get_index(self->pio)]) {
        // irq not yet initialized
        int8_t pio_irq = (self->pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;

        if (irq_get_exclusive_handler(pio_irq)) {
            pio_irq++;
            if (irq_get_exclusive_handler(pio_irq)) {
                panic("All IRQs are in use");
            }
        }

        _rdm630_shared_pio_irq[pio_get_index(self->pio)] = pio_irq;
    }
    self->pio_irq = _rdm630_shared_pio_irq[pio_get_index(self->pio)];

    // Work out which interrupt handler we need
    if (self->pio == pio0) {
        switch (self->pio_sm) {
        case 0:
            self->pio_irq_func = _rdm630_shared_pio_irq_func_pio0_sm0;
            break;
        case 1:
            self->pio_irq_func = _rdm630_shared_pio_irq_func_pio0_sm1;
            break;
        case 2:
            self->pio_irq_func = _rdm630_shared_pio_irq_func_pio0_sm2;
            break;
        case 3:
            self->pio_irq_func = _rdm630_shared_pio_irq_func_pio0_sm3;
            break;
        }
    } else {
        switch (self->pio_sm) {
        case 0:
            self->pio_irq_func = _rdm630_shared_pio_irq_func_pio1_sm0;
            break;
        case 1:
            self->pio_irq_func = _rdm630_shared_pio_irq_func_pio1_sm1;
            break;
        case 2:
            self->pio_irq_func = _rdm630_shared_pio_irq_func_pio1_sm2;
            break;
        case 3:
            self->pio_irq_func = _rdm630_shared_pio_irq_func_pio1_sm3;
            break;
        }
    }

    // Enable interrupt
    async_context_execute_sync(&_rdm630_share_async_context.core, _rdm630_irq_init, self);
    #endif


    return true;
}

// untested
bool rdm630_pio_deinit(rdm630_pio_t *rdm630_pio) {
    rdm630_pio_t *self = rdm630_pio;

    #if CONFIG_RDM630_USE_DMA
    dma_irqn_set_channel_enabled(CONFIG_RDM630_DMA_IRQ_TO_USE, self->dma_ch, false);
    dma_channel_abort(self->dma_ch);
    dma_irqn_acknowledge_channel(CONFIG_RDM630_DMA_IRQ_TO_USE, self->dma_ch);
    #else
    const unsigned int irq_index = self->pio_irq - ((self->pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0); // Get index of the IRQ
    pio_set_irqn_source_enabled(self->pio, irq_index, pis_sm0_rx_fifo_not_empty + self->pio_sm, false);
    irq_set_enabled(self->pio_irq, false);
    irq_remove_handler(self->pio_irq, self->pio_irq_func);
    #endif

    // Cleanup pio
    pio_sm_set_enabled(self->pio, self->pio_sm, false);
    // pio_remove_program(pio, &uart_rx_program, offset);
    pio_sm_unclaim(self->pio, self->pio_sm);

    #if CONFIG_RDM630_USE_DMA
    async_context_remove_when_pending_worker(&_rdm630_share_async_context.core, &self->dma_worker);
    #else
    async_context_remove_when_pending_worker(&_rdm630_share_async_context.core, &self->irq_worker);
    #endif
    // async_context_deinit(&async_context.core);
    #if CONFIG_RDM630_USE_DMA != 1
    queue_free(&self->fifo);
    #endif

    _rdm630_pio_instances[pio_get_index(self->pio)][self->pio_sm] = NULL;
}

void set_tag_timeout(rdm630_pio_t *rdm630_pio, uint32_t tag_timeout_ms) {
    rdm630_pio->_tag_timeout_ms = tag_timeout_ms;
}

uint32_t rdm630_pio_get_tag_id(rdm630_pio_t *rdm630_pio) {
    return rdm630_pio->_tag_id;
}

uint32_t rdm630_pio_get_new_tag_id(rdm630_pio_t *rdm630_pio) {
    uint32_t tag_id = rdm630_pio->_new_tag_id;
    rdm630_pio->_new_tag_id = 0;

    return tag_id;
}
