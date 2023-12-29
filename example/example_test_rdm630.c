#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico_rdm630_pio.h"

// Standard Task priority
#define MAIN_TASK_STACK_SIZE (1024 * 2)
static const char *MAIN_TASK_NAME = "MainThread";
#define MAIN_TASK_PRIORITY ( tskIDLE_PRIORITY + 1UL )

#define RDM_ONE_PIN (11)
#define RDM_TWO_PIN (12)
#define RDM_THREE_PIN (13)
#define RDM_FOUR_PIN (14)

rdm630_pio_t rmd_1;
rdm630_pio_t rmd_2;
rdm630_pio_t rmd_3;
rdm630_pio_t rmd_4;

static void new_tag_one_cb(void *rdm630_pio, uint32_t tag_id) {
    rdm630_pio_t *self = rdm630_pio;
    printf("new tag on reader one: %d\n", tag_id);
}

static void new_tag_two_cb(void *rdm630_pio, uint32_t tag_id) {
    rdm630_pio_t *self = rdm630_pio;
    printf("new tag on reader two: %d\n", tag_id);
}

static void new_tag_shared_cb(void *rdm630_pio, uint32_t tag_id) {
    rdm630_pio_t *self = rdm630_pio;
    printf("new tag on reader pin: %d, tag: %d\n", self->rx_pin, tag_id);
}


void main_task(void *params)
{
    printf("Main task: start\n");

    PIO pio = pio0;

    printf("setting up rdm 0\n");
    rdm630_pio_init(
        &rmd_1,
        pio,
        pio_claim_unused_sm(pio, true),
        RDM_ONE_PIN,
        new_tag_one_cb
    );
    printf("setting up rdm 1\n");
    rdm630_pio_init(
        &rmd_2,
        pio,
        pio_claim_unused_sm(pio, true),
        RDM_TWO_PIN,
        new_tag_two_cb
    );
    printf("setting up rdm 2\n");
    rdm630_pio_init(
        &rmd_3,
        pio,
        pio_claim_unused_sm(pio, true),
        RDM_THREE_PIN,
        new_tag_shared_cb
    );
    printf("setting up rdm 3\n");
    rdm630_pio_init(
        &rmd_4,
        pio,
        pio_claim_unused_sm(pio, true),
        RDM_FOUR_PIN,
        new_tag_shared_cb
    );

    printf("Main task: Entering loop\n");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vLaunch()
{
    TaskHandle_t task;

    xTaskCreate(main_task, MAIN_TASK_NAME, MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &task);

    // more tasks, they can set vTaskCoreAffinitySet if needed

    // Start the tasks and timer running.
    vTaskStartScheduler();
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("GO\n");

    vLaunch();

    return 0;
}
