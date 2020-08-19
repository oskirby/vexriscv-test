#include <stdint.h>

#define LED_PWM_BASE    (volatile uint32_t *)0x2000000
#define LED_PWM_COUNT   4

int main(void)
{
    volatile uint32_t *ledpwm = LED_PWM_BASE;
    static uint8_t val = 0xff;
    int i;

    for (i = 0; i < LED_PWM_COUNT; i++) {
        ledpwm[i] = val;
        val >>= 2;
    }

    /* And finally - do nothing */
    while (1) {
        /* nop */
    }
}
