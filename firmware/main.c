#include <stdint.h>
#include <printf.h>

#define LED_PWM_BASE    (volatile uint32_t *)0x20000000
#define LED_PWM_COUNT   4

// NS16650 Serial Interface
typedef uint32_t serial_reg_t;
struct serial_regmap {
    union {
        serial_reg_t thr;   // Transmit Holding Register.
        serial_reg_t rhr;   // Receive Holding Register.
        serial_reg_t dll;   // Baudrate Divisor LSB 
    };
    union {
        serial_reg_t ier;   // Interrupt Enable Register.
        serial_reg_t dlm;   // Baudrate Divisor MSB
    };
    union {
        serial_reg_t isr;   // Interrupt Status Register.
        serial_reg_t fcr;   // FIFO Configuration Register.
        serial_reg_t pld;   // Prescaler Divider.
    };
    serial_reg_t    lcr;    // Line Control Register.
    serial_reg_t    mcr;    // Modem Control Register.
    serial_reg_t    lsr;    // Line Status Register.
    serial_reg_t    msr;    // Modem Status Register.
    serial_reg_t    scratch; // Scratch Value Register.
} __attribute__((packed));
#define SERIAL ((volatile struct serial_regmap *)0x40000000)

static void serial_putc(int ch)
{
    while ((SERIAL->isr & 0x02) == 0) { /* nop */}
    SERIAL->thr = ch;
}

void _putchar(char ch)
{
    while ((SERIAL->isr & 0x02) == 0) { /* nop */}
    SERIAL->thr = ch;
}

struct spi_regmap {
    uint32_t control;
    uint32_t status;
    uint32_t transmit;
    uint32_t receive;
};

static uint32_t
spiflash_getword(uint32_t addr)
{
    volatile const uint32_t *qspi = (volatile const uint32_t *)0x30000000;
    return qspi[addr];
}

static void memtest(void)
{
    volatile uint32_t *sram = (volatile uint32_t *)0x10000004;
    const int size = 64;
    uint32_t seed = 5381;
    int i;

    /* Test reading from SPI flash. */
    (void)spiflash_getword(0x1234);
    (void)spiflash_getword(0x1235);
    (void)spiflash_getword(0x1236);
    (void)spiflash_getword(0xDEAD);
    (void)spiflash_getword(0xBEEF);

    /* Write memory */
    for (i = 0; i < size; i++) {
        sram[i] = seed;
        /* Quick and dirty hash algorithm. */
        seed = (seed << 5) + seed + i;
    }

    /* Check memory. */
    seed = 5381;
    for (i = 0; i < size; i++) {
        if (sram[i] != seed) {
            printf("memtest error: addr=0x%08x expected=0x%08x got=0x%08x\n",
                (uint32_t)(uintptr_t)&sram[i], seed, sram[i]);
        }
        seed = (seed << 5) + seed + i;
    }
}

int main(void)
{
    volatile uint32_t *ledpwm = LED_PWM_BASE;
    static uint8_t val = 0xff;
    int toggle = 0;
    char ch = 'A';
    int i;
    int count = 0;
    
    memtest();

    for (i = 0; i < LED_PWM_COUNT; i++) {
        ledpwm[i] = val;
        val >>= 2;
    }
 
    printf("Hello World!\n");

    /* And finally - the main loop. */
    while (1) {
        /* If there are characters received, echo them back. */
        if (SERIAL->isr & 0x01) {
            uint8_t ch = SERIAL->rhr;
            ledpwm[0] = (toggle) ? 0xff : 0;
            toggle = (toggle == 0);
            count++;
#if 1
            if ((count % 64) == 0) {
            }
#else
            serial_putc(ch);
#endif
        }
    }
}
