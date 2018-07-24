#include "qemu/osdep.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/arm/arm.h"
#include "exec/address-spaces.h"
#include "qapi/error.h"
#include "hw/misc/unimp.h"
#include "hw/ptimer.h"
#include "sysemu/sysemu.h"
#include "nrf51.h"
#include "nrf51_radio.h"
#include "sys/socket.h"
#include "crypto/cipher.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#endif

/*************************************\
 * Defines that are required here.
\*************************************/
#define RTC_NUM_CC_REGS (4)
#define AES_ECB_BLOCK_SZ (16) //Unit is bytes
#define AES_ECB_READ_SZ (AES_ECB_BLOCK_SZ*2)
#define AES_ECB_CIPHERTEXT_OFFSET (AES_ECB_BLOCK_SZ*2)
#define AES_ECB_HDR_SZ (AES_ECB_BLOCK_SZ*3)

/*************************************\
 * Device Types
\*************************************/
static const char TYPE_NRF51_ADC[] = "nrf51-adc";
static const char TYPE_NRF51_CLOCK[] = "nrf51-clock";
static const char TYPE_NRF51_ECB[] = "nrf51-ecb";
static const char TYPE_NRF51_GPIO[] = "nrf51-gpio";
static const char TYPE_NRF51_GPTE[] = "nrf51-gpte";
static const char TYPE_NRF51_RADIO[] = "nrf51-radio";
static const char TYPE_NRF51_RNG[] = "nrf51-rng";
static const char TYPE_NRF51_RTC[] = "nrf51-rtc";
static const char TYPE_NRF51_TIMER[] = "nrf51-timer";
static const char TYPE_NRF51_UART[] = "nrf51-uart";

#pragma region DevInit
/*************************************\
 * Dev. Init & IO Function Prototypes.
\*************************************/
static void nrf51_adc_init(Object *obj);
static void nrf51_clock_init(Object *obj);
static void nrf51_ecb_init(Object *obj);
static void nrf51_gpio_init(Object *obj);
static void nrf51_gpte_init(Object *obj);
static void nrf51_radio_init(Object *obj);
static void nrf51_rng_init(Object *obj);
static void nrf51_rtc_init(Object *obj);
static void nrf51_timer_init(Object *obj);
static void nrf51_uart_init(Object *obj);

static void nrf51_adc_class_init(ObjectClass *class, void *data);
static void nrf51_clock_class_init(ObjectClass *class, void *data);
static void nrf51_ecb_class_init(ObjectClass *class, void *data);
static void nrf51_gpio_class_init(ObjectClass *class, void *data);
static void nrf51_gpte_class_init(ObjectClass *class, void *data);
static void nrf51_radio_class_init(ObjectClass *class, void *data);
static void nrf51_rng_class_init(ObjectClass *class, void *data);
static void nrf51_rtc_class_init(ObjectClass *class, void *data);
static void nrf51_timer_class_init(ObjectClass *class, void *data);
static void nrf51_uart_class_init(ObjectClass *class, void *data);

static void nrf51_gpio_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_clock_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_gpte_write(void *opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_rtc_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_adc_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_uart_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_ecb_write(void *opaque, hwaddr offset, uint64_t value, unsigned size);
static void nrf51_rng_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_timer_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);

static uint64_t nrf51_adc_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_clock_read(void * opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_ecb_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_gpio_read(void * opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_gpte_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_rng_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_rtc_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_timer_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_uart_read(void *opaque, hwaddr offset, unsigned int size);
#pragma endregion

#pragma region DeviceStates
/*************************************\
 * Device States
\*************************************/
typedef struct _nrf51_rng_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state * ptimer;
    struct
    {
        uint32_t VALRDY;
        uint32_t SHORTS;
        uint32_t INTEN;
        uint32_t CONFIG;
        uint32_t VALUE;
    } REG;
} nrf51_rng_state;

typedef struct _nrf51_gpio_state {
    SysBusDevice sb_parent;
    MemoryRegion iomem;

    struct _nrf51_gpte_state * owner[NUM_GPIO_PINS];
    int owner_id[NUM_GPIO_PINS];
    struct {
        uint32_t OUT;       //0x504
        uint32_t OUTSET;    //0x508
        uint32_t OUTCLR;    //0x50C
        uint32_t IN;
        uint32_t DIR;
        uint32_t DIRSET;
        uint32_t DIRCLR;
        uint32_t PINCNF[NUM_GPIO_PINS];
    } REG;
} nrf51_gpio_state;

typedef struct _nrf51_gpte_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    //ptimer_state * pt_conversion;

    //Can be used for in/out, depending on configured mode.
    uint8_t io_state[4];
    struct
    {
        //Events
        uint32_t IN[4];
        uint32_t PORT;

        //Registers
        uint32_t INTEN;
        uint32_t CONFIG[4];
    } REG;
} nrf51_gpte_state;

typedef struct _nrf51_clock_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    struct
    {
        //Events
        uint32_t HFCLKSTARTED;
        uint32_t LFCLKSTARTED;
        uint32_t DONE;
        uint32_t CTTO;

        //Registers
        uint32_t INTEN;
        uint32_t HFCLKRUN;
        uint32_t HFCLKSTAT;
        uint32_t LFCLKRUN;
        uint32_t LFCLKSTAT;
        uint32_t LFCLKSRCCOPY;
        uint32_t LFCLKSRC;
        uint32_t CTIV;
        uint32_t XTALFREQ;
    } REG;
} nrf51_clock_state;

typedef struct
{
    uint8_t key[AES_ECB_BLOCK_SZ];
    uint8_t cleartext[AES_ECB_BLOCK_SZ];
    uint8_t ciphertext[AES_ECB_BLOCK_SZ];
} nrf51_ecb_data;

//Make sure that struct is not aligned in any way.
QEMU_BUILD_BUG_ON(sizeof(nrf51_ecb_data) !=  AES_ECB_HDR_SZ);

typedef struct _nrf51_ecb_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    struct
    {
        //Events
        uint32_t ENDECB;
        uint32_t ERRORECB;

        //Registers
        uint32_t INTEN;
        uint32_t ECBDATAPTR;
    } REG;

    QCryptoCipher * cipher_ctx;
    uint8_t current_key[AES_ECB_BLOCK_SZ];
    nrf51_ecb_data ecb_data;
} nrf51_ecb_state;

typedef struct _nrf51_uart_state {
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state * ptimer;

    int bUartEnabled;
    int bFlowCtrlEnabled; //HW Flow Control
    int bCtsEnabled;
    int bNotCtsEnabled;
    int bNewByte;
    int bReadOk;
    uint32_t uTimerTaskFlags;

    struct {
        //Events
        uint32_t CTS;
        uint32_t NCTS;
        uint32_t RXDRDY;
        uint32_t TXDRDY;
        uint32_t ERROR;
        uint32_t RXTO; //RX Timeout

        //Registers
        uint32_t INTEN;
        uint32_t ENABLE;
        uint32_t PSELCTS;
        uint32_t PSELRTS;
        uint32_t PSELRXD;
        uint32_t PSELTXD;
        uint32_t RXD;
        uint32_t TXD;
        uint32_t BAUDRATE;
        uint32_t CONFIG;
    } REG;
} nrf51_uart_state;

typedef struct _nrf51_rtc_state {
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state * ptimer;
    int num_instance;
    bool bRunning;
    struct {
        //Tasks
        uint32_t START;
        uint32_t STOP;
        uint32_t CLEAR;
        uint32_t TRIGOVRFLW;
        //Events
        uint32_t TICK;
        uint32_t OVRFLW;
        uint32_t COMPARE[RTC_NUM_CC_REGS]; //4 Events
        //Registers
        uint32_t INTEN;
        uint32_t INTENSET;
        uint32_t INTENCLR;
        uint32_t EVTEN;
        uint32_t EVTENSET;
        uint32_t EVTENCLR;
        uint32_t COUNTER;
        uint32_t PRESCALER;
        uint32_t CC[RTC_NUM_CC_REGS]; // 4 Compare registers
    } REG;
} nrf51_rtc_state;

typedef struct _nrf51_adc_state {
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state * pt_conversion;
    int bConversionActive;
    int nNoiseBits;
    struct {
        uint32_t START;
        uint32_t STOP;
        uint32_t END;
        uint32_t INTEN;
        uint32_t ENABLE;
        uint32_t CONFIG;
        uint32_t RESULT;
    } REG;
} nrf51_adc_state;

typedef struct _nrf51_timer_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    //ptimer_state * pt_conversion;
    struct
    {

    } REG;
} nrf51_timer_state;
#pragma endregion

#pragma region TypeInfo
/*************************************\
 * TypeInfo
\*************************************/
static const TypeInfo nrf51_mod_gpio = {
    .name          = TYPE_NRF51_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_gpio_state),
    .instance_init = nrf51_gpio_init,
    .class_init    = nrf51_gpio_class_init,
};

static const TypeInfo nrf51_mod_gpte = {
    .name          = TYPE_NRF51_GPTE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_gpte_state),
    .instance_init = nrf51_gpte_init,
    .class_init    = nrf51_gpte_class_init,
};

static const TypeInfo nrf51_mod_clock = {
    .name          = TYPE_NRF51_CLOCK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_clock_state),
    .instance_init = nrf51_clock_init,
    .class_init    = nrf51_clock_class_init,
};

static const TypeInfo nrf51_mod_rtc = {
    .name          = TYPE_NRF51_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_rtc_state),
    .instance_init = nrf51_rtc_init,
    .class_init    = nrf51_rtc_class_init,
};

static const TypeInfo nrf51_mod_ecb = {
    .name          = TYPE_NRF51_ECB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_ecb_state),
    .instance_init = nrf51_ecb_init,
    .class_init    = nrf51_ecb_class_init,
};

static const TypeInfo nrf51_mod_adc = {
    .name          = TYPE_NRF51_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_adc_state),
    .instance_init = nrf51_adc_init,
    .class_init    = nrf51_adc_class_init,
};

static const TypeInfo nrf51_mod_uart = {
    .name          = TYPE_NRF51_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_uart_state),
    .instance_init = nrf51_uart_init,
    .class_init    = nrf51_uart_class_init,
};

static const TypeInfo nrf51_mod_radio = {
    .name          = TYPE_NRF51_RADIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_radio_state),
    .instance_init = nrf51_radio_init,
    .class_init    = nrf51_radio_class_init,
};

static const TypeInfo nrf51_mod_timer = {
    .name          = TYPE_NRF51_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_timer_state),
    .instance_init = nrf51_timer_init,
    .class_init    = nrf51_timer_class_init,
};

static const TypeInfo nrf51_mod_rng = {
    .name          = TYPE_NRF51_RNG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_rng_state),
    .instance_init = nrf51_rng_init,
    .class_init    = nrf51_rng_class_init,
};
#pragma endregion

#pragma region MemoryRegionOps
/*************************************\
 * MemoryRegionOps
\*************************************/
static const MemoryRegionOps nrf51_gpio_ops = {
    .read = nrf51_gpio_read,
    .write = nrf51_gpio_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_rtc_ops = {
    .read = nrf51_rtc_read,
    .write = nrf51_rtc_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_adc_ops = {
    .read = nrf51_adc_read,
    .write = nrf51_adc_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_uart_ops = {
    .read = nrf51_uart_read,
    .write = nrf51_uart_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_clock_ops = {
    .read = nrf51_clock_read,
    .write = nrf51_clock_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_ecb_ops = {
    .read = nrf51_ecb_read,
    .write = nrf51_ecb_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_radio_ops = {
    .read = nrf51_radio_read,
    .write = nrf51_radio_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_timer_ops = {
    .read = nrf51_timer_read,
    .write = nrf51_timer_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_rng_ops = {
    .read = nrf51_rng_read,
    .write = nrf51_rng_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_gpte_ops = {
    .read = nrf51_gpte_read,
    .write = nrf51_gpte_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#pragma endregion

//TODO: Use the correct value, check datasheet?
#define NUM_IRQ_LINES (64)

#define UART_ENABLE_REG_VAL  0x4
#define UART_DISABLE_REG_VAL 0x0

#define ECB_ERRORECB_MASK (0x2)
#define ECB_ENDECB_MASK (0x1)

#define SERVER_PORT 5151
#define SERVER_ADDR "127.0.0.1"

typedef struct
{
    hwaddr base;
    int irq;
} base_irq_pair;

typedef struct _nrf51_udp_conn
{
    int fd;
    struct sockaddr_in si_server;
    bool bCanWrite;
    uint_fast32_t uBytesRcvd;
    uint_fast32_t uBytesSent;
} nrf51_udp_conn;

static void nrf51_udp_send_id_handler(void * opaque, uint8_t * data, uint_fast16_t len);
static void nrf51_gpio_udp_handler(void * opaque, uint8_t * data, uint_fast16_t len);

static inline int nrf51_gpte_read_mode(nrf51_gpte_state *s, const int id)
{
    return s->REG.CONFIG[id] & 0x3;
}

static inline int nrf51_gpte_read_psel(nrf51_gpte_state *s, const int id)
{
    return (s->REG.CONFIG[id] >> 8) & 0x1f;
}

static inline int nrf51_gpte_read_polarity(nrf51_gpte_state *s, const int id)
{
    return (s->REG.CONFIG[id] >> 16) & 0x3;
}

static void nrf51_gpte_set_event(nrf51_gpte_state *s, const int id, const bool state)
{
    int polarity = nrf51_gpte_read_polarity(s, id);
    printf("GPIOTE event, id: %d\n", id);

    switch(polarity)
    {
        case GPTE_CONFIG_NONE:
            //No event generated.
            return;

        case GPTE_CONFIG_RISING:
            if (!state){
                //No event.
                return;
            }

            if (s->io_state[id]){
                //Already HIGH
                return;
            }
            break;

        case GPTE_CONFIG_FALLING:
            if(state){
                //No event.
                return;
            }

            if(!s->io_state[id]){
                //Already LOW
                return;
            }

        case GPTE_CONFIG_TOGGLE:
            //Let it generate event for any change.
            break;
    }

    s->io_state[id] = state;
    //FIXME: Check PORT event.
    s->REG.IN[id] = 1;
    if (s->REG.INTEN & (1<<id)){
        qemu_irq_pulse(s->irq);
    }
}

static const base_irq_pair RTC_BASE_IRQ_LIST[RTC_TOTAL] = {
    { RTC0_BASE, IRQ_RTC0 },
    { RTC1_BASE, IRQ_RTC1 }
    //{ RTC2_BASE, IRQ_RTC2 } //Doesn't exist on NRF51
};

static const base_irq_pair TIMER_BASE_IRQ_LIST[TIMER_TOTAL] = {
    { TIMER0_BASE, IRQ_TIMER0},
    { TIMER1_BASE, IRQ_TIMER1},
    { TIMER2_BASE, IRQ_TIMER2}
};

nrf51_udp_packet_handler packet_handlers[PROTO_TOTAL] =
{
    [PROTO_SEND_ID] = nrf51_udp_send_id_handler,
    [PROTO_RADIO] = nrf51_radio_do_rx,
    [PROTO_GPIO] = nrf51_gpio_udp_handler
};

void * packet_handler_context[PROTO_TOTAL] = {0x0};

//TODO: Use global prefix g_?
static nrf51_udp_conn udp_conn;

static DeviceState * nvic;

/*
 * Almost all modules are implemented
 * in a way that they can be duplicated
 * by only defining an unused memory
 * IO region.
 * For GPIO and GPIOTE modules, the
 * only exception is that they use
 * global variables to share state
 * information.
 *
 * Since NRF51 doesn't have any difference
 * between device models, it doesn't
 * require to be configurable from that
 * perspective but with small changes
 * NRF52 board can also be supported
 * and possibly future devices as well
 * with minimal changes. So we sill
 * have to prioritize portability.
 */

/*
 * This is a less portable solution
 * but GPIO and GPTE modules need to
 * know about each other's state.
 */
static nrf51_gpio_state * g_gpio_state;

static uint32_t seed_xsrand = 16777619; //Some prime number.

#if defined(__unix__) | defined(__APPLE__)
static void xsrand_init(void)
{
    ssize_t n;
    uint32_t seed;
    int fd = open("/dev/urandom", O_RDONLY);
    if(fd < 0){
        printf("cannon init random seed\n");
        return;
    }

    n = read(fd, &seed, sizeof(seed));

    if (n != sizeof(seed)){
        printf("unable to read required bits from urandom\n");
        return;
    }

    seed_xsrand ^= seed;

    printf("new seed: 0x%x\n", seed_xsrand);
}
#endif

/*
 * Warning: Following random function cannot
 * be used in any kind of cryptographic
 * application. It is only for educational
 * purposes.
 */
static uint32_t xsrand(void)
{
    uint32_t x = seed_xsrand;
    //Taken from https://en.wikipedia.org/wiki/Xorshift
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    seed_xsrand = x;

    return x;
}

static void nrf51_udp_send_id_handler(void * opaque, uint8_t * data, uint_fast16_t len)
{
    printf("warning: id packet received\n");
    //do nothing.
}

static void nrf51_gpio_udp_handler(void * opaque, uint8_t * data, uint_fast16_t len)
{
    nrf51_udp_gpio_hdr * gpio_msg = (nrf51_udp_gpio_hdr*)data;
    nrf51_udp_proto_hdr * hdr = (nrf51_udp_proto_hdr*)data;
    nrf51_gpio_state * s = opaque;
    uint8_t pin;
    bool state;

    if ( len != sizeof(nrf51_udp_gpio_hdr) ||
         udp_read_len(hdr) != UDP_GPIO_MSG_SZ ) {
        printf("gpio packet was corrupted, drop\n");
        return;
    }

    pin = gpio_msg->pin_state & 0x1f;
    state = gpio_msg->pin_state >> 7;

    printf("msg = set pin: %u <- %s\n", pin, state ? "HIGH" : "LOW");

   /*
    * GPIOTE module has priority on pins.
    * So if GPIOTE is configured to use
    * pins, this event must be routed
    * to GPIOTE.
    */

    if (s->owner[pin])
    {
        //Event mode = INPUT
        //Task mode = OUTPUT
        //GPIOTE.CONFIG[x] must be configured in event(input) mode.
        //Otherwise we ignore this request, because otherside
        //is configured its pin as output as well.
        if(nrf51_gpte_read_mode(s->owner[pin], s->owner_id[pin]) == GPTE_CONFIG_EVENT){
            nrf51_gpte_set_event(s->owner[pin], s->owner_id[pin], state);
        }
        else{
            printf("received input on GPIOTE task (output) pin\n");
        }

        return;
    }

    //a. Pin must be set in input mode
    //  0 == INPUT, 1 == OUTPUT
    //b. Input buffer must be connected.
    //  0 == Connected, 1 == Disconnected
    if ( !(s->REG.PINCNF[pin] & (MASK_GPIO_PINCNF_DIR | MASK_GPIO_PINCNF_INPUT)) )
    {
        //FIXME:
        //We would only set IN state here.
        //but at this stage we will only warn user.
        //We must keep the state of input (the input from the peer)
        //Rather than directly writing it into IN register.
        //This will be required when some other peripherals
        //are implemented.
    }
    else
    {
        printf("warning: pin '%u' was not configured as input "
               "and/or input buffer is disconnected\n", pin);
    }

    if (state){
        //Set
        s->REG.IN |= 1 << pin;
        printf("set pin: %u\n", pin);
    }
    else{
        //Clear
        s->REG.IN &= ~(1 << pin);
        printf("clear pin: %u\n", pin);
    }

}

static void nrf51_udp_rd_handler(void * opaque)
{
    uint8_t buffer[512];
    struct sockaddr_in si_server;
    socklen_t slen;
    nrf51_udp_conn * pCon = opaque;
    nrf51_udp_proto_hdr * hdr = (nrf51_udp_proto_hdr*)buffer;
    //si_server is not needed here...
    int len = recvfrom(pCon->fd, buffer, sizeof(buffer), 0, (struct sockaddr *) &si_server, &slen);

    if (len < 1)
    {
        printf("no data received or socket closed\n");
        return;
    }

    if (len < PROTO_HDR_SZ)
    {
        printf("packet too small, drop\n");
        return;
    }

    if (len != udp_read_len(hdr) + PROTO_HDR_SZ)
    {
        puts("drop, packet incomplete");
        return;
    }

    udp_conn.uBytesRcvd += (unsigned int) len;

    if (hdr->reserved != 0x00)
    {
        printf("reserved byte is non-zero, drop\n");
        return;
    }

    printf("received data, proto_type: %u\n", hdr->proto_type);

    if (hdr->proto_type < PROTO_TOTAL)
    {
        packet_handlers[hdr->proto_type](
            packet_handler_context[hdr->proto_type],
            buffer, (uint_fast16_t) len);
    }
    else
    {
        printf("packet type unknown, drop\n");
    }
}

static void nrf51_udp_wr_handler(void *opaque)
{
    udp_conn.bCanWrite = true;
    qemu_set_fd_handler(udp_conn.fd, nrf51_udp_rd_handler, NULL, opaque);
}

int nrf51_udp_send(void * data, size_t len)
{
    int sent;

    if (udp_conn.fd < 0)
    {
        printf("socket not initialized\n");
        return -1;
    }

    if (!udp_conn.bCanWrite)
    {
        //TODO: we actually 'might be' able to write
        //before event loop checks for fds.
        //It would be good to check if socket is writeable here instead.
        return -1;
    }

    udp_conn.bCanWrite = false;

    printf("sending data, len: %lu\n", len);
    sent = sendto(udp_conn.fd, data, len, 0,
            (struct sockaddr *) &udp_conn.si_server, sizeof(udp_conn.si_server));

    qemu_set_fd_handler(udp_conn.fd, nrf51_udp_rd_handler, nrf51_udp_wr_handler, &udp_conn);

    if (sent < 0)
    {
        printf("unable to send udp packet\n");
        //TODO: keep track of dropped data, if required.
        return -1;
    }
    else if(sent != len)
    {
        printf("not all bytes were sent\n");
    }

    udp_conn.uBytesSent += (uint_fast32_t) sent;
    printf("total bytes out: %u\n", udp_conn.uBytesSent);
    return sent;

}

void nrf51_udp_fill_hdr(nrf51_udp_proto_hdr *hdr, uint8_t proto_type, uint16_t len)
{
    hdr->proto_type = proto_type;
    hdr->reserved = 0x0;
    udp_set_len(hdr, len);
}

static void nrf51_udp_init(void)
{
    udp_conn = (nrf51_udp_conn){
        .fd = -1,
        .bCanWrite = true,
        .uBytesRcvd = 0,
        .uBytesSent = 0
    };

    udp_conn.fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if ( udp_conn.fd == -1 )
    {
        printf("unable to create UDP socket");
    }
    else
    {
        memset((char *) &udp_conn.si_server, 0x0, sizeof(udp_conn.si_server));
        udp_conn.si_server.sin_family = AF_INET;
        udp_conn.si_server.sin_port = htons(SERVER_PORT);
        
        if (inet_aton(SERVER_ADDR, &udp_conn.si_server.sin_addr) == 0)
        {
            printf("inet_aton() failed\n");
            close(udp_conn.fd);
            udp_conn.fd = -1;
        }
    }

    if(udp_conn.fd > -1)
    {
        nrf51_udp_send_id_hdr pkt;

        qemu_set_fd_handler(udp_conn.fd, nrf51_udp_rd_handler, NULL, &udp_conn);

        nrf51_udp_fill_hdr((nrf51_udp_proto_hdr*)&pkt,
                            PROTO_SEND_ID, UDP_SEND_ID_MSG_SZ);
        //FIXME: Get id from cmd line.
        udp_set_uint16(pkt.id, nrf_id);
        nrf51_udp_send(&pkt, sizeof(pkt));
    }
}

static void nrf51_init(MachineState *ms)
{
    int k;

    printf("[DP] Kernel file: %s\n", ms->kernel_filename);

    if (nrf_id >= 0xFFFF){
        printf("Please specify device id with -nrf-id <id>\n");
        exit(1);
    }
#if defined(__unix__) | defined(__APPLE__)
    xsrand_init();
#else
#warning "No random seed initializer function is defined for this platform."
#endif

    //DeviceState *nvic, *dev;

    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    MemoryRegion *system_memory = get_system_memory();

    /* Flash */
    memory_region_init_ram(flash, NULL, "nrf51_flash", FLASH_256K,
                           &error_fatal);
    memory_region_set_readonly(flash, true);
    memory_region_add_subregion(system_memory, 0x0, flash); //Subregion at offset 0x0

    /* SRAM */
    memory_region_init_ram(sram, NULL, "nrf51_sram", SRAM_32K,
                           &error_fatal);
    memory_region_add_subregion(system_memory, NRF51_SRAM_BASE, sram);

    //TODO: null check, can this even fail?
    nvic = armv7m_init(system_memory, FLASH_256K, NUM_IRQ_LINES,
                       ms->kernel_filename, ms->cpu_type);

    //Create global udp connection for multi instance communication.
    nrf51_udp_init();

    //Create GPIO Module
    sysbus_create_simple(TYPE_NRF51_GPIO, GPIO_BASE, NULL);

    //Create GPTE Module
    sysbus_create_simple(TYPE_NRF51_GPTE, GPTE_BASE, NULL);

    //Create CLOCK Module
    sysbus_create_simple(TYPE_NRF51_CLOCK, POWER_CLOCK_BASE, NULL);

    //Create RTC Modules
    for (k = 0; k < RTC_TOTAL; k++)
    {
        sysbus_create_simple(TYPE_NRF51_RTC, RTC_BASE_IRQ_LIST[k].base, NULL);
    }

    //Create TIMER Modules
    for (k = 0; k < TIMER_TOTAL; k++)
    {
        sysbus_create_simple(TYPE_NRF51_TIMER, TIMER_BASE_IRQ_LIST[k].base, NULL);
    }

    //Create ADC Module
    sysbus_create_simple(TYPE_NRF51_ADC, ADC_BASE, NULL);

    //Create UART Module
    sysbus_create_simple(TYPE_NRF51_UART, UART_BASE, NULL);

    /* FIXME:
     * The ECB, CCM, and AAR share the same AES module.
     * The ECB will always have lowest priority and if
     * there is a sharing conflict during encryption,
     * the ECB operation will be aborted and an ERRORECB
     * event will be generated.
     */
    //Create AES Module
    sysbus_create_simple(TYPE_NRF51_ECB, ECB_BASE, NULL);

    //Create RADIO Module
    sysbus_create_simple(TYPE_NRF51_RADIO, RADIO_BASE, NULL);

    sysbus_create_simple(TYPE_NRF51_RNG, RNG_BASE, NULL);

    //Used in is_manual_peripheral_setup_needed, called by SystemInit
    create_unimplemented_device("UNKNOWN", 0xF0000000, 0x1000);

}

static void nrf51_machine_init(MachineClass *mc)
{
    mc->desc = "nRF51 Series Development Board";
    mc->init = nrf51_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m3");
}

#define UART_STARTRX_TASK (1<<0)
#define UART_STARTTX_TASK (1<<2)

static uint64_t nrf51_gpio_read(void *opaque, hwaddr offset,
                                   unsigned int size)
{
    nrf51_gpio_state * s = opaque;
    switch (offset)
    {
        //Following 3 registers return the same value on read.
        case O_GPIO_OUT:
        case O_GPIO_OUTSET:
        case O_GPIO_OUTCLR:
            return s->REG.OUT; //Reading OUTSET/OUTCLR returns value of OUT.

        case O_GPIO_IN:
            return s->REG.IN;

        //Similar to OUT/OUTSET/OUTCLR register.
        case O_GPIO_DIR:
        case O_GPIO_DIRSET:
        case O_GPIO_DIRCLR:
            return s->REG.DIR;

        case O_GPIO_PIN_CNF0 ... O_GPIO_PIN_CNF31:
        {
            //Calculate index for the pin configuration registers.
            const unsigned int idx = (offset - O_GPIO_PIN_CNF0) / 4;
            return s->REG.PINCNF[idx];
        }
        default:
            return 0xFFFFFFFF;
    }

    printf("[uart] not implemented: 0x%u\n",
        (unsigned int) offset);

    return 0xFFFFFFFF;
}

static void nrf51_gpio_send_single(uint8_t pin, uint8_t state)
{
    nrf51_udp_gpio_hdr pkt;
    nrf51_udp_fill_hdr(&pkt.proto_hdr, PROTO_GPIO, 1);
    pkt.pin_state = ((!!state) << 7) | (pin & 0x1f);
    nrf51_udp_send(&pkt, sizeof(pkt));
}

static void nrf51_gpte_task_out(nrf51_gpte_state *s, int idx)
{
    uint8_t state;
    //TODO: Check if PSEL can be changed after CONFIG.
    int pin = nrf51_gpte_read_psel(s, idx);
    int polarity = nrf51_gpte_read_polarity(s, idx);

    switch(polarity)
    {
        case GPTE_CONFIG_NONE:
            //Don't sync
            return;

        case GPTE_CONFIG_RISING:
            state = 1;
            break;

        case GPTE_CONFIG_FALLING:
            state = 0;
            break;

        case GPTE_CONFIG_TOGGLE:
            state = !s->io_state[idx];
            break;
    }

    if (s->io_state[idx] != state){
        //Sync only if state is changed.
        s->io_state[idx] = state;
        nrf51_gpio_send_single(pin, state);
    }
}

static void nrf51_gpio_sync_state(nrf51_gpio_state *s, uint32_t old_value)
{
    int i;
    uint32_t modified = s->REG.OUT ^ old_value;
    //Send modified pin states.
    for (i = 0; i < 32; i++, modified >>= 1)
    {
        if (modified & 0x1)
        {
            if (s->owner[i]){
                printf("pin %d owned by GPIOTE, don't sync\n", i);
                //If this pin is controled by GPIOTE module
                //we must skip it when state change was attempted
                //from GPIO module.
                continue;
            }
            //TODO: Check if REG.PINCNF affects REG.DIR

            //Check that it is output pin.
            if (/*GET_BIT(s->REG.DIR, i) && */ GET_BIT(s->REG.PINCNF[i], BIT_GPIO_PINCNF_DIR))
            {
                const uint8_t state = (s->REG.OUT >> i) & 0x1;
                printf("send single: %0x\n", state);
                nrf51_gpio_send_single(i, state);
            }
            else
            {
                printf("won't sync input pin: %d\n", i);
            }
        }
    }
}

static void nrf51_gpio_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned int size)
{
    nrf51_gpio_state * s = opaque;
    uint32_t old_value;

    switch (offset)
    {
        case O_GPIO_OUT:
            if (s->REG.OUT != value)
            {
                old_value = s->REG.OUT;
                s->REG.OUT = value;
                nrf51_gpio_sync_state(s, old_value);
            }
            break;

        /*
         * Set individual bits of OUT register.
         * Writing 0 has no effect
         */
        case O_GPIO_OUTSET:
            old_value = s->REG.OUT;
            s->REG.OUT |= (uint32_t) value;
            if (old_value != s->REG.OUT)
            {
                nrf51_gpio_sync_state(s, old_value);
            }
            break;
        /*
         * Clear individual bits of OUT register.
         * Writing 0 has no effect.
         * Writing 1 clears bit.
         */
        case O_GPIO_OUTCLR:
            old_value = s->REG.OUT;
            s->REG.OUT &= (uint32_t) ~value;
            if (old_value != s->REG.OUT)
            {
                nrf51_gpio_sync_state(s, old_value);
            }
            break;

        case O_GPIO_DIR:
            printf("Set DIR: 0x%llx\n", value);
            s->REG.DIR = (uint32_t) value;
            break;

        case O_GPIO_DIRSET:
            s->REG.DIR |= (uint32_t) value;
            break;

        case O_GPIO_DIRCLR:
            s->REG.DIR &= (uint32_t) ~value;
            break;

        case O_GPIO_PIN_CNF0 ... O_GPIO_PIN_CNF31:
        {
            //Calculate index for the pin configuration registers.
            const unsigned int idx = (offset - O_GPIO_PIN_CNF0) / 4;
            s->REG.PINCNF[idx] = value;
            break;
        }
    }
}

static uint64_t nrf51_gpte_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_gpte_state *s = opaque;

    switch(offset)
    {
        //Events
        case O_GPTE_IN0: return s->REG.IN[0];
        case O_GPTE_IN1: return s->REG.IN[1];
        case O_GPTE_IN2: return s->REG.IN[2];
        case O_GPTE_IN3: return s->REG.IN[3];
        case O_GPTE_PORT: return s->REG.PORT;

        //Registers
        case O_GPTE_INTEN:
        case O_GPTE_INTENSET:
        case O_GPTE_INTENCLR:
            return s->REG.INTEN;

        case O_GPTE_CONFIG0: return s->REG.CONFIG[0];
        case O_GPTE_CONFIG1: return s->REG.CONFIG[1];
        case O_GPTE_CONFIG2: return s->REG.CONFIG[2];
        case O_GPTE_CONFIG3: return s->REG.CONFIG[3];

    }

    printf("gpiote rd unknown register\n");
    return (uint64_t)-1;
}

static void nrf51_gpte_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    nrf51_gpte_state *s = opaque;

    printf("---- gpte write at 0x%x\n", offset);
    switch(offset)
    {
        //Tasks
        case O_GPTE_OUT0 ... O_GPTE_OUT3:
        {
            //
            //FIXME: Check if PSEL can be changed after CONFIG.
            //

            int idx = (offset - O_GPTE_OUT0) / 4;
            if(nrf51_gpte_read_mode(s, idx) == GPTE_CONFIG_TASK){
                nrf51_gpte_task_out(s, idx);
            }
            break;
        }

        //Events
        case O_GPTE_IN0:
            s->REG.IN[0] = value;
            break;

        case O_GPTE_IN1:
            s->REG.IN[1] = value;
            break;

        case O_GPTE_IN2:
            s->REG.IN[2] = value;
            break;

        case O_GPTE_IN3:
            s->REG.IN[3] = value;
            break;

        case O_GPTE_PORT:
            s->REG.PORT = value;
            break;

        //Registers
        case O_GPTE_INTEN:
            printf("set gpte inten\n");
            s->REG.INTEN = value;
            break;

        case O_GPTE_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_GPTE_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_GPTE_CONFIG0 ... O_GPTE_CONFIG3:
        {
            //
            //FIXME: Check if PSEL can be changed after CONFIG.
            //

            int mode;
            int psel;
            int idx = (offset - O_GPTE_CONFIG0) / 4;
            s->REG.CONFIG[idx] = value;

            printf("---- config gpte: %d\n", idx);

            mode = nrf51_gpte_read_mode(s, idx);
            psel = nrf51_gpte_read_psel(s, idx);
            if (mode == GPTE_CONFIG_EVENT || mode == GPTE_CONFIG_TASK){
                //Assign an owner for this pin
                g_gpio_state->owner[psel] = s;
                g_gpio_state->owner_id[psel] = idx;

                if (mode == GPTE_CONFIG_TASK){
                    s->io_state[idx] = GET_BIT(s->REG.CONFIG[idx], BIT_GPTE_CONFIG_OUTINIT);
                    nrf51_gpio_send_single(psel, s->io_state[idx]);
                }
                else{
                    s->io_state[idx] = 0;
                }
            }
            break;
        }

        default:
            printf("gpte wr unknown register\n");
            break;
    }
}

static uint64_t nrf51_clock_read(void *opaque, hwaddr offset,
                                 unsigned int size)
{
    nrf51_clock_state *s = opaque;

    switch(offset)
    {
        //Events
        case O_CLOCK_HFCLKSTARTED: return s->REG.HFCLKSTARTED;
        case O_CLOCK_LFCLKSTARTED: return s->REG.LFCLKSTARTED;
        case O_CLOCK_DONE: return s->REG.DONE;
        case O_CLOCK_CTTO: return s->REG.CTTO;

        //Registers
        case O_CLOCK_INTENSET:
        case O_CLOCK_INTENCLR:
            return s->REG.INTEN;

        case O_CLOCK_HFCLKRUN: return s->REG.HFCLKRUN;
        case O_CLOCK_HFCLKSTAT: return s->REG.HFCLKSTAT;
        case O_CLOCK_LFCLKRUN: return s->REG.LFCLKRUN;
        case O_CLOCK_LFCLKSTAT: return s->REG.LFCLKSTAT;
        case O_CLOCK_LFCLKSRCCOPY: return s->REG.LFCLKSRCCOPY;
        case O_CLOCK_LFCLKSRC: return s->REG.LFCLKSRC;
        case O_CLOCK_CTIV: return s->REG.CTIV;
        case O_CLOCK_XTALFREQ: return s->REG.XTALFREQ;
    }
    return (uint64_t)-1;
}

static void nrf51_clock_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned int size)
{
    nrf51_clock_state *s = opaque;

    switch(offset)
    {
        //Tasks
        case O_CLOCK_HFCLKSTART:
            if (value){
                s->REG.HFCLKSTARTED = 1;
                s->REG.HFCLKRUN = 1;
                //Set running state
                s->REG.HFCLKSTAT = 1<<16;
                if (GET_BIT(s->REG.INTEN, BIT_CLOCK_INT_HFCLKSTARTED)){
                    qemu_irq_pulse(s->irq);
                }
            }
            break;

        case O_CLOCK_HFCLKSTOP:
            if(value){
                s->REG.HFCLKSTARTED = 0;
                s->REG.HFCLKRUN = 0;
                s->REG.HFCLKSTAT = 0;
            }
            break;

        case O_CLOCK_LFCLKSTART:
            if(value){
                s->REG.LFCLKSRCCOPY = s->REG.LFCLKSRC;
                s->REG.LFCLKSTARTED = 1;
                s->REG.LFCLKRUN = 1;
                //Set running state
                s->REG.LFCLKSTAT = 1<<16;
                if (GET_BIT(s->REG.INTEN, BIT_CLOCK_INT_LFCLKSTARTED)){
                    qemu_irq_pulse(s->irq);
                }
            }
            break;

        case O_CLOCK_LFCLKSTOP:
            if(value){
                s->REG.LFCLKSTARTED = 0;
                s->REG.LFCLKRUN = 0;
                s->REG.LFCLKSTAT = 0;
            }
            break;

        case O_CLOCK_CAL:
            if(value){
                s->REG.DONE = 1;
                if(GET_BIT(s->REG.INTEN, BIT_CLOCK_INT_DONE)){
                    qemu_irq_pulse(s->irq);
                }
            }
            break;

        //These tasks are HW level (calibration)
        case O_CLOCK_CTSTART:
        case O_CLOCK_CTSTOP:
            break;

        //Events
        case O_CLOCK_HFCLKSTARTED:
            s->REG.HFCLKSTARTED = value;
            break;
        case O_CLOCK_LFCLKSTARTED:
            s->REG.LFCLKSTARTED = value;
            break;
        case O_CLOCK_DONE:
            s->REG.DONE = value;
            break;
        case O_CLOCK_CTTO:
            s->REG.CTTO = value;
            break;

        //Registers
        case O_CLOCK_INTENSET:
            s->REG.INTEN |= value;
            break;
        case O_CLOCK_INTENCLR:
            s->REG.INTEN &= ~value;
            break;
        case O_CLOCK_LFCLKSRC:
            s->REG.LFCLKSRC = value & 0x3;
            break;
        case O_CLOCK_CTIV:
            s->REG.CTIV = value & 0x7F;
            break;
        case O_CLOCK_XTALFREQ:
            s->REG.XTALFREQ = value & 0xFF;
            break;
    }

}

static uint64_t nrf51_rtc_read(void *opaque, hwaddr offset,
                                   unsigned int size)
{
    const nrf51_rtc_state * s = opaque;
    switch(offset)
    {
        /* Tasks */
        case O_RTC_START:
        case O_RTC_STOP:
        case O_RTC_CLEAR:
        case O_RTC_TRIGOVRFLW:
            return 0; //TODO: Is it correct to return 0?

        /* Events */
        case O_RTC_TICK: return s->REG.TICK;
        case O_RTC_OVRFLW: return s->REG.OVRFLW;
        case O_RTC_COMPARE0: return s->REG.COMPARE[0];
        case O_RTC_COMPARE1: return s->REG.COMPARE[1];
        case O_RTC_COMPARE2: return s->REG.COMPARE[2];
        case O_RTC_COMPARE3: return s->REG.COMPARE[3];

        /* Registers */
        case O_RTC_COUNTER: return s->REG.COUNTER & MASK_RTC_COUNTER;
        case O_RTC_PRESCALER: return s->REG.PRESCALER;
        case O_RTC_CC0: return s->REG.CC[0];
        case O_RTC_CC1: return s->REG.CC[1];
        case O_RTC_CC2: return s->REG.CC[2];
        case O_RTC_CC3: return s->REG.CC[3];

        case O_RTC_INTEN:
        case O_RTC_INTENSET:
        case O_RTC_INTENCLR:
            return s->REG.INTEN;

        case O_RTC_EVTEN:
        case O_RTC_EVTENSET:
        case O_RTC_EVTENCLR:
            return s->REG.EVTEN;

    }

    return 0xffffffff;
}

static void nrf51_rtc_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned int size)
{
        nrf51_rtc_state * s = opaque;

        switch(offset)
        {
            /* Tasks */
            case O_RTC_START:
                if (!value || s->bRunning) //TODO: Compare '1' or any value?
                    break;

                //TODO: Frequency can be different depending on the clock source
                //Start task for counter.
                ptimer_set_freq(s->ptimer, 32768);
                ptimer_set_count(s->ptimer, s->REG.PRESCALER + 1);
                ptimer_run(s->ptimer, 1);
                printf("RTC started\n");
                //TODO: Do we even need to save start?
                break;

            case O_RTC_STOP:
                if (!value || !s->bRunning)
                    break;
                ptimer_stop(s->ptimer);
                break;

            case O_RTC_CLEAR:
                s->REG.COUNTER = 0;
                break;

            case O_RTC_TRIGOVRFLW:
                s->REG.COUNTER = 0xFFFFF0;
                break;

            /* Events */
            case O_RTC_TICK:
                s->REG.TICK = value;
                break;

            case O_RTC_OVRFLW:
                s->REG.OVRFLW = value;
                break;

            case O_RTC_COMPARE0:
                s->REG.COMPARE[0] = value;
                break;

            case O_RTC_COMPARE1:
                s->REG.COMPARE[1] = value;
                break;
            case O_RTC_COMPARE2:
                s->REG.COMPARE[2] = value;
                break;
            case O_RTC_COMPARE3:
                s->REG.COMPARE[3] = value;
                break;

            /* Registers */
            case O_RTC_PRESCALER:
                if (s->bRunning) //Read only when running.
                    break;
                //Prescaler size is 12-bit
                s->REG.PRESCALER = value & ((1<<12)-1);
                printf("Prescaler set to: %u\n", s->REG.PRESCALER);
                break;

            case O_RTC_INTEN:
                s->REG.INTEN = value;
                break;

            case O_RTC_INTENSET:
                s->REG.INTEN |= value;
                break;

            case O_RTC_INTENCLR:
                s->REG.INTEN &= ~value;
                break;

            case O_RTC_EVTEN:
                s->REG.EVTEN = value;
                break;

            case O_RTC_EVTENSET:
                s->REG.EVTEN |= value;
                break;

            case O_RTC_EVTENCLR:
                s->REG.EVTEN &= ~value;
                break;

            case O_RTC_CC0:
                s->REG.CC[0] = value;
                break;

            case O_RTC_CC1:
                s->REG.CC[1] = value;
                break;

            case O_RTC_CC2:
                s->REG.CC[2] = value;
                break;

            case O_RTC_CC3:
                s->REG.CC[3] = value;
                break;

            default:
                printf("[rtc] unk wr request at 0x%x, value: %llu\n", (unsigned int) offset, (unsigned long long)value);
                break;
        }

}

static void nrf51_rtc_tick(void *opaque)
{
    int i;
    uint32_t cc_bit;
    nrf51_rtc_state * s = opaque;
    bool pulse = false;

    s->REG.COUNTER++;
    if (s->REG.COUNTER > MASK_RTC_COUNTER)
    {
        s->REG.COUNTER = 0;
        s->REG.OVRFLW = 1;
        pulse = true;
    }

    //TODO: Check event routing register when PPI is implemented. p.105, figure 36
    if (s->REG.INTEN & MASK_RTC_INTEN_TICK)
    {
        s->REG.TICK = 1;
        pulse = true;
    }

    //Check for all compare events if ANY compare is enabled.
    //but only trigger enabled ones.
    if ( s->REG.INTEN & (MASK_RTC_INTEN_ALLCOMPARE) )
    {
        for(i = 0, cc_bit = MASK_RTC_INTEN_COMPARE0; i < RTC_NUM_CC_REGS; i++)
        {
            if (s->REG.INTEN & cc_bit) //Is event enabled?
            {
                if (s->REG.CC[i] == s->REG.COUNTER) //Does COUNTER match CC?
                {
                    s->REG.COMPARE[i] = 1;
                    pulse = true;
                }
            }
            //COMPARE0..COMPARE4 bits in INTENT register are adjacent.
            //Checking bits from 16 to 19.
            cc_bit <<= 1;
        }
    }

    if (pulse)
        qemu_irq_pulse(s->irq);

    ptimer_set_count(s->ptimer, s->REG.PRESCALER + 1);
    ptimer_run(s->ptimer, 1);
    //printf("rtc tick: %u\n", s->REG.COUNTER & MASK_RTC_COUNTER);
}

static uint64_t nrf51_adc_read(void *opaque, hwaddr offset,
                                   unsigned int size)
{
    //printf("[adc] rd at offset: 0x%x\n", (unsigned int) offset);
    nrf51_adc_state * s = opaque;

    switch(offset)
    {
        case O_ADC_CONFIG:
            return s->REG.CONFIG;

        case O_ADC_RESULT:
            return s->REG.RESULT;

        case O_ADC_ENABLE:
            return s->REG.ENABLE;

        case O_ADC_BUSY:
            return !!s->bConversionActive;

        case O_ADC_INTEN:
        case O_ADC_INTENSET:
        case O_ADC_INTENCLR:
            return s->REG.INTEN;

        case O_ADC_END:
            return s->REG.END;

    }
    return 0xffffffff;
}

static void nrf51_adc_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned int size)
{
    nrf51_adc_state * s = opaque;

    switch(offset)
    {
        case O_ADC_START:
            //printf("ADC_START\n");
            //Check that ADC is enabled and no conversion is active.
            if (!s->REG.ENABLE || s->bConversionActive)
                break;
            s->bConversionActive = true;
            ptimer_set_freq(s->pt_conversion, 32768); //TODO: 32kHz, too low?
            ptimer_set_count(s->pt_conversion, 1);
            ptimer_run(s->pt_conversion, 1);
            break;

        case O_ADC_STOP:
            if (!s->bConversionActive)
                break;
            s->bConversionActive = false;
            ptimer_stop(s->pt_conversion);
            break;

        case O_ADC_ENABLE:
            s->REG.ENABLE = value;
            break;

        case O_ADC_CONFIG:
            s->REG.CONFIG = value;
            break;

        case O_ADC_INTEN:
            s->REG.INTEN = value;
            break;

        case O_ADC_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_ADC_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_ADC_END:
            s->REG.END = value;
            break;

        default:
            printf("[adc] unk wr request at 0x%x, value: %llu\n", (unsigned int) offset, (unsigned long long)value);
            break;
    }
}

static void nrf51_adc_conversion_complete(void *opaque)
{
    nrf51_adc_state * s = opaque;
    uint32_t uRes = 600;

    if (s->nNoiseBits > 0)
    {
        const uint32_t uMask = (1 << (s->nNoiseBits - 1)) - 1;
        s->REG.RESULT = (uRes & ~uMask) | (xsrand() & uMask);
    }
    else
    {
        s->REG.RESULT = uRes;
    }

    s->REG.END = 1;

    s->bConversionActive = false;

    if (s->REG.INTEN & 0x1) //There is only one event for ADC.
    {
        qemu_irq_pulse(s->irq);
    }

}

static void nrf51_uart_tx_task(nrf51_uart_state * const s)
{
    char chPendingByte;

    if (s->bNewByte)
    {
        /*
        printf("(%u) new tx byte: 0x%x (%1s)\n",
            ctr,
            s->REG.TXD,
            (char*)&s->REG.TXD);
        */

        chPendingByte = s->REG.TXD;
        //TODO: Would it be better to keep bytes in buffer?
        if (1 != nrf51_uart_comm_write(&chPendingByte, 1))
        {
            //Couldn't write try again later.
            return;
        }

        /*
         * Set TXDRDY event and pulse IRQ.
         */
        s->REG.TXDRDY = 0x1;
        s->bNewByte = false;
        qemu_irq_pulse(s->irq);
    }
}

static void nrf51_uart_rx_task(nrf51_uart_state * const s)
{
    //TODO: Keep stats.
    enum
    {
        BUF_SIZE = 256,
        BUF_AVA = BUF_SIZE - 1,
        BUF_LIMIT = BUF_SIZE / 4 * 3
    };

    static char pchBuf[BUF_SIZE];
    static int nHead = 0;
    static int nTail = 0;

    if (nTail <= BUF_LIMIT)
    {
        int nRead;
        //Read data from UNIX socket.
        nRead = nrf51_uart_comm_read(pchBuf, BUF_AVA - nTail);
        if (nRead > 0)
        {
            nTail += nRead;
            //Just a sanity check, can't happen.
            if (nTail > BUF_AVA || nTail < 0)
            {
                //Something really bad must have happened here.
                nTail = BUF_AVA;
                return;
            }
        }
    }

    //VM didn't handle last byte
    if (!s->bReadOk)
        return;

    if (nHead >= nTail)
    {
        //Nothing to read in buffer.
        return;
    }

    s->REG.RXD = pchBuf[nHead];
    nHead++;
    //Unset this flag so we know that data is pending to be read.
    s->bReadOk = false;
    s->REG.RXDRDY = 0x1;
    qemu_irq_pulse(s->irq);

    if (nHead == nTail)
    {
        //If we consumed all bytes, then reset cursors.
        nHead = nTail = 0;
    }
}

static void nrf51_uart_timer(void *opaque)
{
    nrf51_uart_state * const s = opaque;
    static uint32_t ctr;
    ctr++;

    if (s->uTimerTaskFlags & UART_STARTRX_TASK)
    {
        nrf51_uart_rx_task(s);
    }

    if (s->uTimerTaskFlags & UART_STARTTX_TASK)
    {
        nrf51_uart_tx_task(s);
    }

    if (!s->uTimerTaskFlags)
    {
        printf("uart timer stop\n");
        //If there is no active task, stop.
        ptimer_stop(s->ptimer);
    }
}
/*
 * TODO: Be careful with clear on read
 * register. Check if there are any others.
 */
static uint64_t nrf51_uart_read(void *opaque, hwaddr offset,
                                   unsigned int size)
{
    nrf51_uart_state * s = opaque;

    switch(offset)
    {
        //Events
        case O_UART_RXDRDY:
            return s->REG.RXDRDY;
        case O_UART_TXDRDY:
            return s->REG.TXDRDY;
        case O_UART_ERROR:
            return s->REG.ERROR;
        case O_UART_RXTO:
            return s->REG.RXTO;

        case O_UART_INTEN:
        case O_UART_INTENSET:
        case O_UART_INTENCLR:
            return s->REG.INTEN;

        //Pin select registers.
        case O_UART_PSELCTS:
            return s->REG.PSELCTS;
        case O_UART_PSELRTS:
            return s->REG.PSELRTS;
        case O_UART_PSELRXD:
            return s->REG.PSELRXD;
        case O_UART_PSELTXD:
            return s->REG.PSELTXD;

        case O_UART_RXD:
            s->bReadOk = true;
            return s->REG.RXD;
        //case O_UART_ERRORSRC:
            //TODO: Need to simulate some errors on UART.
            //break;
    }

    printf("UART rd unimplemented reg: 0x%x\n", (unsigned int)offset);
    return (uint64_t) -1;
}

static void nrf51_uart_start_timer(nrf51_uart_state * s, uint32_t uFlag)
{
    const int start = !s->uTimerTaskFlags;

    s->uTimerTaskFlags |= uFlag;

    if(start)
    {
        printf("UART timer started\n");
        //TODO: Why frequency doesn't match?
        //TODO: when freq increased, TXD is slower?
        ptimer_set_freq(s->ptimer, 8192); //TODO: 32kHz, adjust baud rate.
        ptimer_set_count(s->ptimer, 1);

        //Set limit to 1 and enable reload.
        ptimer_set_limit(s->ptimer, 1, 1);

        //Continuous mode
        ptimer_run(s->ptimer, 0);
    }

}

static void nrf51_uart_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned int size)
{
    nrf51_uart_state * s = opaque;

    switch(offset)
    {
        //Tasks
        case O_UART_STARTRX:
            if (value & 0x1)
            {
                printf("Start RX\n");
                s->bReadOk = true;
                nrf51_uart_start_timer(s, UART_STARTRX_TASK);
            }
            break;

        case O_UART_STOPRX:
            if (value){
                s->uTimerTaskFlags &= ~UART_STARTRX_TASK;
                printf("Stop RX\n");
            }
            break;

        case O_UART_STARTTX:
            if (value & 0x1)
            {
                //printf("Start TX\n");
                nrf51_uart_start_timer(s, UART_STARTTX_TASK);
            }
            break;

        case O_UART_STOPTX:
            if (value){
                s->uTimerTaskFlags &= ~UART_STARTTX_TASK;
                printf("Stop TX\n");
            }
            break;

        case O_UART_SUSPEND:
            if(value){
                s->uTimerTaskFlags &= ~(UART_STARTTX_TASK | UART_STARTRX_TASK);
                printf("UART Suspend\n");
            }
            break;

        //Events
        case O_UART_RXDRDY:
            s->REG.RXDRDY = value;
            break;
        case O_UART_TXDRDY:
            s->REG.TXDRDY = value;
            break;
        case O_UART_ERROR:
            s->REG.ERROR = value;
            break;
        case O_UART_RXTO:
            s->REG.RXTO = value;
            break;

        case O_UART_ENABLE:
            s->REG.ENABLE = value;
            if (value == UART_ENABLE_REG_VAL)
            {
                s->bUartEnabled = true;
                printf("UART enabled.\n");
            }
            else if (value == UART_DISABLE_REG_VAL)
                s->bUartEnabled = false;
            else
            {
                //TODO: check with real hardware and see what happens.
            }
            break;

        case O_UART_INTEN:
            s->REG.INTEN = value;
            break;

        case O_UART_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_UART_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        //TODO: simulate baudrate with timers.
        case O_UART_BAUDRATE:
            s->REG.BAUDRATE = value;
            break;

        case O_UART_CONFIG:
            printf("UART CONFIG: 0x%x\n", (unsigned int) value);
            /*
             * We are not interested in parity,
             * only check HW flow control.
             */
            s->bFlowCtrlEnabled = (value & 0x1);
            s->REG.CONFIG = value;
            break;

        case O_UART_TXD:
            //printf("new tx byte\n");
            s->bNewByte = true;
            s->REG.TXD = value;
            break;
        /*
         * TODO: How to simulate disconnected (0xFFFFFFFF) state for pins?
         * Stop UART when pin is disconnected?
         */
        //Pin select registers are ignored.
        case O_UART_PSELCTS:
            s->REG.PSELCTS = value;
            break;

        case O_UART_PSELRTS:
            s->REG.PSELRTS = value;
            break;

        case O_UART_PSELRXD:
            s->REG.PSELRXD = value;
            break;

        case O_UART_PSELTXD:
            s->REG.PSELTXD = value;
            break;

        default:
            printf("UART wr unimplemented reg: 0x%x = 0x%x\n",
                (unsigned int) offset, (unsigned int) value);
            break;
    }
}

static uint64_t nrf51_rng_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_rng_state *s = opaque;

    switch(offset)
    {
        case O_RNG_VALRDY: return s->REG.VALRDY;
        case O_RNG_SHORTS: return s->REG.SHORTS;
        case O_RNG_CONFIG: return s->REG.CONFIG;
        case O_RNG_VALUE:  return s->REG.VALUE;

        case O_RNG_INTEN:
        case O_RNG_INTENSET:
        case O_RNG_INTENCLR:
            return s->REG.INTEN;

    }

    printf("[rng] unk read offset: %llx\n", offset);
    return (uint64_t)-1;
}

static void nrf51_rng_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    nrf51_rng_state * s = opaque;
    switch(offset)
    {
        case O_RNG_START:
            ptimer_set_freq(s->ptimer, 32768);
            ptimer_set_count(s->ptimer, 1);
            ptimer_set_limit(s->ptimer, 1, 1);
            ptimer_run(s->ptimer, 0);
            break;

        case O_RNG_STOP:
            ptimer_stop(s->ptimer);
            break;

        case O_RNG_VALRDY:
            s->REG.VALRDY = value;
            break;

        case O_RNG_SHORTS:
            s->REG.SHORTS = value;
            break;

        case O_RNG_INTEN:
            s->REG.INTEN = value;
            break;

        case O_RNG_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_RNG_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_RNG_CONFIG:
            s->REG.CONFIG = value;
            break;

        default:
            printf("[rng] unk write offset: %llx\n", offset);
            break;
    }
}

static uint64_t nrf51_timer_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    //nrf51_timer_state *s = opaque;

    return (uint64_t)-1;
}

static void nrf51_timer_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{

}

static void nrf51_ecb_init(Object *obj)
{
    nrf51_ecb_state *s = NRF51_ECB_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_ecb_ops, s,
                          "mod-aes-ecb", ECB_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_ECB);

}

static void nrf51_gpio_init(Object *obj)
{
    //DeviceState *dev = DEVICE(obj);
    nrf51_gpio_state *s = NRF51_GPIO_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    g_gpio_state = s;

    memory_region_init_io(&s->iomem, obj, &nrf51_gpio_ops, s,
                          "mod-gpio", GPIO_REG_SPACE);
    sysbus_init_mmio(sbd, &s->iomem);

    packet_handler_context[PROTO_GPIO] = s;
}

static void nrf51_gpte_init(Object *obj)
{
    nrf51_gpte_state *s = NRF51_GPTE_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    //QEMUBH *bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_gpte_ops, s,
                          "mod-gpiote", GPTE_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_GPTE);

    //bh = qemu_bh_new(..., s);
    //s->ptimer = ptimer_init(bh, TIMER_POLICY_DEFAULT);

}

static void nrf51_clock_init(Object *obj)
{
    nrf51_clock_state *s = NRF51_CLOCK_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    //QEMUBH *bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_clock_ops, s,
                          "mod-clock", POWER_CLOCK_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_POWER_CLOCK);

    //bh = qemu_bh_new(..., s);
    //s->ptimer = ptimer_init(bh, TIMER_POLICY_DEFAULT);
}

/*
 * Called for each RTC module
 */
static void nrf51_rtc_init(Object *obj)
{
    static int num_rtc_instance;
    static char dev_name[] = "mod-rtcX";
    nrf51_rtc_state *s = NRF51_RTC_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    QEMUBH *bh;

    INIT_DEV_STATE(s);

    dev_name[sizeof(dev_name) - 2] = '0' + num_rtc_instance;
    s->num_instance = num_rtc_instance;

    memory_region_init_io(&s->iomem, obj, &nrf51_rtc_ops, s,
                          dev_name, RTC_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, RTC_BASE_IRQ_LIST[num_rtc_instance].irq);

    //Periodic timer to track RTC's counter value.
    bh = qemu_bh_new(nrf51_rtc_tick, s);
    s->ptimer = ptimer_init(bh, PTIMER_POLICY_DEFAULT);
    s->bRunning = false;

    num_rtc_instance++;
}

static void nrf51_adc_init(Object *obj)
{
    nrf51_adc_state *s = NRF51_ADC_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    QEMUBH *bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_adc_ops, s,
                          "mod-adc", ADC_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_ADC);

    //A timer to simulate ADC conversion.
    bh = qemu_bh_new(nrf51_adc_conversion_complete, s);
    s->pt_conversion = ptimer_init(bh, PTIMER_POLICY_DEFAULT);

    s->bConversionActive = false;

    //0 = No noise,
    //n+1 = n bits noise
    s->nNoiseBits = 4 + 1;
}

static void nrf51_uart_init(Object *obj)
{
    nrf51_uart_state *s = NRF51_UART_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    QEMUBH *bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_uart_ops, s,
                          "mod-uart", UART_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_UART);

    //A timer to simulate uart speeds and events.
    bh = qemu_bh_new(nrf51_uart_timer, s);
    s->ptimer = ptimer_init(bh, PTIMER_POLICY_DEFAULT);

    //Initialize UNIX socket
    nrf51_uart_comm_init();
}

void nrf51_trigger_hardfault(void)
{
    printf("FIXME: trigger HardFault\n");
    fflush(stdout);
    for(;;)
        ;
}

static void nrf51_clock_class_init(ObjectClass *class, void *data)
{

}

static void nrf51_ecb_class_init(ObjectClass *class, void *data)
{

    if(!qcrypto_cipher_supports(QCRYPTO_CIPHER_ALG_AES_128, QCRYPTO_CIPHER_MODE_ECB))
    {
        fprintf(stderr, "NRF51 requires AES128 cipher support\n");
        /*
         * FIXME:
         * 2. choices:
         * - Set a flag here and do not init. ecb module.
         * - Init. ECB module but guest won't be able
         *   to get result, instead ERRORECB will be
         *   triggered.
         */
    }
}

static void nrf51_gpio_class_init(ObjectClass *class, void *data)
{

}

static void nrf51_gpte_class_init(ObjectClass *class, void *data)
{

}

static void nrf51_rtc_class_init(ObjectClass *class, void *data)
{

}

static void nrf51_adc_class_init(ObjectClass *class, void *data)
{

}

static void nrf51_uart_class_init(ObjectClass *class, void *data)
{

}

static void nrf51_radio_init(Object *obj)
{
    nrf51_radio_state *s = NRF51_RADIO_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    QEMUBH *bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_radio_ops, s,
                          "mod-radio", RADIO_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_RADIO);

    bh = qemu_bh_new(nrf51_radio_timer, s);
    s->ptimer = ptimer_init(bh, PTIMER_POLICY_DEFAULT);

    s->REG.POWER = 1;

    packet_handler_context[PROTO_RADIO] = s;

    nrf51_radio_udp_init(s);
}

static void nrf51_timer_init(Object *obj)
{
    static int num_timer_instance;
    static char dev_name[] = "mod-timerX";
    nrf51_timer_state *s = NRF51_TIMER_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    //QEMUBH *bh;

    INIT_DEV_STATE(s);

    //replace 'X' with instance index.
    dev_name[sizeof(dev_name) - 2] = '0' + num_timer_instance;

    memory_region_init_io(&s->iomem, obj, &nrf51_timer_ops, s,
                          dev_name, TIMER_REG_SPACE); /*FIXME: use mod-timer0,1,2 */

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, TIMER_BASE_IRQ_LIST[num_timer_instance].irq);

    //bh = qemu_bh_new(nrf51_uart_timer, s);
    //s->ptimer = ptimer_init(bh, PTIMER_POLICY_DEFAULT);

    num_timer_instance++;

}

static void nrf51_rng_random_cb(void * opaque)
{
    nrf51_rng_state * s = opaque;
    uint32_t val = xsrand();

    val = val ^ (val >> 16);
    val = val ^ (val >> 8);

    s->REG.VALRDY = 1;
    s->REG.VALUE = val & 0xFF;

    if (s->REG.INTEN & 0x1)
    {
        qemu_irq_pulse(s->irq);
    }

    if ( s->REG.SHORTS & 0x1 )
    {
        ptimer_stop(s->ptimer);
    }
}

static void nrf51_rng_init(Object *obj)
{
    nrf51_rng_state *s = NRF51_RNG_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    QEMUBH * bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_rng_ops, s,
                          "mod-rng", RNG_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_RNG);

    /*
     * Instead of generating random number
     * right after START task is triggered
     * it will be generated by a timer
     * so there will be a small delay.
     */
    bh = qemu_bh_new(nrf51_rng_random_cb, s);
    s->ptimer = ptimer_init(bh, PTIMER_POLICY_DEFAULT);

}

static uint64_t nrf51_ecb_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_ecb_state *s = opaque;

    switch(offset)
    {
        case O_ECB_ENDECB:     return s->REG.ENDECB;
        case O_ECB_ERRORECB:   return s->REG.ERRORECB;
        case O_ECB_ECBDATAPTR: return s->REG.ECBDATAPTR;

        case O_ECB_INTENSET:
        case O_ECB_INTENCLR:
            /* 
             * TODO:
             * This register doesn't have base address
             * in ref. manual. However, it mentions
             * existence of it. It might be internal.
             */
            return s->REG.INTEN;
    }

    printf("[ecb_read] undefined register: 0x%lx\n", (unsigned long) offset);

    return (uint64_t)-1;
}

static void nrf51_ecb_error(nrf51_ecb_state *s)
{
    s->REG.ERRORECB = 1;
    if (s->REG.INTEN & ECB_ERRORECB_MASK) {
        qemu_irq_pulse(s->irq);
    }
}

static void nrf51_ecb_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    nrf51_ecb_state *s = opaque;
    MemTxResult res;
    Error *err;

    switch(offset)
    {
        //Tasks
        case O_ECB_STARTECB:
            if(NRF51_OUT_OF_RAM(s->REG.ECBDATAPTR, AES_ECB_HDR_SZ))
            {
                nrf51_trigger_hardfault();
                break;
            }
            //Fetch AES data from guest.
            res = address_space_read(&address_space_memory,
                    s->REG.ECBDATAPTR,
                    MEMTXATTRS_UNSPECIFIED,
                    s->ecb_data.key,
                    AES_ECB_READ_SZ); //Only read KEY and CLEARTEXT
            if (res != MEMTX_OK)
            {
                printf("ecb mem read error\n");
                nrf51_ecb_error(s);
                break;
            }

            if(s->cipher_ctx)
            {
                /*
                 * Compare current key and obtained key.
                 */
                if(memcmp(s->current_key, s->ecb_data.key, AES_ECB_BLOCK_SZ)){
                    //Key changed
                    qcrypto_cipher_free(s->cipher_ctx);
                    s->cipher_ctx = NULL;
                }
            }

            /*
             * Note:
             * if(s->cipher_ctx) and if(!s->cipher_ctx)
             * conditions can be fulfilled in the same
             * execution path.
             */

            if (!s->cipher_ctx){
                memcpy(s->current_key, s->ecb_data.key, sizeof(s->current_key));
                s->cipher_ctx = qcrypto_cipher_new(
                                    QCRYPTO_CIPHER_ALG_AES_128,
                                    QCRYPTO_CIPHER_MODE_ECB,
                                    s->current_key,
                                    G_N_ELEMENTS(s->current_key),
                                    &err);
                if(!s->cipher_ctx){
                    printf("[ecb] Cannot create cipher context\n");
                    nrf51_ecb_error(s);
                    break;
                }
            }

            //At this point we will always have cipher_ctx
            if (qcrypto_cipher_encrypt(s->cipher_ctx, s->ecb_data.cleartext,
                        s->ecb_data.ciphertext,
                        G_N_ELEMENTS(s->current_key),
                        &err) < 0){
                    printf("[ecb] encrypt returned error\n");
                    nrf51_ecb_error(s);
                    break;
            }

            //All good, put it into guest RAM.
            address_space_write(&address_space_memory,
                                s->REG.ECBDATAPTR + AES_ECB_CIPHERTEXT_OFFSET,
                                MEMTXATTRS_UNSPECIFIED, s->ecb_data.ciphertext,
                                AES_ECB_BLOCK_SZ);
            s->REG.ENDECB = 1;
            if(s->REG.INTEN & ECB_ENDECB_MASK){
                qemu_irq_pulse(s->irq);
            }
            break;

        case O_ECB_STOPECB:
            /*
             * It is not much possible to abort
             * ECB operation because QEMU doesn't
             * return to event loop until register
             * rd/wr function is returned. In that
             * case ECB operation will already be
             * finished when guest tries to write
             * into this register. So we ignore it.
             * FIXME: Check if ERRORECB is triggered
             * when AES operation is not started.
             */
            break;

        //Events
        case O_ECB_ENDECB:
            s->REG.ENDECB = value;
            break;

        case O_ECB_ERRORECB:
            s->REG.ERRORECB = value;
            break;

        //Registers
        case O_ECB_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_ECB_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_ECB_ECBDATAPTR:
            s->REG.ECBDATAPTR = value;
            break;

        default:
            printf("[ecb_write] undefined register: 0x%lx\n",
                (unsigned long) offset);
            break;
    }

}

static void nrf51_radio_class_init(ObjectClass *class, void *data)
{
}

static void nrf51_timer_class_init(ObjectClass *class, void *data)
{
}

static void nrf51_rng_class_init(ObjectClass *class, void *data)
{
}

/*************************************\
 * QEMU Specific Calls
\*************************************/
static void nrf51_register_types(void)
{
    type_register_static(&nrf51_mod_adc);
    type_register_static(&nrf51_mod_clock);
    type_register_static(&nrf51_mod_ecb);
    type_register_static(&nrf51_mod_gpio);
    type_register_static(&nrf51_mod_gpte);
    type_register_static(&nrf51_mod_radio);
    type_register_static(&nrf51_mod_rng);
    type_register_static(&nrf51_mod_rtc);
    type_register_static(&nrf51_mod_timer);
    type_register_static(&nrf51_mod_uart);

}

type_init(nrf51_register_types)

DEFINE_MACHINE("nrf51", nrf51_machine_init)

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
