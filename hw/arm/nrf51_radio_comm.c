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

/* NOTES From Ref Manual *//*

17.1.3 Maximum packet length
Independent of the configuration of MAXLEN, the combined length of S0, LENGTH, S1 and PAYLOAD cannot exceed 254 bytes.

*//* NOTES END */

/* Defines */

/* Static Functions */
static void nrf51_radio_task_start(nrf51_radio_state *s);
static void nrf51_radio_timer_start(nrf51_radio_state *s);
static void nrf51_radio_timer_stop(nrf51_radio_state * s);
static const char * task_to_str(NRF51_RADIO_TASK task);
static const char * state_to_str(NRF51_RADIO_STATE state);
static void nrf51_radio_task_txen(nrf51_radio_state *s);
static void nrf51_radio_task_rxen(nrf51_radio_state *s);
static int nrf51_radio_udp_send(nrf51_radio_state * s, nrf51_air_packet_buff * pkt);
static void nrf51_radio_pulse_event(const nrf51_radio_state * s, uint32_t mask);

/* Static Globals */
static nrf51_air_packet_buff air_packet;

static void nrf51_radio_reset(nrf51_radio_state *s)
{
    //Reset anything starting from radio_state field up to REG.
    memset(&s->radio_state, 0x00,
            offsetof(nrf51_radio_state, REG) - offsetof(nrf51_radio_state, radio_state));
    memset(&s->REG, 0x00, sizeof(s->REG));
}

uint64_t nrf51_radio_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    nrf51_radio_state *s = opaque;
    switch(offset)
    {
        /* Events */
        case O_RADIO_READY:     return s->REG.READY;
        case O_RADIO_END:       return s->REG.END;
        case O_RADIO_PAYLOAD:   return s->REG.PAYLOAD;
        case O_RADIO_ADDRESS:   return s->REG.ADDRESS;
        case O_RADIO_DISABLED:  return s->REG.DISABLED;
        case O_RADIO_DEVMATCH:  return s->REG.DEVMATCH;
        case O_RADIO_DEVMISS:   return s->REG.DEVMISS;
        case O_RADIO_RSSIEND:   return s->REG.RSSIEND;

        /* Registers */
        case O_RADIO_INTENSET:
        case O_RADIO_INTENCLR:
            return s->REG.INTEN;

        case O_RADIO_CRCSTATUS: return s->REG.CRCSTATUS;
        case O_RADIO_CRCCNF:    return s->REG.CRCCNF;
        case O_RADIO_RXMATCH:   return s->REG.RXMATCH;
        case O_RADIO_RXCRC:     return s->REG.RXCRC;
        case O_RADIO_PACKETPTR: return s->REG.PACKETPTR;
        case O_RADIO_FREQUENCY: return s->REG.FREQUENCY;
        case O_RADIO_POWER:     return s->REG.POWER;
        case O_RADIO_PCNF0:     return s->REG.PCNF0;
        case O_RADIO_PCNF1:     return s->REG.PCNF1;
        case O_RADIO_TXPOWER:   return s->REG.TXPOWER;
        case O_RADIO_MODE:      return s->REG.MODE;
        case O_RADIO_BASE0:     return s->REG.BASE0;
        case O_RADIO_BASE1:     return s->REG.BASE1;
        case O_RADIO_RXADDRESSES: return s->REG.RXADDRESSES;

    }

    printf("[radio] RD not implemented: 0x%x\n", (unsigned int) offset);
    return (uint64_t) -1;
}

void nrf51_radio_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    nrf51_radio_state *s = opaque;
    switch(offset)
    {
        /* Events */
        case O_RADIO_END:       s->REG.END      = value; break;
        case O_RADIO_READY:     s->REG.READY    = value; break;
        case O_RADIO_PAYLOAD:   s->REG.PAYLOAD  = value; break;
        case O_RADIO_ADDRESS:   s->REG.ADDRESS  = value; break;
        case O_RADIO_DEVMISS:   s->REG.DEVMISS  = value; break;
        case O_RADIO_RSSIEND:   s->REG.RSSIEND  = value; break;
        case O_RADIO_DISABLED:  s->REG.DISABLED = value; break;
        case O_RADIO_DEVMATCH:  s->REG.DEVMATCH = value; break;

        /* Registers */
        case O_RADIO_INTENSET:
            s->REG.INTEN |= value;
            break;
        case O_RADIO_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_RADIO_PACKETPTR:
            //TODO: check that ptr is in Data RAM otherwise give HardFault.
            //Check 5.1 for memory regions.
            //Byte aligned RAM address.
            s->REG.PACKETPTR = value;
            break;

        case O_RADIO_FREQUENCY:
            //TODO: Allowed value is [0..100]. Need to check?
            if (value > 100)
                printf("error: frequency value too big: %llu", value);
            s->REG.FREQUENCY = value;
            RDP("set freq: %u", (unsigned int) value + 2400);
            break;

        case O_RADIO_POWER:
            if ( !(value & 0x1) )
            {
                //Reset device state.
                nrf51_radio_reset(s);
            }
            s->REG.POWER = !!value;
            break;

        case O_RADIO_PCNF0:
            s->REG.PCNF0 = value;
            break;

        case O_RADIO_PCNF1:
            s->REG.PCNF1 = value;
            //Other configuration values are acquired during START task.
            s->ActiveConf.PCNF1.MAXLEN = PCNF1_MAXLEN_READ(s->REG.PCNF1);
            s->ActiveConf.PCNF1.WHITEEN = PCNF1_WHITEEN_READ(s->REG.PCNF1);
            break;

        case O_RADIO_TXPOWER:
            //Ineffective in QEMU.
            s->REG.TXPOWER = value;
            break;

        case O_RADIO_MODE:
            s->REG.MODE = value & 0x3;
            break;

        case O_RADIO_PREFIX0:
            s->REG.PREFIX0 = value;
            break;

        case O_RADIO_PREFIX1:
            s->REG.PREFIX1 = value;
            break;

        case O_RADIO_BASE0:
            s->REG.BASE0 = value;
            break;

        case O_RADIO_BASE1:
            s->REG.BASE1 = value;
            break;

        case O_RADIO_TXADDRESS:
            s->REG.TXADDRESS = value & 0x7;
            break;

        case O_RADIO_RXADDRESSES:
            s->REG.RXADDRESSES = value & 0xff;
            break;

        case O_RADIO_CRCCNF:
            s->REG.CRCCNF = value & 0x1ff; //TODO: Do bit mask? What is the device behavior?
            break;

        case O_RADIO_CRCINIT:
            s->REG.CRCINIT = value & 0xFFFFFF;
            break;

        case O_RADIO_CRCPOLY:
            s->REG.CRCPOLY = value & 0xFFFFFF;
            break;

        /* Tasks */
        case O_RADIO_TXEN:
            s->REG.TXEN = !!value; //TODO: Should we keep the value for tasks?
            if (s->REG.TXEN) //TODO: check for == 1 or any value?
                nrf51_radio_task_txen(s);
            break;

        case O_RADIO_RXEN:
            s->REG.RXEN = !!value;
            if (s->REG.RXEN) //TODO: check for == 1 or any value?
                nrf51_radio_task_rxen(s);
            break;

        case O_RADIO_START:
            //TODO: Check if task is already running?
            s->REG.START = !!value;
            if (s->REG.START)
            {
                nrf51_radio_task_start(s);
            }
            break;

        case O_RADIO_DISABLE:
            if (s->radio_task != enmTaskNone)
            {
                RDP("error: a task is already running: %s", task_to_str(s->radio_task));
            }
            s->radio_task = enmTaskDisable;
            nrf51_radio_timer_start(s);
            break;
//TODO: check for break; in switch statements in entire project.
        default:
            printf("[radio] WR not implemented: 0x%x\n", (unsigned int) offset);
            break;
    }
}

static void nrf51_radio_task_txen(nrf51_radio_state *s)
{
    RDP("%s", "Trigger TXEN");
    //FIXME: get value in state handler (timer).
    s->ActiveConf.uFrequency = s->REG.FREQUENCY & 0x7f;
    //TODO: what is the decision point for MODE?
    s->ActiveConf.uMode = s->REG.MODE;
    if (s->radio_task != enmTaskNone)
    {
        RDP("error: a task is already running: %s", task_to_str(s->radio_task));
    }
    s->radio_task = enmTaskTxEn;
    nrf51_radio_timer_start(s);
}

static void nrf51_radio_task_rxen(nrf51_radio_state *s)
{
    RDP("%s", "Trigger RXEN");
    s->ActiveConf.uFrequency = s->REG.FREQUENCY & 0x7f;
    //TODO: what is the decision point for MODE?
    s->ActiveConf.uMode = s->REG.MODE;
    if (s->radio_task != enmTaskNone)
    {
        RDP("error: a task is already running: %s", task_to_str(s->radio_task));
    }
    s->radio_task = enmTaskRxEn;
    nrf51_radio_timer_start(s);
}

static void nrf51_radio_task_start(nrf51_radio_state *s)
{
    int i,j;
    //Get address prefix (AP0..AP3)
    for (i = 0; i < 4; i++)
        s->ActiveConf.AP[i] = READ_MSB_BYTE(s->REG.PREFIX0, i);

    //Get address prefix (AP4..AP7)
    for (i = 0, j = 4; i < 4; i++, j++)
        s->ActiveConf.AP[j] = READ_MSB_BYTE(s->REG.PREFIX1, i);

    RDP("AP: %02x:%02x:%02x:%02x %02x:%02x:%02x:%02x",
        s->ActiveConf.AP[0], s->ActiveConf.AP[1],
        s->ActiveConf.AP[2], s->ActiveConf.AP[3],
        s->ActiveConf.AP[4], s->ActiveConf.AP[5],
        s->ActiveConf.AP[6], s->ActiveConf.AP[7]);

    //Save base addresses
    s->ActiveConf.BASE[0] = s->REG.BASE0;
    s->ActiveConf.BASE[1] = s->REG.BASE1;
    RDP("BASE0: 0x%x, BASE1: 0x%x", s->REG.BASE0, s->REG.BASE1);

    //Save address select
    s->ActiveConf.uTxAddrSelect = s->REG.TXADDRESS;
    RDP_DUMP(s->ActiveConf.uTxAddrSelect);

    //Save RX address enable bits.
    for (int i = 0; i < 8; i++)
    {
        //Extract ADDRx enable bits from register.
        s->ActiveConf.RXADDRESSES_ADDR[i] = GET_BIT(s->REG.RXADDRESSES, i);
//        RDP_DUMP(s->ActiveConf.RXADDRESSES_ADDR[i]);
    }

    //Save PCNF0 values
    s->ActiveConf.PCNF0.LFLEN = s->REG.PCNF0 & 0xf;
    s->ActiveConf.PCNF0.S0LEN = s->REG.PCNF0 >> 8 & 0x1;
    s->ActiveConf.PCNF0.S1LEN = s->REG.PCNF0 >> 16 & 0xf;

    RDP("PCNF0 {LFLEN: %u, S0LEN: %u, S1LEN: %u}",
        s->ActiveConf.PCNF0.LFLEN,
        s->ActiveConf.PCNF0.S0LEN,
        s->ActiveConf.PCNF0.S1LEN);

    //Save PCNF1 values
    s->ActiveConf.PCNF1.STATLEN = s->REG.PCNF1 & 0xff;
    s->ActiveConf.PCNF1.BALEN = (s->REG.PCNF1 >> 16) & 0x7;
    s->ActiveConf.PCNF1.ENDIAN = GET_BIT(s->REG.PCNF1, 24); //TODO: do conversion in packet later.

    RDP_DUMP(s->ActiveConf.PCNF1.STATLEN);
    RDP_DUMP(s->ActiveConf.PCNF1.BALEN);
    RDP_DUMP(s->ActiveConf.PCNF1.ENDIAN);
    //This value is active as soon as it is written to PCNF1;
    RDP_DUMP(s->ActiveConf.PCNF1.MAXLEN);

    //Save CRCCNF, CRCINIT, CRCPOLY
    s->ActiveConf.CRCCNF.uLen = s->REG.CRCCNF & 0x3;
    s->ActiveConf.CRCCNF.bSkipAddr = GET_BIT(s->REG.CRCCNF, 8);
    s->ActiveConf.CRCINIT = s->REG.CRCINIT;
    s->ActiveConf.CRCPOLY = s->REG.CRCPOLY;

    RDP("CRCCNF {LEN: %u, SKIPADDR: %u}",
        s->ActiveConf.CRCCNF.uLen,
        s->ActiveConf.CRCCNF.bSkipAddr);
    RDP("CRCINIT: 0x%x, CRCPOLY: 0x%x",
        s->ActiveConf.CRCINIT,
        s->ActiveConf.CRCPOLY);

    s->ActiveConf.PACKETPTR = s->REG.PACKETPTR;
    s->radio_task = enmTaskStart;
    nrf51_radio_timer_start(s);
}

static void nrf51_radio_timer_start(nrf51_radio_state *s)
{
    if (!s->REG.POWER)
        return;

    if (s->timer_running)
        return;

    ptimer_set_freq(s->ptimer, 8192);
    ptimer_set_count(s->ptimer, 1);

    //Set limit to 1 and enable reload.
    ptimer_set_limit(s->ptimer, 1, 1);

    //Continuous mode
    s->timer_running = true;
    ptimer_run(s->ptimer, 0);

    RDP("%s", "timer running");
}

static void nrf51_radio_timer_stop(nrf51_radio_state * s)
{
    if (!s->timer_running)
        RDP("warning: %s", "timer was not running");

    s->timer_running = false;
    ptimer_stop(s->ptimer);

    RDP("%s", "timer stopped");
}

static void fire_event_disabled(nrf51_radio_state *s)
{
    s->radio_state = enmStateDisabled;
    s->REG.DISABLED = 1;
    nrf51_radio_pulse_event(s, RADIO_INTEN_DISABLED);
    nrf51_radio_timer_stop(s);
}

static const char * task_to_str(NRF51_RADIO_TASK task)
{
    switch(task)
    {
        case enmTaskNone: return "NONE";
        case enmTaskTxEn: return "TXEN";
        case enmTaskRxEn: return "RXEN";
        case enmTaskStart: return "START";
        case enmTaskStop: return "STOP";
        case enmTaskDisable: return "DISABLE";
        default:
            return "UNK";
    }
}

static const char * state_to_str(NRF51_RADIO_STATE state)
{
    switch (state)
    {
        case enmStateDisabled: return "DISABLED";
        case enmStateRxRu: return "RXRU";
        case enmStateRxIdle: return "RXIDLE";
        case enmStateRx: return "RX";
        case enmStateTxRu: return "TXRU";
        case enmStateTxIdle: return "TXIDLE";
        case enmStateTx: return "TX";
        case enmStateRxDisable: return "RXDISABLE";
        case enmStateTxDisable: return "TXDISABLE";
        default:
            return "UNK";
    }
}

static int nrf51_radio_udp_send(nrf51_radio_state * s, nrf51_air_packet_buff * pkt)
{
    int udp_sz, sent;
    udp_sz = udp_read_len(&pkt->proto_hdr) + PROTO_HDR_SZ;
    sent = nrf51_udp_send((uint8_t*)pkt, udp_sz);
    if (sent == -1)
    {
        RDP("%s", "unable to send udp packet");
        return -1;
    }

    return sent;
}

void nrf51_radio_udp_init(nrf51_radio_state *s)
{

}

static void nrf51_radio_pulse_event(const nrf51_radio_state * s, uint32_t mask)
{
    if (s->REG.INTEN & mask)
        qemu_irq_pulse(s->irq);
}

void nrf51_radio_do_rx(void * opaque, uint8_t * data, uint_fast16_t len)
{
    nrf51_radio_state * s = opaque; //TODO: null check?
    nrf51_air_packet * pkt = (nrf51_air_packet*) data;

    if ( len < (int) (PROTO_HDR_SZ + AIR_PACKET_HDR_SZ) )
    {
        RDP("%s", "header not received");
        //No data received.
        return;// false;
    }

    if (pkt->type != enmAirTypeData)
    {
        RDP("drop, unexpected packet type: %u", pkt->type);
        return;// false;
    }

    RDP("data received pay len: %u, packet len: %u", udp_read_len(&pkt->proto_hdr) - AIR_PACKET_HDR_SZ, len);

    if ( s->radio_task != enmTaskNone || s->radio_state != enmStateRx )
    {
        //Drop this packet. We don't expect any radio data.
        RDP("%s", "Not expecting radio packet, drop");
        return;
    }

    s->bPacketReceived = true;

    //FIXME: Calculate target data space.
    //FIXME: Truncate packet based on MAXLEN, SLEN, etc. params.
    if (NRF51_OUT_OF_RAM(s->ActiveConf.PACKETPTR, 4 /*FIXME: calculate me*/))
    {
        nrf51_trigger_hardfault();
        return;// false;
    }

    address_space_write(&address_space_memory, s->ActiveConf.PACKETPTR, MEMTXATTRS_UNSPECIFIED,
                        pkt->data_start, udp_read_len(&pkt->proto_hdr) - AIR_PACKET_HDR_SZ);

    s->REG.PAYLOAD = 1;
    RDP("%s", "event PAYLOAD");
    nrf51_radio_pulse_event(s, RADIO_INTEN_PAYLOAD);
    //FIXME: Do CRC operation. and IRQ?
    s->REG.CRCSTATUS = 1;

    s->REG.END = 1;
    RDP("%s", "event END");
    nrf51_radio_pulse_event(s, RADIO_INTEN_END);

    //Run the state machine. For immediate state transition.
    nrf51_radio_timer(s);
    //Keep the timer running.
    nrf51_radio_timer_start(s);

    //return true;
}

static void nrf51_radio_do_tx(nrf51_radio_state * s)
{
    unsigned int uHdrSize = 0;
    uint8_t achHdr[3];
    unsigned int uLength;
    unsigned int uPayLen = 0;
    unsigned int uRawSize;

    RDP("sending data pointed at 0x%x", s->ActiveConf.PACKETPTR);

    //This is either 1 byte or none.
    if(s->ActiveConf.PCNF0.S0LEN)
        uHdrSize++;

    //If LFLEN exists and its size is smaller than 8 bits
    //it will occupy 1 byte in memory.
    if (s->ActiveConf.PCNF0.LFLEN)
        uHdrSize++;

    //Same as LFLEN
    if(s->ActiveConf.PCNF0.S1LEN)
        uHdrSize++;

    if (uHdrSize > 0)
    {
        if (NRF51_OUT_OF_RAM(s->ActiveConf.PACKETPTR, uHdrSize))
        {
            nrf51_trigger_hardfault();
            return;
        }

        address_space_read(&address_space_memory, s->ActiveConf.PACKETPTR,
                           MEMTXATTRS_UNSPECIFIED, achHdr, (int) uHdrSize);
        if (s->ActiveConf.PCNF0.LFLEN)
        {
            if(s->ActiveConf.PCNF0.S0LEN)
                uLength = achHdr[1]; //Skip S0 and read length field
            else
                uLength = achHdr[0];
            //Do bit mask for configured number of bits.
            uLength &= (1 << s->ActiveConf.PCNF0.LFLEN) - 1;
            uPayLen += uLength;
        }
    }

    //Static length = number of bytes to transmit AFTER payload.
    uPayLen += s->ActiveConf.PCNF1.STATLEN;
    uRawSize = uHdrSize + uPayLen;

    //Raw packet size cannot exceed 254 bytes
    //Ref. manual, 17.1.3 Maximum packet length
    if (uRawSize > 254)
        uRawSize = 254;

    RDP("raw size: %d", uPayLen);
    RDP("pkt header size: %u", uHdrSize);
    RDP("payload size: %d", uPayLen);

    if (uRawSize == 254)
    {
        //It might be required to adjust maximum payload size.
        uPayLen = uRawSize - uHdrSize;
    }

    //TODO: Not clear if MAXLEN limits the payload size or total packet size. Assuming payload size.
    if (uPayLen > s->ActiveConf.PCNF1.MAXLEN)
        uPayLen = s->ActiveConf.PCNF1.MAXLEN;

    RDP("adjusted payload size: %d", uPayLen);

    if (uPayLen)
    {
        uint8_t payload[uPayLen];
        int i;
        address_space_read(&address_space_memory, s->ActiveConf.PACKETPTR + uHdrSize,
                           MEMTXATTRS_UNSPECIFIED, payload, (int) uPayLen);
        RDP("%s", "payload data: ");
        for (i = 0; i < uPayLen; i++)
        {
            printf("%02x ", payload[i]);
        }
        printf("\n");

        //build packet for udp layer
        air_packet.type = enmAirTypeData;
        air_packet.mode = s->ActiveConf.uMode;
        udp_set_len(&air_packet.proto_hdr, AIR_PACKET_HDR_SZ + uPayLen);
        assert(uPayLen <= PBUF_SZ);
        memcpy(air_packet.buffer, payload , uPayLen);

        nrf51_radio_udp_send(s, &air_packet);
    }

    s->REG.PAYLOAD = 1;
    RDP("%s", "event PAYLOAD");
    nrf51_radio_pulse_event(s, RADIO_INTEN_PAYLOAD);

    //FIXME: Do CRC operation.
    s->REG.END = 1;
    RDP("%s", "event END");
    nrf51_radio_pulse_event(s, RADIO_INTEN_END);
}

void nrf51_radio_timer(void *opaque)
{
    nrf51_radio_state * const s = opaque;

    if (!s->REG.POWER)
    {
        RDP("%s", "powered off, stop timer.");
        if (s->timer_running)
            nrf51_radio_timer_stop(s);
        return;
    }

    if (s->radio_state != enmStateRx)
        RDP("current state: %s, task: %s", state_to_str(s->radio_state), task_to_str(s->radio_task));

    /* Implemented according to ref. manual v3.0 Figure 22: Radio states */
    /* RADIO does not prevent a task being triggered from wrong state (ref. manual 17.1.8 Radio States, p. 84) */
    switch (s->radio_state)
    {
        /* Disabled */
        case enmStateDisabled:
        {
            switch (s->radio_task)
            {
                case enmTaskTxEn:
                    s->radio_state = enmStateTxRu;
                    s->radio_task = enmTaskNone;
                    break;

                case enmTaskRxEn:
                    s->radio_state = enmStateRxRu;
                    s->radio_task = enmTaskNone;
                    break;

                default:
                    PRINT_INCORRECT_STATE();
                    nrf51_radio_timer_stop(s);
                    break;
            }
        }
        break;

        /* TX Ramp-Up */
        case enmStateTxRu:
        {
            switch (s->radio_task)
            {
                case enmTaskNone:
                    s->radio_state = enmStateTxIdle;
                    s->REG.READY = 1;
                    RDP("%s", "event READY");
                    nrf51_radio_pulse_event(s, RADIO_INTEN_READY);
                    break;

                case enmTaskDisable:
                    s->radio_task = enmTaskNone; //Finish task
                    s->radio_state = enmStateTxDisable; //New state
                    break;

                case enmTaskTxEn:
                    //This behavior is not correct but let's handle it.
                    //TXEN task triggered, switch to none.
                    //So it can switch to idle state.
                    s->radio_task = enmTaskNone;
                    break;

                default:
                    PRINT_INCORRECT_STATE();
                    nrf51_radio_timer_stop(s);
                    break;
            }
        }
        break;

        /* TX Idle */
        case enmStateTxIdle:
        {
            switch (s->radio_task)
            {
                case enmTaskNone:
                {
                    //Stop and wait for a task.
                    nrf51_radio_timer_stop(s);
                    break;
                }

                case enmTaskStart:
                {
                    s->radio_task = enmTaskNone;
                    s->radio_state = enmStateTx;
                    break;
                }

                case enmTaskDisable:
                {
                    s->radio_task = enmTaskNone;
                    s->radio_state = enmStateTxDisable;
                    break;
                }

                default:
                    PRINT_INCORRECT_STATE();
                    nrf51_radio_timer_stop(s);
                    break;
            }
        }
        break;

        /* TX */
        case enmStateTx:
        {
            switch(s->radio_task)
            {
                case enmTaskNone:
                {
                    nrf51_radio_do_tx(s);
                    s->radio_state = enmStateTxIdle;
                    break;
                }

                default:
                    PRINT_INCORRECT_STATE();
                    nrf51_radio_timer_stop(s);
                    break;
            }
        }
        break;

        /* RX Ramp-Up */
        case enmStateRxRu:
        {
            switch (s->radio_task)
            {
                case enmTaskNone:
                    s->radio_state = enmStateRxIdle;
                    s->REG.READY = 1;
                    RDP("%s", "event (rx) READY");
                    nrf51_radio_pulse_event(s, RADIO_INTEN_READY);
                    break;

                case enmTaskDisable:
                    s->radio_task = enmTaskNone; //Finish task
                    s->radio_state = enmStateRxDisable; //New state
                    break;

                case enmTaskRxEn:
                    //This behavior is not correct but let's handle it.
                    //RXEN task triggered, switch to none.
                    //So it can switch to idle state.
                    s->radio_task = enmTaskNone;
                    break;

                default:
                    PRINT_INCORRECT_STATE();
                    nrf51_radio_timer_stop(s);
                    break;
            }
        }
        break;

        /* RX Idle */
        case enmStateRxIdle:
        {
            switch (s->radio_task)
            {
                case enmTaskNone:
                {
                    //Stop and wait for a task.
                    nrf51_radio_timer_stop(s);
                    break;
                }

                case enmTaskStart:
                {
                    s->radio_task = enmTaskNone;
                    s->radio_state = enmStateRx;
                    //Eat up all the bytes from UDP socket if there are any.
                    //New data must come during RX state.

                    //Temporarily disable, may not be needed anymore.
                    //nrf51_radio_udp_consume(s);
                    break;
                }

                case enmTaskDisable:
                {
                    s->radio_task = enmTaskNone;
                    s->radio_state = enmStateRxDisable;
                    break;
                }

                default:
                    PRINT_INCORRECT_STATE();
                    nrf51_radio_timer_stop(s);
                    break;
            }
        }
        break;

        /* RX */
        case enmStateRx:
        {
            switch(s->radio_task)
            {
                case enmTaskNone:
                {
                    //Stay in RX state until packet is received
                    if (s->bPacketReceived)
                    {
                        s->radio_state = enmStateRxIdle;
                        s->bPacketReceived = false;
                    }
                    else
                    {
                        nrf51_radio_timer_stop(s);
                    }
                    break;
                }

                default:
                    PRINT_INCORRECT_STATE();
                    nrf51_radio_timer_stop(s);
                    break;
            }
        }
        break;

        /* Rx/Tx Disable */
        case enmStateRxDisable:
        case enmStateTxDisable:
        {
            switch (s->radio_task)
            {
                case enmTaskNone:
                    fire_event_disabled(s);
                    break;

                default:
                    PRINT_INCORRECT_STATE();
                    nrf51_radio_timer_stop(s);
                    break;
            }
        }
        break;

        //Shouldn't reach here
        default:
            RDP("unknown radio state: %u (task: %u)",
                s->radio_state, s->radio_task);
            nrf51_radio_timer_stop(s);
            break;
    }

    if (s->radio_state != enmStateRx)
        RDP("   last state: %s, task: %s\n", state_to_str(s->radio_state), task_to_str(s->radio_task));
}
