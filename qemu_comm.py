#!/usr/bin/env python3

import socket, sys
import re

from struct import unpack

'''
protocol_header
{
    uint8 pkt_type;
    uint8 reserved;
}
'''

PROTO_HDR_SZ = 4

enmMsgTypeSendId = 0x0
enmMsgTypeRadio = 0x1
enmMsgTypeGPIO = 0x2

enmRadioTypeData = 0x1
RADIO_HEADER_SZ = 2

REGEX_GPIO = r"\( *(0x[a-z0-9]{1,4}) *, *([0-9]{1,2}) *\)"

# each pin is mapped with an ID (two ways)
# example:
# (0xbeef, 4) -> (0xface, 7)
# (0xface, 7) -> (0xbeef, 4)
gpio_map = {}

c2id = {} #client to id
id2c = {} #id to client

local_ip = "127.0.0.1"
local_port = 5151

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind ((local_ip, local_port))

HEX = [ chr(ord('0')+i) for i in range(0, 10) ] + [ chr(ord('a')+i) for i in range(0, 6) ]
asciibuf = bytearray(b' ' * 16)

cfg_parsers = {}

def hex_2d(c):
    return (HEX[c>>4] + HEX[c&0xf])

def parse_gpio(value):
    it = re.finditer(REGEX_GPIO, value, re.IGNORECASE)
    n = 0
    tuples = []

    # parse cfg line
    for m in it:
        if (n > 1):
            print("cannot parse gpio config value:", value)
            return False
        # t <- (dev_id, pin)
        t = (int(m.group(1), 0), int(m.group(2)))
        tuples.append(t)
        n += 1

    #map them to each other
    gpio_map[tuples[0]] = tuples[1]
    gpio_map[tuples[1]] = tuples[0]

    return True

def read_config():
    with open("nrf51.cfg", "r") as cfg:
        for line in cfg:
            line = line.strip()
            if len(line) < 1: continue
            if line[0] == '#': continue
            keyend = line.find(" ")
            if keyend > 0:
                key = line[:keyend]
                cfg_parsers[key](line[keyend+1:])

def hexdump(data):
    p = 0
    for i in range(0, len(data)):
        c = data[i]
        sys.stdout.write(hex_2d(c))
        tbits = i & 0xf
        nl = tbits == 0xf or tbits == 0x7
        if (c >=32 and c <= 126):
            asciibuf[p] = c
        else:
            asciibuf[p] = ord('.')
        p += 2
        if (nl):
            p = 0
            sys.stdout.write(' | ')
            sys.stdout.write(asciibuf.decode('ascii'))
            sys.stdout.write('\n')
        else:
            sys.stdout.write(' ')

    if (not nl):
        sys.stdout.write(' ' * (3*(7 - i % 8)))
        sys.stdout.write('| ')
        for i in range(p, 16, 2):
            asciibuf[i] = 0x20
        sys.stdout.write(asciibuf.decode('ascii'))
        print()

def handler_send_id(sender, pkt):
    pkt = pkt[PROTO_HDR_SZ:]
    if len(pkt) != 2:
        print("id packet corrupted")
        return False
    id, = unpack('!H', pkt)

    if id not in id2c:
        print("new client seen:", sender)
    else:
        old_client = id2c[id]
        #c2id.pop(old_client, None) #TODO: enable, disabled for debugging
        print("replace {0} with".format(old_client), sender)


    print("Client ID:", hex(id))
    id2c[id] = sender #add or replace client with this id
    c2id[sender] = id

    return True

def handler_gpio(sender, pkt):
    if len(pkt) < PROTO_HDR_SZ + 1:
        print("Packet corrupt")
        return False

    #find dev id of this sender
    dev_sender_id = c2id.get(sender)
    if dev_sender_id is None:
        print("sender not found:", sender)
        return True
    msg = pkt[PROTO_HDR_SZ]
    pin = msg & 0x1F
    state = msg >> 7
    #find receiver
    t = (dev_sender_id, pin)
    dev_recv_tuple = gpio_map.get(t)
    if dev_recv_tuple is None:
        print("No target found for this configuration:({:#x}, {})".format(dev_sender_id, pin))
        return True
    dev_recv_id, recv_pin = dev_recv_tuple
    receiver = id2c.get(dev_recv_id)
    if receiver is None:
        print("Client with this id is not connected:", hex(dev_recv_id))
        return True

    info = "({}) pin: {} state: {}".format(hex(dev_sender_id), pin, state)

    #replace pin and route it to the receiver
    hdr = bytearray(pkt)
    hdr[PROTO_HDR_SZ] = (state << 7 | recv_pin)
    sock.sendto(hdr, receiver)
    print(info)
    return True

def handler_radio(sender, pkt):
    if handler_radio_helper(sender, data[PROTO_HDR_SZ:]):
        #iterate over active clients
        for c in c2id:
            if (c != sender):
                print("sending to:", c)
                sock.sendto(data, c)

    #We want this packet to be printed
    return False

def handler_radio_helper(sender, pkt):
    if len(pkt) < RADIO_HEADER_SZ:
        print("<packet too small>")
        return False

    ptype, mode = unpack('!BB', pkt[0:RADIO_HEADER_SZ])
    print("mode:", mode)

    if ptype == enmRadioTypeData:
        return True

    print("<unknown packet type>")
    return False

# Order is important as index is read from protocol data.
PktHandlers = [handler_send_id, handler_radio, handler_gpio] #, handler_uart]
cfg_parsers["gpio"] = parse_gpio

while True:

    read_config()

    data, sender  = sock.recvfrom(1024)

    if len(data) < PROTO_HDR_SZ + 1:
        print("Packet too small, drop")
        continue

    hidx, resvd, size = unpack('!BBH', data[:PROTO_HDR_SZ])

    if len(data) != size + PROTO_HDR_SZ:
        print("Packet size does not match")
        continue

    if resvd != 0:
        print("reserved is non-zero, drop")
        continue

    if hidx >= len(PktHandlers):
        print("Unknown packet type")
        continue

    pkt_handler = PktHandlers[hidx]
    if not pkt_handler(sender, data):
        print()
        print(str(sender) + ":")
        hexdump(data)



