"""
Utilities for the wolf firmware

Jens Dede <jd@comnets.uni-bremen.de>
"""

import sys
import os
import utime as time
import machine
import ubinascii
import uhashlib
import uasyncio

from config import nodes_config

def mac2eui(mac):
    mac = mac[0:6] + 'fffe' + mac[6:]
    return hex(int(mac[0:2], 16) ^ 2)[2:] + mac[2:]

def get_millis():
    millisecond = time.ticks_ms()
    return millisecond

def get_nodename():
    uuid = ubinascii.hexlify(machine.unique_id()).decode()
    node_name = "ESP_" + uuid
    return node_name

def get_node_id(hex=False):
    node_id = ubinascii.hexlify(uhashlib.sha1(machine.unique_id()).digest()).decode("utf-8")[-8:] # 4 bytes unsigned int
    if hex:
        return node_id
    else:
        return int(node_id, 16)

def get_this_config():
    myId = str(get_node_id())
    if myId in nodes_config:
        return nodes_config[myId]
    return None

async def blink(pin, freq, duration):
    on_off_time = int(1.0/freq/2.0*1000.0)
    end_time = get_millis() + duration
    while(end_time > get_millis()):
        await uasyncio.sleep_ms(on_off_time)
        pin.value(0)
        await uasyncio.sleep_ms(on_off_time)
        pin.value(1)

async def actor_on(pin, duration):
    pin.value(0)
    await uasyncio.sleep_ms(duration)
    pin.value(1)
