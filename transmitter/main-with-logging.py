"""
Main wolf application

Jens Dede <jd@comnets.uni-bremen.de>

"""
import micropython
import gc
gc.enable()
import sys
from lib.utils import get_nodename, get_node_id, get_this_config, get_millis, blink, actor_on
import network
from machine import Pin, SPI, I2C, Timer
import machine
import utime as time
import ubinascii
from config import encrypt_config, lora_parameters, device_config, node_params, app_config
import datahandler
import batteryhandler

# from sx127x import SX127x
from sx1262 import SX1262

import ssd1306

import uasyncio

from nodetype import NodeTypes as NT

import random
from lib.varlogger import VarLogger as vl
#vl.log(var='0', fun=_fun_name, clas=_cls_name, th=_thread_id)     ### standard logging format
import utime
import _thread


gc.collect()
gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
print('Free memory 1:', gc.mem_free())

def irq_handler(pin):
    _thread_id = _thread.get_ident()
    _fun_name = 'irq_handler'
    _cls_name = '0'

    global irq_triggered, irq_debounce_timer
    if pin.value() and irq_debounce_timer + app_config["DEBOUNCE_TIME"] < get_millis():
        irq_triggered = True
        vl.log(var='irq_triggered', fun=_fun_name, clas=_cls_name, th=_thread_id)
        irq_debounce_timer = get_millis()
        vl.log(var='irq_debounce_timer', fun=_fun_name, clas=_cls_name, th=_thread_id)

def flash_timer_handler(timer):
    _thread_id = _thread.get_ident()
    _fun_name = 'flash_timer_handler'
    _cls_name = '0'

    global actor_pin, actor_timer_timeout, actor_timer
    if actor_timer_timeout < get_millis():
        timer.deinit()
        actor_pin.value(1)
        signal_status(0)
        actor_timer = None
        vl.log(var='actor_timer', fun=_fun_name, clas=_cls_name, th=_thread_id)
    else:
        actor_pin.value(not actor_pin.value())

def ultrasonic_timer_handler(timer):
    _thread_id = _thread.get_ident()
    _fun_name = 'ultrasonic_timer_handler'
    _cls_name = '0'

    global actor_pin, actor_timer_timeout, actor_timer
    timer.deinit()
    actor_pin.value(1)
    signal_status(0)
    actor_timer = None
    vl.log(var='actor_timer', fun=_fun_name, clas=_cls_name, th=_thread_id)


def button_handler(pin):
    _thread_id = _thread.get_ident()
    _fun_name = 'button_handler'
    _cls_name = '0'

    global display_off_time
    display_off_time = time.time() + app_config["SHUTDOWN_DISPLAY_AFTER"]
    vl.log(var='display_off_time', fun=_fun_name, clas=_cls_name, th=_thread_id)

def signal_status(onoff):
    _thread_id = _thread.get_ident()
    _fun_name = 'signal_status'
    _cls_name = '0'

    vl.log(var='0', fun=_fun_name, clas=_cls_name, th=_thread_id)
    global status_led
    if status_led is not None:
        print("New LED status", onoff)
        status_led.value(onoff)

def cb_lora(events):
    _thread_id = _thread.get_ident()
    _fun_name = 'cb_lora'
    _cls_name = '0'

    if events & SX1262.RX_DONE:
        global lora_done
        lora_done = True
        vl.log(var='lora_done', fun=_fun_name, clas=_cls_name, th=_thread_id)

gc.collect()

_thread_id = _thread.get_ident()
_fun_name = '0'
_cls_name = '0'

vl.thread_status(_thread_id, 'active')     #### MUST

reset_causes = {
    machine.PWRON_RESET : "Power on reset",
    machine.HARD_RESET : "Hard reset",
    machine.WDT_RESET : "Watchdog reset",
    machine.DEEPSLEEP_RESET : "Deepsleep reset",
    machine.SOFT_RESET : "Soft reset",
}
vl.log(var='reset_causes', fun=_fun_name, clas=_cls_name, th=_thread_id)

# # Make sure network is really down
# sta_if = network.WLAN(network.STA_IF)
# sta_if.active(False)
# ap_if = network.WLAN(network.AP_IF)
# ap_if.active(False)

irq_triggered = False
vl.log(var='irq_triggered', fun=_fun_name, clas=_cls_name, th=_thread_id)
irq_debounce_timer = get_millis()
vl.log(var='irq_debounce_timer', fun=_fun_name, clas=_cls_name, th=_thread_id)
irq_last_trigger = get_millis()
vl.log(var='irq_last_trigger', fun=_fun_name, clas=_cls_name, th=_thread_id)

display_off_time = time.time() + app_config["SHUTDOWN_DISPLAY_AFTER"]
vl.log(var='display_off_time', fun=_fun_name, clas=_cls_name, th=_thread_id)
display_is_on = True
vl.log(var='display_is_on', fun=_fun_name, clas=_cls_name, th=_thread_id)

actor_timer = None
vl.log(var='actor_timer', fun=_fun_name, clas=_cls_name, th=_thread_id)
actor_timer_channel = None
vl.log(var='actor_timer_channel', fun=_fun_name, clas=_cls_name, th=_thread_id)
actor_timer_timeout = 0
vl.log(var='actor_timer_timeout', fun=_fun_name, clas=_cls_name, th=_thread_id)

num_received_packets = 0
vl.log(var='num_received_packets', fun=_fun_name, clas=_cls_name, th=_thread_id)

### global variable for lora receiver
lora_done = False
vl.log(var='num_received_packets', fun=_fun_name, clas=_cls_name, th=_thread_id)

gc.collect()
print('Free memory 2:', gc.mem_free())


print("Reset cause: ", reset_causes.get(machine.reset_cause(), "Unknown reset"))

# Heltec LoRa 32 with OLED Display
oled_width = 128
vl.log(var='oled_width', fun=_fun_name, clas=_cls_name, th=_thread_id)
oled_height = 64
vl.log(var='oled_height', fun=_fun_name, clas=_cls_name, th=_thread_id)
# OLED reset pin
i2c_rst = Pin(21, Pin.OUT)
vl.log(var='i2c_rst', fun=_fun_name, clas=_cls_name, th=_thread_id)
# Initialize the OLED display
i2c_rst.value(0)
time.sleep_ms(5)
i2c_rst.value(1) # must be held high after initialization
# Setup the I2C lines
i2c_scl = Pin(18, Pin.OUT, Pin.PULL_UP)
vl.log(var='i2c_scl', fun=_fun_name, clas=_cls_name, th=_thread_id)
i2c_sda = Pin(17, Pin.OUT, Pin.PULL_UP)
vl.log(var='i2c_sda', fun=_fun_name, clas=_cls_name, th=_thread_id)
# Create the bus object
i2c = I2C(scl=i2c_scl, sda=i2c_sda)
vl.log(var='i2c', fun=_fun_name, clas=_cls_name, th=_thread_id)
# Create the display object
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
vl.log(var='oled', fun=_fun_name, clas=_cls_name, th=_thread_id)
oled.fill(0)

#oled.line(0, 0, 50, 25, 1)
oled.show()

# device_spi = SPI(baudrate = 10000000,
#         polarity = 0, phase = 0, bits = 8, firstbit = SPI.MSB,
#         sck = Pin(device_config['sck'], Pin.OUT, Pin.PULL_DOWN),
#         mosi = Pin(device_config['mosi'], Pin.OUT, Pin.PULL_UP),
#         miso = Pin(device_config['miso'], Pin.IN, Pin.PULL_UP))

lora = None
vl.log(var='lora', fun=_fun_name, clas=_cls_name, th=_thread_id)
try:
    lora = SX1262(spi_bus=1, clk=9, mosi=10, miso=11, cs=8, irq=14, rst=12, gpio=13)
    vl.log(var='lora', fun=_fun_name, clas=_cls_name, th=_thread_id)
    lora.begin(freq=868.0, bw=125.0, sf=7, cr=5, syncWord=0x12, power=14, currentLimit=60.0,
preambleLength=8, implicit=False, implicitLen=0xFF, crcOn=False, txIq=False, rxIq=False,
tcxoVoltage=1.6, useRegulatorLDO=False, blocking=False)
    lora.setBlockingCallback(False, cb_lora)

except Exception as e:
    print("Error init LoRa radio. Restart and try again.")
    print("Will restart in 10 seconds.")
    print(e)
    time.sleep(10)
    machine.reset()

actor_pin = Pin(12, Pin.OUT)
vl.log(var='actor_pin', fun=_fun_name, clas=_cls_name, th=_thread_id)
actor_pin.value(1)

# On Board LED
led = Pin(35, Pin.OUT)
vl.log(var='led', fun=_fun_name, clas=_cls_name, th=_thread_id)
led.value(0)

# On Board button
button = Pin(0, Pin.IN)
vl.log(var='button', fun=_fun_name, clas=_cls_name, th=_thread_id)
button.irq(button_handler)

print("This is node ID:  " + str(get_node_id()))
print("This is node HEX: " + str(get_node_id(True)))

dh = datahandler.DataHandler(encrypt_config["aes_key"])
vl.log(var='dh', fun=_fun_name, clas=_cls_name, th=_thread_id)

nodeCfg = get_this_config()
vl.log(var='nodeCfg', fun=_fun_name, clas=_cls_name, th=_thread_id)

if nodeCfg is None:
    print("Node is not in the config file. Please set it up, upload the new config and try again.")
    print("")
    sys.exit()

irq_pin = None
vl.log(var='irq_pin', fun=_fun_name, clas=_cls_name, th=_thread_id)
if "gpio_button_irq" in nodeCfg:
    irq_pin = Pin(nodeCfg["gpio_button_irq"], Pin.IN, Pin.PULL_DOWN)
    vl.log(var='irq_pin', fun=_fun_name, clas=_cls_name, th=_thread_id)
    irq_pin.irq(irq_handler)

status_led = None
vl.log(var='status_led', fun=_fun_name, clas=_cls_name, th=_thread_id)
if "gpio_led_status" in nodeCfg:
    print("Using status LED on pin", nodeCfg["gpio_led_status"])
    status_led = Pin(nodeCfg["gpio_led_status"], Pin.OUT)
    vl.log(var='status_led', fun=_fun_name, clas=_cls_name, th=_thread_id)
    status_led.value(0)


if nodeCfg == None:
    print("Node not configured. Please update config.py")
    while(True):
        pass

bh = None
vl.log(var='bh', fun=_fun_name, clas=_cls_name, th=_thread_id)

if "battery_type" in nodeCfg:
    if nodeCfg["battery_type"] == "max17043":
        bh = batteryhandler.Max17043BatteryStatus(i2c)
        vl.log(var='bh', fun=_fun_name, clas=_cls_name, th=_thread_id)
    elif nodeCfg["battery_type"] == "analog":
        bh = batteryhandler.AnalogBatteryStatus()
        vl.log(var='bh', fun=_fun_name, clas=_cls_name, th=_thread_id)

# TODO: Disable Device if Battery is too low

oled_changed = True
vl.log(var='oled_changed', fun=_fun_name, clas=_cls_name, th=_thread_id)
pkg_sent = 0
vl.log(var='pkg_sent', fun=_fun_name, clas=_cls_name, th=_thread_id)

next_beacon_time = time.time()
vl.log(var='next_beacon_time', fun=_fun_name, clas=_cls_name, th=_thread_id)

node_config = None
vl.log(var='node_config', fun=_fun_name, clas=_cls_name, th=_thread_id)

if "receiver_type" in nodeCfg:
    node_config = node_params[nodeCfg["receiver_type"]]
    vl.log(var='node_config', fun=_fun_name, clas=_cls_name, th=_thread_id)
    print("Node config", node_config)


packets_waiting_ack = []
vl.log(var='packets_waiting_ack', fun=_fun_name, clas=_cls_name, th=_thread_id)
dedup_list = []
vl.log(var='dedup_list', fun=_fun_name, clas=_cls_name, th=_thread_id)

last_event_start = 0
vl.log(var='last_event_start', fun=_fun_name, clas=_cls_name, th=_thread_id)


while True:
    gc.collect()

    if display_is_on and display_off_time < time.time():
        oled.poweroff()
        display_is_on = False
        vl.log(var='display_is_on', fun=_fun_name, clas=_cls_name, th=_thread_id)
    elif not display_is_on and display_off_time > time.time():
        oled.poweron()
        display_is_on = True
        vl.log(var='display_is_on', fun=_fun_name, clas=_cls_name, th=_thread_id)
        oled_changed = True
        vl.log(var='oled_changed', fun=_fun_name, clas=_cls_name, th=_thread_id)

    oled.fill(0)
    oled.text("ID:  " + str(get_node_id()), 0, 0)
    oled.text("HEX: " + str(get_node_id(True)), 0,10)

    if irq_triggered:
        if get_millis() > irq_last_trigger + nodeCfg["protection_time"]:
            irq_last_trigger = get_millis()
            vl.log(var='irq_last_trigger', fun=_fun_name, clas=_cls_name, th=_thread_id)
            if nodeCfg and "is_sender" in nodeCfg and nodeCfg["is_sender"]:
                print("Sending alarm message")
                (bindata, seq, ack) = dh.sendEncActor(nodeCfg)
                vl.log(var='bindata', fun=_fun_name, clas=_cls_name, th=_thread_id)
                vl.log(var='seq', fun=_fun_name, clas=_cls_name, th=_thread_id)
                vl.log(var='ack', fun=_fun_name, clas=_cls_name, th=_thread_id)
                lora.send(bindata)
                if ack:
                    packets_waiting_ack.append((bindata, seq, get_millis(), 0))
                pkg_sent += 1
                vl.log(var='pkg_sent', fun=_fun_name, clas=_cls_name, th=_thread_id)

                oled.text("#TX: " + str(pkg_sent), 0, 55)
                oled_changed = True
                vl.log(var='oled_changed', fun=_fun_name, clas=_cls_name, th=_thread_id)
        else:
            print("Alarm was " + str((irq_last_trigger + nodeCfg["protection_time"] - get_millis()) / 1000.0) + "s too early")

        irq_triggered = False
        vl.log(var='irq_triggered', fun=_fun_name, clas=_cls_name, th=_thread_id)


    if time.time() > next_beacon_time and "beacon_interval" in nodeCfg:
        print("Sending out beacon")
        bindata = dh.sendEncBeacon()
        vl.log(var='bindata', fun=_fun_name, clas=_cls_name, th=_thread_id)
        if bh:
            bat_data = bh.do_read()
            vl.log(var='bat_data', fun=_fun_name, clas=_cls_name, th=_thread_id)
            if bat_data is not None:
                print("Attaching battery status to beacon:", bat_data)
                bat = (int(bat_data[0]), int(bat_data[1]*1000))
                vl.log(var='bat', fun=_fun_name, clas=_cls_name, th=_thread_id)
            else:
                print("No battery status information available. Sending None")
                bat =(None,None)
                vl.log(var='bat', fun=_fun_name, clas=_cls_name, th=_thread_id)
            bindata = dh.sendEncBeacon(bat=bat)
            vl.log(var='bindata', fun=_fun_name, clas=_cls_name, th=_thread_id)
        lora.send(bindata)
        jitter = 0
        vl.log(var='jitter', fun=_fun_name, clas=_cls_name, th=_thread_id)
        if "beacon_jitter" in nodeCfg:
            jitter = random.randint(-nodeCfg["beacon_jitter"], nodeCfg["beacon_jitter"])
            vl.log(var='jitter', fun=_fun_name, clas=_cls_name, th=_thread_id)

        next_beacon_time += nodeCfg["beacon_interval"] + jitter
        vl.log(var='next_beacon_time', fun=_fun_name, clas=_cls_name, th=_thread_id)

        print(time.time(), next_beacon_time)

    if lora_done:
        payload, err = lora.recv()
        vl.log(var='payload', fun=_fun_name, clas=_cls_name, th=_thread_id)
        #print(payload)
        packet = None
        vl.log(var='packet', fun=_fun_name, clas=_cls_name, th=_thread_id)
        is_sniffer = False
        vl.log(var='is_sniffer', fun=_fun_name, clas=_cls_name, th=_thread_id)

        if "is_sniffer" in nodeCfg:
            is_sniffer = nodeCfg["is_sniffer"]
            vl.log(var='is_sniffer', fun=_fun_name, clas=_cls_name, th=_thread_id)

        try:
            packet = dh.receiverEncPacket(bytearray(payload), is_sniffer)
            vl.log(var='packet', fun=_fun_name, clas=_cls_name, th=_thread_id)
        except:
            print("Packet parsing failed. Not for us?")
        if packet:
            # packet.set_rssi(lora.packet_rssi())
            # packet.set_snr(lora.packet_snr())
            print("Received:", packet)
            num_received_packets += 1
            vl.log(var='num_received_packets', fun=_fun_name, clas=_cls_name, th=_thread_id)

            print("Total received packets:", num_received_packets)

            if not is_sniffer:
                ack = packet.create_ack()
                vl.log(var='ack', fun=_fun_name, clas=_cls_name, th=_thread_id)

                if ack:
                    lora.send(dh.encrypt(ack))


                current_packet_is_dup = False
                vl.log(var='current_packet_is_dup', fun=_fun_name, clas=_cls_name, th=_thread_id)
                if packet.get_type() in (packet.TYPE_ACTOR_UNIVERSAL, ) and not nodeCfg["is_sender"]:
                    # remove duplicates
                    for item in dedup_list:
                        if item[0] == packet.get_sequence() and item[1] == packet.get_sender():
                            current_packet_is_dup = True
                            vl.log(var='current_packet_is_dup', fun=_fun_name, clas=_cls_name, th=_thread_id)
                            print("DUP")
                            
                    dedup_list.append((packet.get_sequence(), packet.get_sender(), get_millis()))



                if packet.get_type() in (packet.TYPE_ACTOR_UNIVERSAL, ) and not nodeCfg["is_sender"] and not current_packet_is_dup:
                    if (last_event_start + (node_config["block_time"] / 1000) > time.time()) and (last_event_start > 0):
                        print("Inside Block time. Skipping event.", last_event_start + (node_config["block_time"]/1000)-time.time())
                        continue

                    last_event_start = time.time()
                    vl.log(var='last_event_start', fun=_fun_name, clas=_cls_name, th=_thread_id)

                    if nodeCfg["receiver_type"] == NT.FLASH:
                        print("Acting as flash actor")

                        can_cancel = packet.get_params()[0]
                        vl.log(var='can_cancel', fun=_fun_name, clas=_cls_name, th=_thread_id)
                        duration = node_config["duration"]
                        vl.log(var='duration', fun=_fun_name, clas=_cls_name, th=_thread_id)
                        frequency = node_config["frequency"]
                        vl.log(var='frequency', fun=_fun_name, clas=_cls_name, th=_thread_id)


                        if actor_timer != None and can_cancel:
                            try:
                                actor_timer.deinit()
                                actor_timer = None
                                vl.log(var='actor_timer', fun=_fun_name, clas=_cls_name, th=_thread_id)
                                actor_pin.value(1)
                                signal_status(0)
                                print("Cancelled Timer")
                            except:
                                pass
                        else:
                            oled.text('f:' + str(frequency) + "Hz", 0, 35)
                            oled.text('d:' + str(round(duration/1000.0,2)) + "s", 0,45)
                            signal_status(1)

                            actor_timer = Timer(3)
                            vl.log(var='actor_timer', fun=_fun_name, clas=_cls_name, th=_thread_id)
                            actor_timer_timeout = get_millis() + duration
                            vl.log(var='actor_timer_timeout', fun=_fun_name, clas=_cls_name, th=_thread_id)
                            actor_timer.init(mode=Timer.PERIODIC, period=int(1.0/frequency/2.0*1000.0), callback=flash_timer_handler)

                    elif nodeCfg["receiver_type"] == NT.ULTRASOUND_CANNON:
                        print("Acting as ultrasound actor")
                        can_cancel = packet.get_params()[0]
                        vl.log(var='can_cancel', fun=_fun_name, clas=_cls_name, th=_thread_id)
                        duration = node_config["duration"]
                        vl.log(var='duration', fun=_fun_name, clas=_cls_name, th=_thread_id)

                        if actor_timer != None and can_cancel:
                            try:
                                actor_timer.deinit()
                                actor_timer = None
                                vl.log(var='actor_timer', fun=_fun_name, clas=_cls_name, th=_thread_id)
                                actor_pin.value(1)
                                signal_status(0)
                                print("Cancelled timer")
                            except:
                                pass
                        else:
                            oled.text('d:' + str(round(duration/1000.0,2)) + "s", 0,45)
                            actor_timer = Timer(3)
                            vl.log(var='actor_timer', fun=_fun_name, clas=_cls_name, th=_thread_id)
                            actor_timer_timeout = get_millis() + duration
                            vl.log(var='actor_timer_timeout', fun=_fun_name, clas=_cls_name, th=_thread_id)
                            actor_timer.init(mode=Timer.ONE_SHOT, period=duration, callback=ultrasonic_timer_handler)
                            actor_pin.value(0)
                            signal_status(1)
                    else:
                        print("Unhandled packet type")

                elif packet.get_type() == packet.TYPE_BEACON:
                    print("Received beacon from", packet.get_sender(), "with RSSI of ", packet.get_rssi(), "and SNR of", packet.get_snr())
                    print("Battery from beacon:", packet.getBattery())
                    oled.text('b:' + str(packet.get_sender()), 0, 35)
                    oled.text('r:' + str(packet.get_rssi()) + " s:" + str(packet.get_snr()), 0,45)
                    print("RSSI: "+str(packet.get_rssi()) + " SNR: " + str(packet.get_snr()))

                elif packet.get_type() == packet.TYPE_ACK:
                    # Remove acked packets from the list
                    print("Got ack from", packet.get_sender(), "with seq", packet.get_sequence())
                    reack = []
                    vl.log(var='reack', fun=_fun_name, clas=_cls_name, th=_thread_id)

                    for pkg in packets_waiting_ack:
                        if pkg[1] == packet.get_sequence():
                            print("Got ACK after " + str(get_millis() - pkg[2]) + " milliseconds")
                            continue
                        reack.append(pkg)
                    packets_waiting_ack = reack
                    vl.log(var='packets_waiting_ack', fun=_fun_name, clas=_cls_name, th=_thread_id)
                    del reack

            else:
                print("Sniffed_packet")
                print("!{"+str(ubinascii.hexlify(payload).decode())+"}#")
                oled.text('sp:' + str(num_received_packets), 0, 35)
            oled_changed = True
            vl.log(var='oled_changed', fun=_fun_name, clas=_cls_name, th=_thread_id)

    if oled_changed:
        oled_changed = False
        vl.log(var='oled_changed', fun=_fun_name, clas=_cls_name, th=_thread_id)
        oled.show()

    # Handle ACKs
    reack = []
    for pkg in packets_waiting_ack:
        # packets_waiting_ack.append((bindata, seq, time.time(), 0))
        pkg = list(pkg)
        vl.log(var='pkg', fun=_fun_name, clas=_cls_name, th=_thread_id)
        if (pkg[2] + app_config["ACK_TIMEOUT"]) < get_millis(): # retry
            print("Retry")
            pkg[3] = pkg[3] + 1
            vl.log(var='pkg', fun=_fun_name, clas=_cls_name, th=_thread_id)
            pkg[2] = get_millis()
            vl.log(var='pkg', fun=_fun_name, clas=_cls_name, th=_thread_id)
            pkg_sent += 1
            vl.log(var='pkg_sent', fun=_fun_name, clas=_cls_name, th=_thread_id)
            lora.send(pkg[0])

        if pkg[3] >= app_config["ACK_RETRIES"]:
            continue

        reack.append(pkg)

    packets_waiting_ack = reack
    vl.log(var='packets_waiting_ack', fun=_fun_name, clas=_cls_name, th=_thread_id)
    del reack


    # Tidy up dedup list
    dedup = []
    vl.log(var='dedup', fun=_fun_name, clas=_cls_name, th=_thread_id)
    for pkg in dedup_list:
        if pkg[-1] + (2*app_config["ACK_RETRIES"]*app_config["ACK_TIMEOUT"]) < get_millis():
            continue
        else:
            dedup.append(pkg)
    current_packet_is_dup = dedup
    vl.log(var='current_packet_is_dup', fun=_fun_name, clas=_cls_name, th=_thread_id)
    del dedup



#####################################################
############### main functionality ##################
#####################################################
# try:
#     ### initialize it outside the scope of any function to allow access to all the code
#     start_time = utime.ticks_ms()
#     #### get lock to regulate the access to shared resources
#     lock = _thread.allocate_lock()
#     #/// update the thread status
#     vl.thread_status('main', 'active') #//// update the thread status
#     main_thread = _thread.start_new_thread(main, ())

#     while True:
#         ### check if threads are running
#         ids, thread_info = vl.thread_status()
        
#         status = [thread_info[x] for x in ids]
#         #print(ids)
#         #print(status)
#         utime.sleep(2)

#         ### enter REPL if main thread of application is dead
#         if len(status) > 1:
#             if status[-1] == 'dead':
#                 #//// save the data
#                 vl.save()
#                 ### log the error message
#                 _thread.exit()
#                 # with open('log_check', 'rb') as f:
#                 #     f = f.readline()
#                 #     if f <= '2':
#                 #         machine.reset()
#                 #     else:
#                 #         _thread.exit()
               

# except Exception as e:
#     #//// save the data
#     vl.save() 
#     #/// log the traceback message
#     vl.traceback(e)
#     print('Error message:', e)
