import packets
from nodetype import NodeTypes

# Copyright 2020 LeMaRiva|tech lemariva.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#


# Heltec LoRa module
device_config = {
    'miso': 11,
    'mosi': 10,
    'ss': 8,
    'sck': 9,
    'dio_0': 14,
    'reset': 12,
    'led': 35,
}

app_config = {
    "ACK_RETRIES": 3,
    # ms (RTT was in tests aroun 270 - 290 ms -> little bit of margin)
    "ACK_TIMEOUT": 400,
    "DEBOUNCE_TIME": 100,  # ms
    "SHUTDOWN_DISPLAY_AFTER": 120,  # s
}

lora_parameters = {
    'frequency': 868.0,
    # 'tx_power_level': 2,
    'signal_bandwidth': 125.0,
    'spreading_factor': 7,
    'coding_rate': 5,
    'preamble_length': 8,
    'implicit_header': False,
    'sync_word': 0x12,  # 0x12 = private, 0x34 = public
    'enable_CRC': False,
    'invert_IQ': False,
}

wifi_config = {
    'ssid': 'FRITZ!Box 7530 KF',
    'password': '56114591079794255952'
}

# Network wide encryption parameters
encrypt_config = {
    "aes_key": b"ausme8Sdk29dswapausme8Sdk29dswab"
}


# General parameters for the experiments. Used by the actuator box
node_params = {
    NodeTypes.FLASH: {
        "frequency": 20,  # Hz
        "duration": 5000,  # ms
        "block_time": 10000,  # ms
    },
    NodeTypes.ULTRASOUND_CANNON: {
        "duration": 22000,  # ms
        "block_time": 30000,  # ms
    }

}

# Per node config parameters, stored by node id
nodes_config = {

    ### start of test nodes ###
    "708416441": {  # PIR sensor 1, 07.2023
        "is_sender": True,
        "actor_node": None,  # Either address or none to Broadcast
        "action_cancel_previous": False,
        "protection_time": 1000,  # 1000 ms = 1 sec
        "beacon_interval": 120,  # seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "max17043",
        "gpio_button_irq": 13,  # IRQ on pin 13
        "use_ack": False,  # Use acks
    },

    #### end of test nodes ####

    # Start SET 1
    "4172078668": {  # PIR sensor 1, 07.2023
        "is_sender": True,
        "actor_node": 106440645,  # Either address or none to Broadcast
        "action_cancel_previous": False,
        "protection_time": 1000,  # 1000 ms = 1 sec
        "beacon_interval": 120,  # seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "max17043",
        "gpio_button_irq": 13,  # IRQ on pin 13
        "use_ack": True,  # Use acks
    },
    "2583535806": {  # PIR sensor 2, 07.2023, schwarz
        "is_sender": True,
        "actor_node": 106440645,  # Either address or none to Broadcast
        "action_cancel_previous": False,
        "protection_time": 1000,  # 1000 ms = 1 sec
        "beacon_interval": 120,  # seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "max17043",
        "gpio_button_irq": 13,  # IRQ on pin 13
        "use_ack": True,  # Use acks
    },
    "106440645": {  # General actor, 2023-07-12
        "is_sender": False,
        # "receiver_type": NodeTypes.ULTRASOUND_CANNON,
        "receiver_type": NodeTypes.FLASH,
        "beacon_interval": 120,  # 120 seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "analog",  # Read battery from Pin 36
        "gpio_led_status": 13,  # Status LED on pin 13
    },

    # END SET 1


    # START SET 2

    "2308316059": {  # General actor, 2023-04
        "is_sender": False,
        "receiver_type": NodeTypes.ULTRASOUND_CANNON,
        # "receiver_type": NodeTypes.FLASH,
        "beacon_interval": 120,  # 120 seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "analog",  # Read battery from Pin 36
        "gpio_led_status": 13,  # Status LED on pin 13
    },
    "2424541685": {  # PIR sensor 3, 07.2023, schwarz
        "is_sender": True,
        "actor_node": 2308316059,  # Either address or none to Broadcast
        "action_cancel_previous": False,
        "protection_time": 1000,  # 1000 ms = 1 sec
        "beacon_interval": 120,  # seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "max17043",
        "gpio_button_irq": 13,  # IRQ on pin 13
        "use_ack": True,  # Use acks
    },

    "3282280225": {  # PIR sensor 4, 07.2023, schwarz
        "is_sender": True,
        "actor_node": 2308316059,  # Either address or none to Broadcast
        "action_cancel_previous": False,
        "protection_time": 1000,  # 1000 ms = 1 sec
        "beacon_interval": 120,  # seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "max17043",
        "gpio_button_irq": 13,  # IRQ on pin 13
        "use_ack": True,  # Use acks
    },

    # END SET 2

    # START SET 3
    # Replaced ESP32 and changed ID from 951712297 -> 979909243

    "979909243": {  # Universal Actor
        "is_sender": False,
        "receiver_type": NodeTypes.ULTRASOUND_CANNON,
        # "receiver_type": NodeTypes.FLASH,
        "beacon_interval": 120,  # 120 seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "analog",  # Read battery from Pin 36
        "gpio_led_status": 13,  # Status LED on pin 13
    },
    "3902023872": {  # PIR sensor 5, 07.2023, schwarz, new type
        "is_sender": True,
        "actor_node": 979909243,  # Either address or none to Broadcast
        "action_cancel_previous": False,
        "protection_time": 1000,  # 1000 ms = 1 sec
        "beacon_interval": 120,  # seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "max17043",
        "gpio_button_irq": 13,  # IRQ on pin 13
        "use_ack": True,  # Use acks
    },
    "3454793720": {  # PIR sensor 6, 07.2023, schwarz, new type
        "is_sender": True,
        "actor_node": 979909243,  # Either address or none to Broadcast
        "action_cancel_previous": False,
        "protection_time": 1000,  # 1000 ms = 1 sec
        "beacon_interval": 120,  # seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "max17043",
        "gpio_button_irq": 13,  # IRQ on pin 13
        "use_ack": True,  # Use acks
    },

    # END SET 3

    "1445422710": {  # Blank node for testing
        "is_sender": True,
        "actor_node": 320270687,  # Either address or none to Broadcast
        "action_cancel_previous": False,
        "protection_time": 1000,  # 1000 ms = 1 sec
        "beacon_interval": 120,  # seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "max17043",
        "gpio_button_irq": 13,  # IRQ on pin 13
        "use_ack": True,  # Use acks
    },


    "320270687": {  # General Actor, problem?
        "is_sender": False,
        "receiver_type": NodeTypes.ULTRASOUND_CANNON,
        # "receiver_type": NodeTypes.FLASH,
        "beacon_interval": 120,  # 120 seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "analog",  # Read battery from Pin 36
        "gpio_led_status": 13,  # Status LED on pin 13
    },


    "2276286798": {  # PIR sensor
        "is_sender": True,
        "actor_node": None,  # Either address or none to Broadcast
        "msg_type": NodeTypes.FLASH,
        "action_frequency": 20,  # Hz
        "action_duration": 4000,  # 4000 ms = 4 sec
        "action_cancel_previous": False,
        "protection_time": 60000,  # 60000 ms = 60 sec
        "beacon_interval": 240,  # seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "max17043",
        "gpio_button_irq": 13,  # IRQ on pin 13
    },




    "4291548233": {  # Flash actor
        "is_sender": False,
        "receiver_type": NodeTypes.FLASH,
        "beacon_interval": 240,  # seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "analog",  # Read battery from Pin 36, hardcoded
    },
    "1892008357": {  # Sniffer
        "is_sender": False,
        "is_sniffer": True,
    },
    "510366793": {  # Buzzer
        "is_sender": True,
        "msg_type": NodeTypes.FLASH,
        "is_sniffer": False,
        "battery_type": "max17043",
        "actor_node": None,
        "action_cancel_previous": True,
        "action_frequency": 25,  # Hz (if applicable)
        "action_duration": 10000,  # 10000 ms = 10 sec
        "protection_time": 1000,  # 1000 ms = 1 sec
        "beacon_interval": 240,  # 120 seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "gpio_button_irq": 13,  # IRQ on pin 13
    },

    "2628889781": {  # Buzzer 2 2023-04
        "is_sender": True,
        "msg_type": NodeTypes.FLASH,
        "is_sniffer": False,
        "battery_type": "max17043",
        "actor_node": None,
        "action_cancel_previous": True,
        "action_frequency": 20,  # Hz (if applicable)
        "action_duration": 10000,  # 10000 ms = 10 sec
        "protection_time": 1000,  # 1000 ms = 1 sec
        "beacon_interval": 240,  # 120 seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "gpio_button_irq": 13,  # IRQ on pin 13
    },
    "1054058": {  # Ultrasonic actor
        "is_sender": False,
        "receiver_type": NodeTypes.ULTRASOUND_CANNON,
        "beacon_interval": 120,  # 120 seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "analog",  # Read battery from Pin 36
        "action_cancel_previous": True,
        "action_duration": 10000,  # 10000 ms = 10 sec
    },
    "3567154165": {  # Ultrasonic actor, 2023-04
        "is_sender": False,
        "receiver_type": NodeTypes.ULTRASOUND_CANNON,
        "beacon_interval": 120,  # 120 seconds
        "beacon_jitter": 10,    # seconds, will vary the above value by +- 10 seconds
        "battery_type": "analog",  # Read battery from Pin 36
        "action_cancel_previous": True,
        "action_duration": 10000,  # 10000 ms = 10 sec
        "gpio_led_status": 13,  # Status LED on pin 13
    },
}
