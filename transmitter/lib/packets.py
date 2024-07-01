#!/usr/bin/env python3

"""
The WolfWare packet definitions

Jens Dede <jd@comnets.uni-bremen.de>
"""

import struct
import binascii
import random

# The base and most generic packet type
class BasePacket:
    TYPE_UNDEFINED  = 0x0
    TYPE_ACK        = 0x1
    TYPE_STATUS     = 0x2
    TYPE_CONFIG     = 0x3
    TYPE_BEACON     = 0x4
    TYPE_ACTOR_UNIVERSAL = 0x10    

    BIT_BROADCAST   = 0
    BIT_IS_ACK      = 1
    BIT_ACK_REQUEST = 2

    version = 0x0
    sender = None
    config = 0x0
    receiver = None
    msg_type = TYPE_UNDEFINED
    data = b""
    seq = None
    rssi = None
    snr = None

    def __init__(self, sender=None):
        self.sender = sender

    def __str__(self, additional=None):
        ret = ""
        ret += "Version: " + str(self.get_version()) + "\n"
        ret += "Sequence: " + str(self.get_sequence()) + "\n"
        ret += "Sender: " + str(self.get_sender()) + "\n"
        ret += "Receiver: " + str(self.get_receiver()) + "\n"
        ret += "Type: " + str(self.get_type()) + "\n"
        ret += "Is Broadcast: " + str(self.get_broadcast()) + "\n"
        ret += "ACK request: " + str(self.get_ack_request()) + "\n"
        ret += "Is it an ACK: " + str(self.get_is_ack()) + "\n"
        ret += "SNR: " + str(self.get_snr()) + "\n"
        ret += "RSSI: " + str(self.get_rssi()) + "\n"
        
        if additional:
            ret += additional
        else:
            ret += "Data: " + str(self.data)
        return ret

    def get_version(self):
        return self.version

    def set_version(self, version):
        self.version = version

    def set_sequence(self, seq):
        self.seq = seq

    def get_sequence(self):
        return self.seq

    def get_sender(self):
        return self.sender

    def set_sender(self, sender):
        self.sender = sender
    
    def get_receiver(self):
        return self.receiver

    def set_receiver(self, receiver):
        self.set_broadcast(receiver==None)
        self.receiver = receiver

    def get_type(self):
        return self.msg_type

    def get_snr(self):
        return self.snr
    
    def set_snr(self, snr):
        self.snr = snr
    
    def get_rssi(self):
        return self.rssi

    def set_rssi(self, rssi):
        self.rssi = rssi

    def set_type(self, t):
        self.msg_type = t

    def get_data(self):
        return self.data

    def set_data(self, data):
        if type(data) != bytes:
            raise ValueError("data should be bytes")
        self.data = data

    def get_data_len(self):
        return len(self.data)
    
    def set_broadcast(self, bc=True):
        if bc:
            self.config = self.config | (1<<self.BIT_BROADCAST)
        else:
            self.config = self.config & ~(1<<self.BIT_BROADCAST)

    def get_broadcast(self):
        return bool(self.config & (1<<self.BIT_BROADCAST))

    def set_ack_request(self, ack=True):
        if ack:
            self.config = self.config | (1<<self.BIT_ACK_REQUEST)
        else:
            self.config = self.config & ~(1<<self.BIT_ACK_REQUEST)

    def get_ack_request(self):
        return bool(self.config & (1<<self.BIT_ACK_REQUEST))

    def set_is_ack(self, ack=True):
        if ack:
            self.config = self.config | (1<<self.BIT_IS_ACK)
        else:
            self.config = self.config & ~(1<<self.BIT_IS_ACK)

    def get_is_ack(self):
        return bool(self.config & (1<<self.BIT_IS_ACK))

    def get_config(self):
        return self.config

    def create_packet(self):
        if self.sender == None:
            raise ValueError("Sender not set")
        if not self.seq:
            self.seq = random.randint(0, 65535)
        p = struct.pack("!BHIB", self.version, self.seq, self.sender, self.config)
        if not self.get_broadcast():
            if not self.receiver:
                raise ValueError("Recevier not set")
            p = p + struct.pack("!I", self.receiver)
        p = p + struct.pack("!BB", self.msg_type, len(self.data))
        p = p + self.data

        p = p + struct.pack("!L", binascii.crc32(p))

        return p

    def parse_packet(self, packet):
        if len(packet) < 11:
            print("Packet too short")
            return False

        if struct.unpack("!L", packet[-4:])[0] != binascii.crc32(packet[:-4]):
            print("Invalid CRC32")
            return False

        p = packet[:-4] # rm crc

        if len(p) < 8:
            print("Packet too short")
            return False
        
        self.version, self.seq, self.sender, self.config = struct.unpack("!BHIB", p[:8])
        p = p[8:]
        if not self.get_broadcast():
            self.receiver, = struct.unpack("!I", p[:4])
            p = p[4:]

        if len(p) < 2:
            print("Packet too short")
            return False

        self.msg_type, datalen = struct.unpack("!BB", p[:2])
        p = p[2:]

        if len(p) != datalen:
            print("Invalid len")
            return False
        
        self.data = p

        return True

    def create_ack(self):
        if not self.get_ack_request() or self.get_broadcast():
            # No ACK requested or is broadcast
            return None
        return AckPacket(self.get_receiver(), self.get_sender(), self.get_sequence()).create_packet()
        

        
# Actor packet type
class UniversalPacket(BasePacket):
    def __init__(self, sender = None):
        super().__init__(sender)
        self.msg_type = self.TYPE_ACTOR_UNIVERSAL

    def __str__(self):
        data = self.get_params()
        ret = ""
        ret += "Cancel prev: " + str(data[0]) + "\n"
        return super().__str__(ret)

    def set_params(self, cancel_prev):
        self.data = struct.pack("!b", bool(cancel_prev))

    def get_params(self):
        if len(self.data) == 1:
            return struct.unpack("!b", self.data)
        else:
            raise ValueError("Wong Packet format")


# The ACK packet type
class AckPacket(BasePacket):
    def __init__(self, sender=None, receiver=None, sequence=None):
        super().__init__(sender)
        self.set_receiver(receiver)
        self.set_sequence(sequence)
        self.set_is_ack()
        self.set_type(self.TYPE_ACK)


class BeaconPacket(BasePacket):
    def __init__(self, sender=None, bat=(None, None)):
        super().__init__(sender)
        self.set_broadcast()
        self.set_type(self.TYPE_BEACON)
        if bat != (None, None):
            self.data = struct.pack("!II", bat[0], bat[1])

    def getBattery(self):
        if len(self.data) == 8:
            return struct.unpack("!II", self.data)
        return None

    def __str__(self):
        data = self.getBattery()
        ret = ""
        if data:
            ret += "Percent: " + str(data[0]) + "\n"
            ret += "Volt: " + str(data[1]/1000) + "\n"
        return super().__str__(ret)

    

if __name__ == "__main__":

    # Just a little bit of testing

    p = BasePacket(123)
    print(p)
    p.set_ack_request()
    #p.set_broadcast()
    p.set_receiver(332)
    #p.set_is_ack()
    p.set_data(b"TEST")
    print(p)
    print(p.get_config())

    bin_packet = p.create_packet()
    print("LEN", len(bin_packet))

    p2 = BasePacket()
    if p2.parse_packet(bin_packet):
        print("Packet successfully parsed")
        print(p2)


    f = UniversalPacket(321)
    f.set_ack_request()
    bp = f.create_packet()
    f2 = UniversalPacket()
    f2.parse_packet(bp)
    print(f2)

    ack = f2.create_ack()
    print(ack)

    p3 = BasePacket()
    p3.parse_packet(ack)
    print(p3)
