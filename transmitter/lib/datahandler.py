import mpyaes

from packets import UniversalPacket, BasePacket, BeaconPacket
from utils import get_node_id

class DataHandler:
    def __init__(self, key):
        if type(key) != bytes:
            raise ValueError("Key has to be a byte array")
        if len(key) != 32:
            raise ValueError("Key has to be 16 bytes long")
        self.key = key


    def encrypt(self, data):
        iv = mpyaes.generate_IV(16)
        cipher = mpyaes.new(self.key, mpyaes.MODE_CBC, iv)
        return iv + cipher.encrypt(data)

    def decrypt(self, data):
        iv = data[:16]
        cipher = mpyaes.new(self.key, mpyaes.MODE_CBC, iv)
        return cipher.decrypt(data[16:])

    def sendEncActor(self, nodeCfg):
        pkg = None
        pkg = UniversalPacket(get_node_id())
        pkg.set_params(nodeCfg["action_cancel_previous"])
        pkg.set_receiver(nodeCfg["actor_node"])
        if "use_ack" in nodeCfg and nodeCfg["use_ack"] and not pkg.get_broadcast():
            pkg.set_ack_request()
        return (self.encrypt(pkg.create_packet()), pkg.get_sequence(), pkg.get_ack_request())


    def sendEncBeacon(self, *args, **kwargs):
        bp = BeaconPacket(get_node_id(), *args, **kwargs)
        return self.encrypt(bp.create_packet())

    def receiverEncPacket(self, packet, is_sniffer=False):
        data = self.decrypt(packet)
        p = BasePacket()
        p.parse_packet(data)

        if p.get_type() == BasePacket.TYPE_ACTOR_UNIVERSAL:
            p = UniversalPacket()
            p.parse_packet(data)

        elif p.get_type() == BasePacket.TYPE_BEACON:
            p = BeaconPacket()
            p.parse_packet(data)


        if p.get_broadcast() or p.get_receiver() == get_node_id() or is_sniffer:
            return p
        
        # packet not for us
        print("Dropping packet: Not for us")
        return None

