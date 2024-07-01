from machine import Pin, ADC
from max17043 import max17043


class BatteryHandler:
    adc = None
    adc_pin = None
    max17043 = None

    def __init__(self):
        pass

    def do_wakeup(self):
        pass

    def do_sleep(self):
        pass

    def do_read(self):
        return (None, None)


class AnalogBatteryStatus:
    def __init__(self, analog_pin=Pin(36)):
        self.adc_pin = analog_pin
        self.adc = ADC(self.adc_pin)
        self.adc.atten(ADC.ATTN_11DB)
        self.adc.width(ADC.WIDTH_12BIT)

    def do_read(self):
        if self.adc:
            # Voltage divider by 2.7kOhm and 15kOhm
            volt_meas = self.adc.read() * 3.6 / 4095
            bat_volt = volt_meas * (2.7+15) / 2.7
            bat_percent = 0
            if bat_volt > 12.9:
                bat_percent = 100
            elif bat_volt > 12.8:
                bat_percent = 90
            elif bat_volt > 12.6:
                bat_percent = 80
            elif bat_volt > 12.5:
                bat_percent = 70
            elif bat_volt > 12.4:
                bat_percent = 60
            elif bat_volt > 12.25:
                bat_percent = 50
            elif bat_volt > 12.1:
                bat_percent = 40
            elif bat_volt > 11.9:
                bat_percent = 30
            elif bat_volt > 11.8:
                bat_percent = 20
            elif bat_volt > 11.5:
                bat_percent = 10
            else:
                bat_volt = 0
            return (bat_percent, bat_volt)
        return None

class Max17043BatteryStatus:
    def __init__(self, i2c):
        try:
            self.max17043 = max17043(i2c)
            if self.max17043.address is None:
                print("No fuel gauge found")
                # No device found
                self.max17043 = None
            else:
                print("Gauge:", self.max17043)
                print("%:", self.max17043.getSoc())
                print("V:", self.max17043.getVCell())
        except:
            print("Cannot connect to fuel gauge")
            self.max17043 = None

    def do_read(self):
        if self.max17043:
            return (self.max17043.getSoc(), self.max17043.getVCell())
        return None
