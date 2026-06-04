from machine import SPI, Pin
from time import sleep_us, ticks_ms
from uasyncio import sleep_ms

class TouchObject(object):
    def __init__(self, x=0, y=0, is_touched=False):
        self.x = 0
        self.y = 0
        self.is_touched = False


class Comms(object):
    def __init__(self):
        self.spi = SPI(
            2,
            baudrate=1_000_000,   # start slow for testing
            polarity=0,        # must match STM32 CPOL Low
            phase=0,           # must match STM32 CPHA 1 Edge
            bits=8,
            firstbit=SPI.MSB,
            sck=Pin(9),
            mosi=Pin(18),
            miso=Pin(8)
        )
        
        self.lcd2 = TouchObject()
        self.get_touch()
        
    # Function to read bytes. Currently designed for the custom transmision from the STM32.
    def read_bytes(self):
        tx_dummy = bytes([255]*5)
        rx = bytearray(5)

        self.spi.write_readinto(tx_dummy, rx)
        return list(rx)
    
    # USE TO GET TOUCH (position and state)
    def get_touch(self):
        value = self.read_bytes()
        if value[2] == 0 and (value[0] == 255 or value[0] == 0) and value[4] < 2 and value[1] != 0 and value[3] != 0:
            
            self.lcd2.x = 240 - value[1]
            self.lcd2.y = 320 - (value[3] + 256*value[4])
            self.lcd2.is_touched = value[0]
            
            #print("x:", self.lcd2.x, "\ty:", self.lcd2.y, "\ttouched" if self.lcd2.is_touched else "\tnot touched")
            return (self.lcd2.x, self.lcd2.y, self.lcd2.is_touched)
        
    async def background_touch_updater(self, update_rate=10):
        while True:
            self.get_touch()
            await sleep_ms(1000//10)
            
    @property
    def touch_pos(self):
        return (self.lcd2.x, self.lcd2.y, self.lcd2.is_touched)
        

if __name__ == '__main__':
    test = Comms()
    while True:
        test.get_touch()
        sleep_us(5000)
    
    
    
        
        
        
        


