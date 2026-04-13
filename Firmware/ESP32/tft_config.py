from machine import Pin, SPI
import st7789

TFA = 0
BFA = 0

spi = SPI(1, baudrate=60_000_000, sck=Pin(12), mosi=Pin(11))

def config(rotation=0, buffer_size=2457600, options=0):
    return st7789.ST7789(
        spi,
        240,
        320,
        #reset=Pin(21, Pin.OUT),
        reset=Pin(47, Pin.OUT),
        #cs=Pin(10, Pin.OUT),
        cs=Pin(13, Pin.OUT),        
        dc=Pin(14, Pin.OUT),
        backlight=Pin(16, Pin.OUT),
        rotation=rotation,
        options=options,
        buffer_size=buffer_size)

def config1(rotation=0, buffer_size=2457600, options=0):
    return st7789.ST7789(
        spi,
        240,
        320,
        reset=Pin(21, Pin.OUT),
        #reset=Pin(47, Pin.OUT),
        cs=Pin(10, Pin.OUT),
        #cs=Pin(13, Pin.OUT),        
        dc=Pin(14, Pin.OUT),
        backlight=Pin(16, Pin.OUT),
        rotation=rotation,
        options=options,
        buffer_size=buffer_size)