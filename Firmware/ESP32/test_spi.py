from machine import SPI, Pin
from time import sleep_us, ticks_ms

spi = SPI(
    1,
    baudrate=1_000_000,   # start slow for testing
    polarity=0,        # must match STM32 CPOL Low
    phase=0,           # must match STM32 CPHA 1 Edge
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(9),
    mosi=Pin(18),
    miso=Pin(8)
)


def read_bytes():
    tx_dummy = bytes([255]*5)   # send dummy byte (0xFF)
    rx = bytearray(5)

    spi.write_readinto(tx_dummy, rx)
    return list(rx)

lv = []

while True:
    value = read_bytes()

    if value[2] == 0 and (value[0] == 255 or value[0] == 0) and value[4] < 2 and value[1] != 0 and value[3] != 0:
        if value != lv:
            print("Raw bytes:", value, f"    \tx: {value[1]:03d} y: {value[3] + 256*value[4]:03d} tstamp: {ticks_ms()%10000}")
            lv = value
    elif value == [0,0,0,0,0]:
        pass
        #print(value)
    else:
        pass
        #print(value)
    
    sleep_us(1000)