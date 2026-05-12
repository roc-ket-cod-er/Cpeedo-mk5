# Important Imports
from machine import Pin, UART, freq, ADC
import uasyncio as asyncio
from random import randint

# Prevent Automatic Shutdown / Comms
stm_com = Pin(43, Pin.OUT)
stm_com.off()

# Imports
from sim7080g import Cell
from gps import GPS
from time import sleep_ms, ticks_ms

cell = Cell()
gps = GPS()

spd = 20

adc = ADC(Pin(3))
adc.atten(ADC.ATTN_11DB)
adc.width(ADC.WIDTH_12BIT)

while True:
    loop_start=ticks_ms()
    gps.ban_updates()
    
    cell.connect('io')
    
    spd += randint(-30, 30)/10
    spd = min(40, spd)
    spd = max(0, spd)
    spd = gps.spd
    
    raw = adc.read()
    voltage_adc = raw / 4095 * 3.3
    battery_voltage = round(voltage_adc * (127 / 27), 3)

    print(battery_voltage)
    
    if gps.lock:
        cell.send_message(
            f'{battery_voltage}V {spd} {cell.at('CBC').split(',')[2][:-8]} {"/".join(map(str, gps.sats))}', "ev-run"
        )
    else:
        cell.send_message(
            f'{battery_voltage}V-nolock {spd} {cell.at('CBC').split(',')[2][:-8]} {"/".join(map(str, gps.sats))}', "ev-run"
        )
        
    print(ticks_ms() - loop_start)
    gps.allow_updates()
    gps.update(4096)
    while 2100+loop_start-ticks_ms() > 0:
        gps.update(4096)