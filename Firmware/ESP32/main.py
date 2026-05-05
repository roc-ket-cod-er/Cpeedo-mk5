# Important Imports
from machine import Pin, UART, freq

# Prevent Automatic Shutdown / Comms
stm_com = Pin(43, Pin.OUT)
stm_com.off()

race_pin = Pin(3, Pin.IN, Pin.PULL_UP)
if not race_pin.value():
    import race
# Imports
from gps import GPS
from time import sleep_ms, ticks_ms
import uasyncio as asyncio

from wifi import Wifi
from sim7080g import Cell
from mqtt import MQTT

# Check if boot was for tracking or for 
track_pin = Pin(44, Pin.IN)
TRACK = True if track_pin.value() else False

# If running normally, import other things and initialize screen
if not TRACK:
    from screen import TFT
    import gc
    
    tft = TFT()

# Simple swap-ins to help simplify typing
ms = "ms"

# initialize objects
cell = Cell()
gps = GPS()
wifi = Wifi()
mqtt = MQTT(cell, wifi)

# initialize IOs
shut_off_pin = Pin(0, Pin.IN, Pin.PULL_UP)
shut_off_pin2= Pin(44,Pin.IN, Pin.PULL_DOWN)


# -------------- Function Defines ---------------- #

# To shut everything down
def shut_down():
    if cell.is_off:
        sleep_ms(500)
    else:
        cell.off()
    stm_com.on()
    
    
async def update_mqtt():
    global request_cell
    cell_free = False
    # Connect to mqtt
    await cell.aon()
    
    # Get Battery Percentage
    btry = cell.btry
    
    # Updated screens
    if btry[0] < 30:
        tft.right.write(
            tft.font,
            f"{btry[0]:02d}%",
            160, 8,
            tft.red,
            tft.black
        )
    tft.right.write(
        tft.font,
        f"{btry[0]:02d}%",
        160, 8,
        tft.light_green,
        tft.black
    )
    
    if gps.lock:
        await mqtt.amsg(f'{gps.pos[1][0]} {gps.pos[1][1]} {gps.pos[0][0]} {gps.pos[0][1]}', "trips", qos=0)
    
    await cell.aoff()
    cell_free = True

# ---------------- TRACKING CODE STARTS HERE ----------------- #
async def track():
    try:
        freq(80_000_000)
        st = ticks_ms()
        while ticks_ms() < 25_000 + st:
            gps.update(4096)
            if gps.sats[0] > 5:
                break
            print(gps.sats)
            sleep_ms(20)
        
        cell.on()
        wifi.on()
        
        while ticks_ms() < 40_000 + st:
            sleep_ms(20)
            gps.update(4096)
            if gps.lock:
                break
        
        gps.ban_updates()
        bat_list = cell.at('CBC').split(',')
        if gps.lock:
            gps.off()
            
            await mqtt.amsg(
                f"{gps.pos[1][0]}",
                "latitude"
            )
            
            if wifi.is_connected_to_wifi:
                cell.off()
            await mqtt.amsg(
                f"{gps.pos[0][0]}",
                "longitude"
            )
            
            await mqtt.amsg(
                f"{"/".join(map(str, gps.sats))}s {round(gps.speed, 2)}km/h {gps.gps.hdop} hdop {bat_list[1]}% ({bat_list[2][:-5]}mV) waited: {(ticks_ms()-st)//1000}s",
                feed='other-info'
            )
            
        else:
            cell.send_message(f"NO LOCK: {"/".join(map(str, gps.sats))}s {gps.gps.hdop} hdop {bat_list[1]}% ({bat_list[2][:-5]}mV) waited: {(ticks_ms()-st)//1000}s", feed='other-info')
    except Exception as e:
        print(e)
        shut_down()
    except KeyboardInterrupt:
        return
    shut_down()
    
# -------------------- MAIN LOOP STARTS HERE -------------------- #
async def main():
    global request_cell
    # Track if requested
    if TRACK:
        await track()
        return 1
    # Max out speed
    freq(240_000_000)
    
    # Set up screens
    tft.fill_all(tft.black)
    
    tft.left.write(
        tft.big_font,
        b'KM/H',
        10, 200,
        tft.white,
        tft.black)
    
    # Starting Variables
    old_speed = 978
    last_gc_collect = 0
    last_cell_on = ticks_ms()-42_000
    sat_string = ''
    cell_free = True
    request_cell = False
    
    # Clear GPS Buffer
    gps.uart.read()
    
    # Run loop
    while True:
        # Update GPS Values
        gps.update()
        
        # await to give the async functions a chance
        await asyncio.sleep_ms(1)
        
        # Check if it has been 45 seconds, and if so update the mqtt servers
        if last_cell_on + 45_000 < ticks_ms():
            last_cell_on = ticks_ms()
            if cell_free: # Ensure cell isn't alread on
                asyncio.create_task(update_mqtt())
                
        # await to give the async functions a chance
        await asyncio.sleep_ms(1)
        
        # Check if shutdown signal is given
        if not shut_off_pin.value() or shut_off_pin2.value():
            shut_down()
            
        # Collect GC if it is getting full
        if ticks_ms() > last_gc_collect + 120_000:
            print("COLLECTING GARBAGE")
            gc.collect()
            last_gc_collect = ticks_ms()
            
            # await to give the async functions a chance
            await asyncio.sleep_ms(1)
            
        
        # Update screens if there is new GPS information
        if gps.new:
            
            # Don't let GPS values change while updating screens
            gps.ban_updates()
            
            # Get starting time to monitor time taken to update
            s_t = ticks_ms()
            
            # Add main speed
            if round(old_speed) != round(gps.spd):
                tft.left.write(
                    tft.speed_font,
                    f"{round(gps.spd):02d}",
                    5, 30,
                    tft.white,
                    tft.black
                )
                
            # await to give the async functions a chance
            await asyncio.sleep_ms(1)
                
            # Add screen decimal and hdop
            if old_speed != gps.spd:
                tft.left.write(
                    tft.font,
                    f"{round(gps.spd%1, 1)}"[2:],
                    210, 160,
                    tft.white,
                    tft.black
                )
                old_speed = round(gps.spd, 1)
                
            # await to give the async functions a chance
            await asyncio.sleep_ms(1)
            
            time_stamp = [f"{gps.time[0]:02d}:{gps.time[1]:02d}", f":{gps.time[2]:0>4.1f}"]
            tft.right.write(
                tft.font,
                time_stamp[0],
                10, 43,
                tft.white,
                tft.black
            )
            tft.right.write(
                tft.small_font,
                time_stamp[1],
                134, 54,
                tft.white,
                tft.black
            )
            
            if sat_string != f"{gps.sats[0]:02d}/{gps.sats[1]:02d}/{gps.hdop:0>.1f}":
                tft.right.write(
                    tft.small_font,
                    f"{gps.sats[0]:02d}/{gps.sats[1]:02d}/{gps.hdop:0>.1f}",
                    10, 12,
                    tft.white,
                    tft.black,
                )
                sat_string = f"{gps.sats[0]:02d}/{gps.sats[1]:02d}/{gps.hdop:0>.1f}"
                
            # await to give the async functions a chance
            await asyncio.sleep_ms(1)
            
            # Print info out
            print(gps.time, gps.pos, gps.spd, gps.sats, gps.hdop, ticks_ms()-s_t, last_cell_on + 45_000 - ticks_ms(), gps.uart.any())

            # Re-allow updates
            gps.allow_updates()

        if s_t + 100 < ticks_ms():
            time_stamp = [f"{gps.time[0]:02d}:{gps.time[1]:02d}", f":{gps.time[2]+0.1:0>4.1f}"]
            tft.right.write(
                tft.font,
                time_stamp[0],
                10, 43,
                tft.white,
                tft.black
            )
            tft.right.write(
                tft.small_font,
                time_stamp[1],
                134, 54,
                tft.white,
                tft.black
            )
            s_t = ticks_ms() + 200
                

if __name__ == '__main__':
    try:
        asyncio.run(main())
    finally:
        cell.off()