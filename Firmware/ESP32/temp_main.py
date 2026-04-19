# Important Imports
from machine import Pin, UART, freq

# Prevent Automatic Shutdown / Comms
stm_com = Pin(43, Pin.OUT)
stm_com.off()

# Imports
from sim7080g import Cell
from gps import GPS
from time import sleep_ms, ticks_ms
import gc
import uasyncio as asyncio

# Check if boot was for tracking or for 
track_pin = Pin(9, Pin.IN)
TRACK = True if track_pin.value() else False

# If running normally, import the screen driver
if not TRACK:
    from screen import TFT

# Simpler swap-ins to help simplify typing
ms = "ms"

# initialize objects
cell = Cell()
gps = GPS()

# initialize IOs
shut_off_pin = Pin(0, Pin.IN, Pin.PULL_UP)
shut_off_pin2= Pin(44,Pin.IN, Pin.PULL_DOWN)

# initialize slow objects if not tracking
if not TRACK:
    tft = TFT()

# Function Defines

# To shut everything down
def shut_down():
    if cell.is_off:
        sleep_ms(500)
    else:
        cell.off()
    stm_com.on()

def track():
    try:
        freq(80_000_000)
        st = ticks_ms()
        while ticks_ms() < 15_000 + st:
            if gps.sats[0] > 5:
                break
            print(gps.sats)
            
        cell.connect('io')
            
        while ticks_ms() < 40_000 + st:
            if gps.lock:
                break
        
        bat_list = cell.at('CBC').split(',')
        if gps.lock:
            gps.off()
            cell.send_message(f"{gps.pos[1][0]}",  feed='latitude') # Send latitude
            cell.send_message(f"{gps.pos[0][0]}", feed='longitude') # Send longitude
            cell.send_message(f"{"/".join(map(str, gps.sats))}s {round(gps.speed, 2)}km/h {gps.gps.hdop} hdop {bat_list[1]}% ({bat_list[2][:-5]}mV) waited: {(ticks_ms()-st)//1000}s", feed='other-info')
        else:
            cell.send_message(f"NO LOCK: {"/".join(map(str, gps.sats))}s {gps.gps.hdop} hdop {bat_list[1]}% ({bat_list[2][:-5]}mV)", feed='other-info')
    except Exception as e:
        print(e)
        shut_down()
    except KeyboardInterrupt:
        return
    shut_down()
# Main

async def main():
    # Track if requested
    if TRACK:
        track()
        return
    
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
    last_cell_on = 0
    btry = None
    sat_string = ''
    
    # Clear GPS Buffer
    gps.uart.read()
    
    # Run loop
    while True:
        # Update GPS Values
        gps.update()
        
        # await to give the cell on/off functions a chance
        await asyncio.sleep_ms(1)
        
        # check if cell chip is on
        if cell.is_on and btry==None:
            btry = cell.btry
            print(btry)
            asyncio.create_task(cell.aoff())
            
        if last_cell_on + 45_000 < ticks_ms():
            asyncio.create_task(cell.aon())
            btry = None
            last_cell_on = ticks_ms()
        
        # Check if shutdown signal is given
        if not shut_off_pin.value() or shut_off_pin2.value():
            shut_down()
            
        # Collect GC if it is getting full
        if ticks_ms() > last_gc_collect + 120_000:
            gc.collect()
            last_gc_collect = ticks_ms()
        
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
                
            # Add screen decimal and hdop
            if old_speed != gps.spd:
                tft.left.text(
                    tft.font,
                    f"{round(gps.spd%1, 1)}"[2:],
                    210, 160,
                    tft.white,
                    tft.black
                )
                old_speed = round(gps.spd, 1)
            
            time_stamp = ':'.join(map(str, gps.time))
            tft.right.text(
                tft.font,
                time_stamp,
                10, 10,
                tft.white,
                tft.black
            )
            
            if sat_string != f"{gps.sats[0]}/{gps.sats[1]}/{gps.hdop:0>.1f}":
                tft.right.text(
                    tft.font,
                    f"{gps.sats[0]}/{gps.sats[1]}/{gps.hdop:0>.1f}",
                    10, 48,
                    tft.sky,
                    tft.black,
                )
                sat_string = f"{gps.sats[0]}/{gps.sats[1]}/{gps.hdop:0>.1f}"
                
            # Re-allow updates
            gps.allow_updates()
            
            print(ticks_ms()-s_t, gps.uart.any())
            gps.update()
            print(ticks_ms()-s_t)
            
            # Print info out
            print(gps.time, gps.pos, gps.spd, gps.sats, gps.hdop, ticks_ms()-s_t, last_cell_on + 30_000 - ticks_ms())
                

if __name__ == '__main__':
    asyncio.run(main())