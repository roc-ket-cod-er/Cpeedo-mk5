# Important Imports
from machine import Pin, UART

# Prevent Automatic Shutdown / Comms
stm_com = Pin(43, Pin.OUT)
stm_com.off()

# Imports
from sim7080g import Cell
from gps import GPS
from time import sleep_ms, ticks_ms

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

# initialize slow objects if not tracking
if not TRACK:
    tft = TFT()

# Function Defines

# To shut everything down
def shut_down():
    cell.off()
    stm_com.on()

def track():
    try:
        st = ticks_ms()
        cell.connect('io')
        while ticks_ms() < 30_000 + st:
            if gps.sats[0] > 5:
                break
            print(gps.sats)
            
        while ticks_ms() < 40_000 + st:
            if gps.lock:
                break
        
        bat_list = cell.at('CBC').split(',')
        if gps.lock:
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

def main():
    # Track if requested
    if TRACK:
        track()
        return
    tft.fill_all(tft.black)
    tft.left.write(
        tft.speed_font,
        b'00',
        5, 20,
        tft.white,
        tft.black)
    
    tft.left.text(
        tft.font,
        f"{gps.spd%1:0<4.2f}"[2:],
        190, 165,
        tft.white,
        tft.black
    )
    
    tft.left.write(
        tft.big_font,
        b'KM/H',
        10, 205,
        tft.white,
        tft.black)
    
    # Starting Variables
    old_speed = 0
    
    # Run loop
    while True:
        gps.update()
        if gps.new:
            if old_speed != round(gps.spd):
                tft.left.write(
                    tft.speed_font,
                    f"{round(gps.spd):02d}",
                    10, 20,
                    tft.white,
                    tft.black
                )
                print("spd")
                old_speed = round(gps.spd)
            tft.left.text(
                tft.font,
                f"{gps.spd%1:0<4.2f}"[2:],
                190, 165,
                tft.white,
                tft.black
            )
            tft.right.text(
                tft.font,
                str(gps.spd)+"   ",
                10, 30,
                tft.white,
                tft.black
            )
            print(gps.time, gps.pos, gps.spd, gps.sats, gps.hdop)
                

if __name__ == '__main__':
    main()