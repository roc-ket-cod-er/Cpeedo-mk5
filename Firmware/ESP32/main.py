from machine import UART, Pin
import machine

stm_com = Pin(43, Pin.OUT)
stm_com.off()

speed_boot = Pin(9, Pin.IN)
if speed_boot.value():
    SPEED_BOOT = True
else:
    SPEED_BOOT = False

from time import sleep
from micropyGPS import MicropyGPS
import gc
import time
import math

if not SPEED_BOOT:
    print("lazy pants")
    import st7789
    import tft_config
    import tft.tfont1 as font
    import tft.tfont2 as kmh
    import vga2_bold_16x32 as font_small
    import vga1_8x16 as sSmall_font
    
    tft2 = tft_config.config(2)
    tft  = tft_config.config1(2)


TX_PIN = 41
RX_PIN = 42
PWRKEY_PIN = 48
SHUT_OFF_PIN = 0
SHUT_OFF_PIN2 = 44
SPEED_SENSE = 7

my_gps = MicropyGPS(0, "dd")

uart = UART(1, baudrate=115200, tx=TX_PIN, rx=RX_PIN, cts=40, rts=39)
gnss = UART(2, baudrate=9600, rx=5, tx=4)

def send_gps(msg):
    gnss.write(f"{msg}\r\n")


send_gps("$PMTK251,115200*1F")
sleep(0.1)
gnss = UART(2, baudrate=115200, rx=5, tx=4)

send_gps("$PMTK220,200*2C")

pwrkey = Pin(PWRKEY_PIN, Pin.OUT)
pwrkey.value(0)

shut_off_pin = Pin(SHUT_OFF_PIN, Pin.IN, Pin.PULL_UP)
shut_off_pin2= Pin(SHUT_OFF_PIN2,Pin.IN, Pin.PULL_DOWN)

msg = 'TESTINGtesting!@#123'
cell_power_pin = Pin(15, Pin.IN)

last_gc = 100000

def powered_on():
    '''global uart
    print(uart.read())
    uart.write("AT\r\n")
    sleep(0.1)
    hold = uart.read()
    if hold == b"AT\r\r\nOK\r\n":
        return True
    print([hold])
    return False'''
    if cell_power_pin.value() == 1:
        return True
    return False

def power_on():
    if powered_on():
        return
    global uart
    print("Toggling PWRKEY...")
    pwrkey.value(1)   # pull PWRKEY LOW via MOSFET
    time.sleep(1.5)   # hold ~1–2 seconds
    pwrkey.value(0)   # release
    print("Waiting for boot...")
    
    for _ in range(100):
        hold = send_at("AT", wait=0.1)
        if hold == "AT\r\r\nOK\r\n":
            break
    sleep(1)
    uart.read()
    '''uart.write("AT+IPR=115200\r\n")
    time.sleep(0.2)
    print(uart.read())
    uart = UART(1, baudrate=115200, tx=TX_PIN, rx=RX_PIN, timeout=1000)'''
    
def power_down(block=True):
    sleep(0.1)
    if powered_on():
        print("Powering down SIM7080G...")
        pwrkey.value(1)   # pull PWRKEY HIGH
        time.sleep(2)     # hold 2 seconds
        pwrkey.value(0)   # release
        print("SIM7080G should shut down now")
        if block:
            time.sleep(5)
    
def power_off(block=True):
    power_down(block)

def send_at(cmd, wait=5, show=True, end="\n", smart=True, exWait=0, search=False):
    resp = b''
    rFound = False
    start = time.ticks_ms()
    if uart.any():
        print(1, uart.read())
    uart.write(cmd + "\r\n")
    if not smart:
        time.sleep(wait)
        resp = uart.read()
    else:
        while time.ticks_ms() - start < wait * 1000:
            if uart.any():
                resp += uart.read()
                if b'OK' in resp or b'ERROR' in resp:
                    #print("hehe", resp)
                    rFound = True
                    #sleep(1)
                    break
                elif search:
                    if search in resp:
                        rFound = True
                        break
            time.sleep_ms(1)
            
        sleep(exWait)
        if uart.any():
            resp += uart.read()
        
    if show:
        print(">>", cmd)
    
    '''if not rFound and:
        print("NO RESPONSE")
        sleep(1)'''
        
    if resp:
        try:
            if show:
                print("<<", resp.decode(), end=end)
            return resp.decode()
        except UnicodeError:
            if show:
                print("<< UNICODE ERROR", end="\t")
                print("resp", end=end)
            return 1234567890
    else:
        if show:
            print()
        return ""
    
def connected():
    resp = send_at("AT+SMSTATE?", show=True)
    if '+SMSTATE: 1' in resp:
        return "mqtt"
    
    resp = send_at("AT+SHSTATE?", show=True)
    if '+SHSTATE: 1' in resp:
        resp = send_at("AT+SHCONF?")
        return "web"
    return 0
        
def connect_to_cell(server, url='', force=False):
    if not powered_on():
        power_on()
    mqtt = ['io', 'adafruit', 'mqtt']
    web  = ['https', "web", "http"]
    
    if not force:
        if server in mqtt:
            if connected() == "mqtt":
                return
        elif server in web:
            if connected() == "web":
                return
        else:
            print("huh?")
    
    send_at('AT+SMDISC') #DISCONNECT FROM ADAFRUIT IO
    send_at('AT+SHDISC')  #DISCONNECT FROM HTTP
    send_at('AT+CGDCONT=1,"IP","simbase"')
    
    while "99,99" in send_at("AT+CSQ"):		# signal
        sleep(0.3)

    send_at("AT+CEREG?")		#signal type

    while "NO SERVICE" in send_at("AT+CPSI?"):
        send_at("AT+CSQ")
        send_at("AT+CEREG?") 
        sleep(0.3)

    send_at("AT+CGACT?")		#active?
    send_at('AT+CGDCONT?')		#ip & stuff
    send_at('AT+CGPADDR=1')		#connect to wifi
    
    send_at('AT+CNACT=0,1', exWait=0.1)
    send_at('AT+CACID=0')

    if server in mqtt:
        send_at('AT+SMCONF="URL","io.adafruit.com",1883')
        send_at('AT+SMCONF="CLIENTID","esp32-sim7080g"')
        send_at('AT+SMCONF="USERNAME","space_coder"')
        send_at('AT+SMCONF="PASSWORD","uhhhhhh why do you want to know????"')
        send_at('AT+SMCONN', 10)
    if server in web:
        send_at('AT+SHCONF="BODYLEN",1024')
        send_at('AT+SHCONF="HEADERLEN",350')
        send_at(f'AT+SHCONF="URL","{url}"')
        send_at('AT+SHCONN')

def http(url):
    connect_to_cell("https", url)
        
def send_message(dif_msg=None, feed="test"):
    connect_to_cell('io')
    if dif_msg:
        send_at(f'AT+SMPUB="space_coder/feeds/{feed}",{len(str(dif_msg))},1,0', end=" ", search=">", exWait=0.1)
        uart.write(str(dif_msg))
    else:
        send_at(f'AT+SMPUB="space_coder/feeds/{feed}",{len(str(msg))},1,0', end=" ", search=">", exWait=0.1)
        uart.write(str(msg))
        
    start = time.ticks_ms()
    resp = b''
    
    while time.ticks_ms() - start < 5000:
        if uart.any():
            resp += uart.read()
            if b'OK' in resp or b'ERROR' in resp:
                #print("hehe", resp)
                rFound = True
                #sleep(1)
                break
            
        time.sleep_ms(1)
            
    sleep(0.1)
    
    if uart.any():
        resp += uart.read()
    try:
        print(resp.decode())
    except Exception as e:
        print(e)
        print(resp)

'''power_off()
raise RuntimeError'''

if not SPEED_BOOT:
    last_sat_data_len = 0
    
    lst_btry = -10
    lst_btrys= []
    
    tft.init()
    tft2.init()
    
    tft.fill(st7789.RED)
    tft2.fill(st7789.RED)
    
    tft.write(
        font,
        b'00',
        10, 40,
        st7789.WHITE,
        st7789.RED)
    tft.write(
        kmh,
        b'KM/H',
        10, 200,
        st7789.WHITE,
        st7789.RED)
    
    last = [0, 0, 0]

    power_on()

    try:
        while True:
            if shut_off_pin.value() == 0 or shut_off_pin2.value() == 1:
                raise KeyboardInterrupt
            while gnss.any():
                data = gnss.read()
                try:
                    for byte in data:
                        stat = my_gps.update(chr(byte))
                except IndexError:
                    pass
                
            if my_gps.timestamp != last:
                if my_gps.timestamp[2] % 1 < 0.2:
                    tft2_refresh = True
                else:
                    tft2_refresh = False
                
                st = time.ticks_ms()
                spd = my_gps.speed[2]
                spd = round(spd)
                spd = f"\n\n{spd:02d}"
                try:
                    btry = int(send_at('AT+CBC', show=False).split(',')[1])
                    btry = f"{btry:02d}"
                except IndexError:
                    pass
                
                sat_data = []
                for key in my_gps.satellite_data:
                    if my_gps.satellite_data[key][2]:
                        sat_data.append([my_gps.satellite_data[key][2], key])
                    else:
                        pass
                        #print(my_gps.satellite_data[key], key)
                sat_data = sorted(sat_data, reverse=True)
                try:
                    sat_data = sat_data[:5]
                except IndexError:
                    pass
                
                sat_y = 5
                
                if tft2_refresh:
                    if len(sat_data) < last_sat_data_len:
                        tft2.fill(st7789.RED)
                    for sat in sat_data:
                        sat_y += 15
                        tft2.text(
                            sSmall_font,
                            f"sat: {sat[1]:02d}, sig: {sat[0]:02d}  ",
                            10, 100 + sat_y,
                            st7789.WHITE,
                            st7789.RED)
                        
                    tft2.text(
                            sSmall_font,
                            f"hdop: {my_gps.hdop:.2f}  ",
                            10, 115 + sat_y,
                            st7789.WHITE,
                            st7789.RED)
                
                last_sat_data_len = len(sat_data)
                
                lst_btrys.append(int(btry))
                
                if len(lst_btrys) > 50:
                    lst_btrys.pop(0)
                    lst_btry = sum(lst_btrys) // 50
                else:
                    lst_btry = btry
                
                tm = f"{(my_gps.timestamp[0]-4)%12:02d}:{my_gps.timestamp[1]:02d}:{math.floor(my_gps.timestamp[2]):02d}s"
                #print(f"{(my_gps.timestamp[0]-4)%12}:{my_gps.timestamp[1]}:{my_gps.timestamp[2]}s")
                tft.write(
                    font,
                    spd,
                    10, 40,
                    st7789.WHITE,
                    st7789.RED)
                #send_message(spd)
                
                last = my_gps.timestamp
                print(tm, lst_btry, "%", btry, my_gps.timestamp, shut_off_pin.value())
                
                if tft2_refresh:
                    tft2.hline(5, 45, 230, st7789.WHITE)
                    tft2.text(
                        font_small,
                        f"{tm}  {lst_btry}%",
                        5, 10,
                        st7789.WHITE,
                        st7789.RED)
                    tft2.vline(165, 2, 43, st7789.WHITE)
                    tft2.text(
                        font_small,
                        f"{my_gps.satellites_in_use} sats in use",
                        5, 50,
                        st7789.WHITE,
                        st7789.RED)
                    tft2.text(
                        font_small,
                        f" ({my_gps.satellites_in_view} Visible)",
                        5, 80,
                        st7789.WHITE,
                        st7789.RED)
                
                if time.ticks_diff(time.ticks_ms(), last_gc) > 120000:
                    gc.collect()
                    gnss.read()
                    last_gc = time.ticks_ms()
                    gc_collect_time = tm
                    gc_collect_comp_time = time.ticks_ms()-st
                    
                    tft2.text(
                        sSmall_font,
                        f"GC in {gc_collect_comp_time}ms @ {gc_collect_time}",
                        10, 300,
                        st7789.WHITE,
                        st7789.RED)
                
                
                print(time.ticks_ms()-st)
                #print("\n\n", sat_data)

                
    except KeyboardInterrupt:
        tft2.fill(st7789.BLACK)
        tft2.text(
            font_small,
            "SHUTTING DOWN",
            5, 10,
            st7789.WHITE,
            st7789.BLACK)

    #send_message(f"FULL TEST !!! x{i}")
    #send_message(f"test connection !!! x{i}")
    
    #http("http://example.com")
    #send_at('AT+SHREQ="/",1', exWait=1.5)
    #send_at('AT+SHREAD=0,528', exWait=0.528/10)
    #send_at('AT+SHDISC')  #DISCONNECT FROM HTTP
else:
    try:
        power_on()
        i = 0
        while i < 3000:
            while gnss.any():
                data = gnss.read()
                for byte in data:
                    stat = my_gps.update(chr(byte))
            sleep(0.01)
            print(my_gps.satellites_in_use)
            i += 1
            if my_gps.satellites_in_use > (3000-i)//300:
                break
            
        while i < 5000:
            while gnss.any():
                data = gnss.read()
                for byte in data:
                    stat = my_gps.update(chr(byte))
            sleep(0.01)
            print(my_gps.satellites_in_use)
            i += 1
            if my_gps.latitude[0] != 0 and my_gps.longitude[0] != 0:
                break
        
        connect_to_cell('io')
        if my_gps.latitude[0] != 0 and my_gps.longitude[0] != 0:
            bat_list = send_at('AT+CBC', show=False).split(',')
            send_message(f"{my_gps.latitude[0]}",  feed='latitude')
            send_message(f"{my_gps.longitude[0]}", feed='longitude')
            send_message(f"{my_gps.satellites_in_use}s {round(my_gps.speed[2], 2)}km/h {my_gps.hdop} hdop {bat_list[1]}% ({bat_list[2][:-5]}mV) wait: {i//100}s", feed='other-info')
        else:
            send_message(f"NO LOCK: {my_gps.satellites_in_use}s {my_gps.hdop} hdop {bat_list[1]}% ({bat_list[2][:-5]}mV)", feed='other-info')
        sleep(0.5)
    except Exception as e:
        print(e)
        sleep(2)
        power_off(False)
        stm_com.on()

power_off(False)

if not SPEED_BOOT:
    tft2.fill(st7789.BLACK)
    tft2.text(
        font_small,
        "SHUT DOWN",
        5, 10,
        st7789.WHITE,
        st7789.BLACK)

sleep(2)
stm_com.on()