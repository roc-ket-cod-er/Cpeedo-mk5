from machine import UART, Pin
from time import sleep
from micropyGPS import MicropyGPS
import time
import st7789
import tft_config
import tfont1 as font
import tfont2 as kmh
import vga2_bold_16x32 as font_small
from datetime import timezone, timedelta, datetime


TX_PIN = 41
RX_PIN = 42
PWRKEY_PIN = 48


my_gps = MicropyGPS()
tft2 = tft_config.config()
tft= tft_config.config1()

uart = UART(1, baudrate=115200, tx=TX_PIN, rx=RX_PIN, cts=40, rts=39)
gnss = UART(2, baudrate=9600, rx=5)


pwrkey = Pin(PWRKEY_PIN, Pin.OUT, Pin.PULL_DOWN)
pwrkey.value(0)

msg = 'TESTINGtesting!@#123'
connected = False
cell_power_pin = Pin(15, Pin.IN)

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
    
def power_down():
    if powered_on():
        print("Powering down SIM7080G...")
        pwrkey.value(1)   # pull PWRKEY HIGH
        time.sleep(2)     # hold 2 seconds
        pwrkey.value(0)   # release
        print("SIM7080G should shut down now")
        time.sleep(5)
    
def power_off():
    power_down()

def send_at(cmd, wait=5, show=True, end="\n", smart=True, exWait=0):
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
    resp = send_at("AT+SMSTATE?", show=False)
    if '+SMSTATE: 1' in resp:
        return "mqtt"
    
    resp = send_at("AT+SHSTATE?", show=False)
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
        if server in web:
            if connected() == "web":
                return
    
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
    
    send_at('AT+CNACT=0,1', wait=0.3, smart=False)
    send_at('AT+CACID=0')

    if server in mqtt:
        send_at('AT+SMCONF="URL","io.adafruit.com",1883')
        send_at('AT+SMCONF="CLIENTID","esp32-sim7080g"')
        send_at('AT+SMCONF="USERNAME","space_coder"')
        send_at('AT+SMCONF="PASSWORD","io key"')
        send_at('AT+SMCONN', 1)
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
        send_at(f'AT+SMPUB="space_coder/feeds/{feed}",{len(str(dif_msg))},1,0', end=" ", wait=0.1)
        uart.write(str(dif_msg))
    else:
        send_at(f'AT+SMPUB="space_coder/feeds/{feed}",{len(str(msg))},1,0', end=" ", wait=0.1)
        uart.write(str(msg))
        
    while not uart.any():
        sleep(0.01)
    
    sleep(1)
    resp = uart.read()
    try:
        print(resp.decode())
    except Exception as e:
        print(e)
        print(resp)

'''power_off()
raise RuntimeError'''

for i in range(1):
    
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
            while gnss.any():
                data = gnss.read()
                for byte in data:
                    stat = my_gps.update(chr(byte))
                    if stat is not None:
                        # Print parsed GPS data
                        '''print('UTC Timestamp:', my_gps.timestamp)
                        print('Date:', my_gps.date_string('long'))
                        print('Latitude:', my_gps.latitude_string())
                        print('Longitude:', my_gps.longitude_string())
                        print('Altitude:', my_gps.altitude)
                        print('Satellites in use:', my_gps.satellites_in_use)
                        print('Horizontal Dilution of Precision:', my_gps.hdop)
                        print()'''
                
            if my_gps.timestamp != last:
                st = time.ticks_ms()
                spd = my_gps.speed[2]
                spd = round(spd)
                spd = f"\n\n{spd:02d}"
                btry = int(send_at('AT+CBC', show=False).split(',')[1])
                btry = f"{btry:02d}"
                
                lst_btrys.append(int(btry))
                
                if len(lst_btrys) > 10:
                    lst_btrys.pop()
                    lst_btry = sum(lst_btrys) // 10
                else:
                    lst_btry = btry
                
                tm = f"{(my_gps.timestamp[0]-4)%12:02d}:{my_gps.timestamp[1]:02d}:{round(my_gps.timestamp[2]):02d}s"
                #print(f"{(my_gps.timestamp[0]-4)%12}:{my_gps.timestamp[1]}:{my_gps.timestamp[2]}s")
                tft.write(
                    font,
                    spd,
                    10, 40,
                    st7789.WHITE,
                    st7789.RED)
                #send_message(spd)
                
                if abs(int(lst_btry) - int(btry)) > 1:   
                    lst_btry = btry
                
                last = my_gps.timestamp
                print(tm, lst_btry, "%", btry, my_gps.timestamp)
                tft2.text(
                    font_small,
                    f"{tm}  {lst_btry}%",
                    5, 10,
                    st7789.WHITE,
                    st7789.RED)
                
                print(time.ticks_ms()-st)

                
    except KeyboardInterrupt:
        pass

    #send_message(f"FULL TEST !!! x{i}")
    #send_message(f"test connection !!! x{i}")
    
    #http("http://example.com")
    #send_at('AT+SHREQ="/",1', exWait=1.5)
    #send_at('AT+SHREAD=0,528', exWait=0.528/10)
    #send_at('AT+SHDISC')  #DISCONNECT FROM HTTP
    
    print(f"{send_at('AT+CBC').split(",")[1]}%")		#check battery %

power_off()





