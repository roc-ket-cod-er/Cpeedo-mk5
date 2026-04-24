from machine import UART, Pin
from time import sleep_ms, ticks_ms
import secrets
import uasyncio as asyncio

class Cell(object):
    def __init__(self, pwrkey=48, pwr_detect=15, init_uart=True):
        self.uart_init = False
        self.pwrkey = Pin(pwrkey, Pin.OUT)
        self.pwrkey.value(0)
        self.pwr_detect = Pin(pwr_detect, Pin.IN)
        self.init_uart(1)
        self.comms_availible = True
                
    def warn(self, msg):
        print("ERROR:", msg)
        
    def init_uart(self, uart, baudrate=115200, tx=41, rx=42, cts=40, rts=39):
        self.uart = UART(uart, baudrate=baudrate, tx=tx, rx=rx, cts=cts, rts=rts, rxbuf=32768)
    
    def cmd(self, cmd):
        self.uart.write(cmd+"\r\n")
    
    def at(self, cmd, wait=10, show=False, end="\n", exWait=0, search=False, include="AT+"):
        resp = b''
        found = False
        start = ticks_ms()
        self.uart.read()    
        self.cmd(include+cmd)
        
        while ticks_ms() - start < wait * 1000:
            if self.uart.any():
                resp += self.uart.read()
                if b'OK' in resp or b'ERROR' in resp:
                    found = True
                    break
                elif search:
                    if search in resp:
                        found = True
                        break
            sleep_ms(1)
        sleep_ms(int(exWait*1000))
        if self.uart.any():
            resp += self.uart.read()
        if show:
            print(">>", cmd)
        if resp:
            try:
                if show:
                    print("<<", resp.decode(), end=end)
                return resp.decode()
            except UnicodeError:
                if show:
                    print("<< UNICODE ERROR", end="\t")
                    print("resp", end=end)
                return resp
        else:
            if show:
                print()
            return None
        
    async def aat(self, cmd, wait=10, include="AT+", show=False, end="\n", exWait=0, search=False):
        self.comms_availible = False
        resp = b''
        found = False
        start = ticks_ms()
        self.uart.read()    
        self.cmd(include+cmd)
        while ticks_ms() - start < wait * 1000:
            if self.uart.any():
                resp += self.uart.read()
                if b'OK' in resp or b'ERROR' in resp:
                    found = True
                    break
                elif search:
                    if search in resp:
                        found = True
                        break
            await asyncio.sleep_ms(5)
        await asyncio.sleep_ms(int(exWait*1000))
        if self.uart.any():
            resp += self.uart.read()
        if show:
            print(">>", cmd)
        if resp:
            try:
                if show:
                    print("<<", resp.decode(), end=end)
                self.comms_availible = True
                return resp.decode()
            except UnicodeError:
                if show:
                    print("<< UNICODE ERROR", end="\t")
                    print("resp", end=end)
                self.comms_availible = True
                return resp
        else:
            if show:
                print()
            self.comms_availible = True
            return None

    @property
    def is_on(self):
        if self.pwr_detect.value():
            return True
        return False
    
    @property
    def is_off(self):
        return not self.is_on
    
    
    def turn_on(self, shush=False, wait_for_boot=True):
        if self.is_on:
            if not shush:
                self.warn("already on")
            return
        self.pwrkey.value(1)
        sleep_ms(1300)
        self.pwrkey.value(0)
        i=0
        
        if wait_for_boot:
            while i < 2000:
                if self.at("AT", wait=0.1, include='') == "AT\r\r\nOK\r\n":
                    break
                i += 1
            
    def pwr_on(self, shush=False, wait_for_boot=True):
        return self.turn_on(shush, wait_for_boot)     
    def power_on(self, shush=False, wait_for_boot=True):
        return self.turn_on(shush, wait_for_boot)      
    def boot(self, shush=False, wait_for_boot=True):
        return self.turn_on(shush, wait_for_boot)
    def on(self, shush=False, wait_for_boot=True):
        return self.turn_on(shush, wait_for_boot)
    
    async def aon(self, shush=False, wait_for_boot=False):
        if self.is_on:
            if not shush:
                print("ERROR: already on")
            return
        self.pwrkey.value(1)
        await asyncio.sleep_ms(1500)
        self.pwrkey.value(0)

    async def aoff(self, shush=False, hold=False):
        if self.is_off:
            if not shush:
                self.warn("already off")
            return
        self.pwrkey.value(1)
        await asyncio.sleep_ms(1500)
        self.pwrkey.value(0)
    
    def off(self, shush=False, hold=True):
        if self.is_off:
            if not shush:
                self.warn("already off")
            return
        if hold:
            sleep_ms(100)
        self.pwrkey.value(1)
        sleep_ms(1300)
        self.pwrkey.value(0)
        if hold:
            sleep_ms(2000)
            
    def turn_off(self, shush=False, hold=True):
        self.off(shush=False, hold=True)
    def power_off(self, shush=False, hold=True):
        self.off(shush=False, hold=True)
    def shut_down(self, shush=False, hold=True):
        self.off(shush=False, hold=True)
    
    
    def connected(self):
        resp = self.at("SMSTATE?")
        if '+SMSTATE: 1' in resp:
            return "mqtt"
        resp = self.at("SHSTATE?")
        if '+SHSTATE: 1' in resp:
            resp = self.at("SHCONF?")
            return "web"
        return None
    
    @property
    def is_connected(self):
        return self.connected()

    def connect(self, server, url='', force=False):
        if not self.is_on:
            self.power_on()
            
        while "99,99" in self.at("CSQ"):		# signal
            sleep_ms(50)
            
        mqtt = ['io', 'adafruit', 'mqtt']
        web  = ['https', "web", "http"]
        
        if not force:
            if server in mqtt:
                if self.connected() == "mqtt":
                    return
            elif server in web:
                if self.connected() == "web":
                    return
            else:
                self.warn("huh?")
        
        self.at('SMDISC') #DISCONNECT FROM ADAFRUIT IO
        self.at('SHDISC')  #DISCONNECT FROM HTTP
        self.at('CGDCONT=1,"IP","simbase"')

        self.at("CEREG?", show=True)		#signal type

        while "NO SERVICE" in self.at("CPSI?"):
            self.at("CSQ")
            self.at("CEREG?") 
            sleep_ms(100)

        #self.at("AT+CGACT?")		#active?
        #self.at('AT+CGDCONT?')		#ip & stuff
        self.at('CGPADDR=1')		#connect to wifi
        
        self.at('CNACT=0,1', exWait=0.1)
        self.at('CACID=0')

        if server in mqtt:
            self.at('SMCONF="URL","io.adafruit.com",1883')
            self.at(f'SMCONF="CLIENTID","{secrets.client_id}"')
            self.at(f'SMCONF="USERNAME","{secrets.user_name}"')
            self.at(f'SMCONF="PASSWORD","{secrets.key}"')
            self.at('SMCONN', 10)
        if server in web:
            self.at('SHCONF="BODYLEN",1024')
            self.at('SHCONF="HEADERLEN",350')
            self.at(f'SHCONF="URL","{url}"')
            self.at('SHCONN')
            
    async def aconnect(self, server, url='', force=False, show=True):
        if not self.is_on:
            await self.aon()
            
        while "99,99" in await self.aat("CSQ"):		# signal
            await asyncio.sleep_ms(100)
            
        mqtt = ['io', 'adafruit', 'mqtt']
        web  = ['https', "web", "http"]
        
        if not force:
            if server in mqtt:
                if self.connected() == "mqtt":
                    return
            elif server in web:
                if self.connected() == "web":
                    return
            else:
                self.warn("huh?")
        
        await self.aat('SMDISC') #DISCONNECT FROM ADAFRUIT IO
        await self.aat('SHDISC')  #DISCONNECT FROM HTTP
        await self.aat('CGDCONT=1,"IP","simbase"')

        if show:
            await self.aat("CEREG?", show=True)		#signal type

        while "NO SERVICE" in self.at("CPSI?"):
            '''self.at("CSQ")
            self.at("CEREG?")'''
            await asyncio.sleep_ms(100)

        #self.at("AT+CGACT?")		#active?
        #self.at('AT+CGDCONT?')		#ip & stuff
        await self.aat('CGPADDR=1')		#connect to wifi
        
        await self.aat('CNACT=0,1', exWait=0.1)
        await self.aat('CACID=0')

        if server in mqtt:
            await self.aat('SMCONF="URL","io.adafruit.com",1883')
            await self.aat(f'SMCONF="CLIENTID","{secrets.client_id}"')
            await self.aat(f'SMCONF="USERNAME","{secrets.user_name}"')
            await self.aat(f'SMCONF="PASSWORD","{secrets.key}"')
            await self.aat('SMCONN', 15)
        if server in web:
            await self.aat('SHCONF="BODYLEN",1024')
            await self.aat('SHCONF="HEADERLEN",350')
            await self.aat(f'SHCONF="URL","{url}"')
            await self.aat('SHCONN')
        
        if show:
            print(self.is_connected, "connected, iirc")
            
    def send_message(self, msg, feed="test"):
        self.connect('io')
        self.at(f'SMPUB="space_coder/feeds/{feed}",{len(str(msg))},1,0', end=" ", search=">")
        self.uart.write(str(msg))
            
        start = ticks_ms()
        resp = b''
        found = False
        
        while ticks_ms() - start < 5000:
            sleep_ms(1)
            if self.uart.any():
                resp += self.uart.read()
                if b'OK' in resp or b'ERROR' in resp:
                    found = True
                    break
        try:
            print(resp.decode())
        except Exception as e:
            print(e)
            print(resp)
            
            
    async def amsg(self, msg, feed="test"):
        await self.aconnect('io')
        await self.aat(f'SMPUB="space_coder/feeds/{feed}",{len(str(msg))},1,0', end=" ", search=">")
        self.uart.write(str(msg))
            
        start = ticks_ms()
        resp = b''
        found = False
        
        while ticks_ms() - start < 5000:
            await asyncio.sleep_ms(10)
            if self.uart.any():
                resp += self.uart.read()
                if b'OK' in resp or b'ERROR' in resp:
                    found = True
                    break
        try:
            print(resp.decode())
        except Exception as e:
            print(e)
            print(resp)
    
    def sleep(self, duration, unit="s"):
        if unit == "s":
            sleep_ms(duration * 1000)
        else:
            sleep_ms(duration)
            
    @property
    def btry(self):
        try:
            btry, btryV = self.at('CBC', show=False).split(',')[1:3]
            
            btryV = int(btryV[0:4])
            btry = int(btry)
            
            return btry, btryV
        except (IndexError, ValueError, AttributeError):
            return [-1, -1]