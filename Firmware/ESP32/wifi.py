from time import ticks_ms, sleep_ms
from umqtt.robust import MQTTClient
from network import WLAN, STA_IF, STAT_CONNECTING, STAT_GOT_IP 
import _thread
import uasyncio as asyncio

class Wifi(object):
    def __init__(self, ssid=None, password=None, on=False):
        try:
            from secrets import wifi_info, client_id, key, user_name
            self.no_info=False
        except ImportError as e:
            self.no_info=True
            
        self.wlan = WLAN(STA_IF)
        self.wlan.active(on)
        self._scan_start = _thread.allocate_lock()
        self._scan_start.acquire()
        self._scan_complete = asyncio.ThreadSafeFlag()
        self._scan_thread = _thread.start_new_thread(self._scan, ())
        self._scan_results = None
        
        self.wifi_info = wifi_info
        self.connected_to = None
        
        self.latitude = '{:s}/feeds/{:s}'.format(user_name, 'latitude')
        self.longitude = '{:s}/feeds/{:s}'.format(user_name, 'longitude')
        if not self.no_info:
            self.client = MQTTClient(client_id=client_id+'-wifi', 
                        server='io.adafruit.com', 
                        user=user_name, 
                        password=key,
                        ssl=False)
        else:
            self.client = None
        
        if self.no_info:
            if ssid != None:
                self.ssid = ssid
            else:
                raise ValueError("No SSID Provided")
            if password != None:
                self.password = password
            else:
                raise ValueError("No Password Provided")
        
    def on(self):
        self.wlan.active(True)
    def off(self):
        self.wlan.active(False)
        self.connected_to=None
    
    def _scan(self):
        while True:
            self._scan_start.acquire()
            try:
                self._scan_results = self.wlan.scan()
            except Exception as exc:
                print("Exception running scan", exc)
                self._scan_results = []
            self._scan_complete.set()

    async def scan(self, force=True):
        starting_on=True
        
        if not self.wlan.active():
            starting_on=False
            if force:
                self.wlan.active(True)
            else:
                return False

        self._scan_start.release()
        await self._scan_complete.wait()
        self._scan_complete.clear()
        
        if not starting_on:
            self.wlan.active(False)
        
        return self._scan_results
    
    async def _connect(self, ssid, password):
        self.wlan.connect(ssid, password)
        await asyncio.sleep_ms(200)
        while self.wlan.status() == STAT_CONNECTING:
            await asyncio.sleep_ms(300)
                  
        if self.wlan.isconnected():
            self.connected_to = ssid
            return True
        else:
            print(WLAN.status)
            return False
    
    async def connect(self, force=False):
        if self.wlan.isconnected() and not force:
            return True
        elif force:
            self.off()
            await asyncio.sleep_ms(50)
            self.on()
            
        if self.no_info:
            return await self._connect(self.ssid, self.password)
        else:
            visible_networks = []
            info = await self.scan()
            
            for network in info:
                visible_networks.append(network[0].decode())
                
            print(visible_networks)
            for network in self.wifi_info:
                if network in visible_networks:
                    if await self._connect(network, self.wifi_info[network]):
                        return True
            return False
    
    def force_new_info(self, ssid, password):
        self.no_info = True
        self.ssid = ssid
        self.password = password
        
    def connect_to_mqtt(self):
        if self.client == None:
            print("Failed to connect: No secrets file detected")
            return "No secrets file detected"
        try:
            self.client.connect()
        except Exception as e:
            print(f"Failed to connect: {type(e).__name__} {e}")
    
    def pub_lat(self, msg):
        self.client.publish(self.latitude,    
                   msg,
                   qos=0)
        
    def pub_long(self, msg):
        self.client.publish(self.longitude,    
                   msg,
                   qos=0)  
        
    
async def test():
    wifi = Wifi()
    wifi.off()
    sleep_ms(50)
    wifi.on()
    print(f"\n\n{await wifi.connect()}")
    print(wifi.connected_to)
    wifi.connect_to_mqtt()
    wifi.pub_lat("43.50528")
    wifi.pub_long("80.52275")
    sleep_ms(10_000)
    wifi.off()
    

if __name__ == '__main__':
    asyncio.run(test())
    
    
    