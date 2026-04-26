from time import ticks_ms, sleep_ms
from network import WLAN, STA_IF,
import _thread
import uasyncio as asyncio

try:
    from secrets import wifi_ssid, wifi_password
    no_info=False
except ImportError:
    no_info=True

class Wifi(object):
    def __init__(self, ssid=None, password=None, on=False):
        if no_info:
            if ssid != None:
                self.ssid = ssid
            else:
                raise ValueError("No SSID Provided")
            if password != None:
                self.password = password
            else:
                raise ValueError("No Password Provided")
        else:
            self.ssid = wifi_ssid
            self.password = wifi_password
            
        self.wlan = WLAN(STA_IF)
        self.wlan.active(on)
        self._scan_start = _thread.allocate_lock()
        self._scan_start.acquire()
        self._scan_complete = asyncio.ThreadSafeFlag()
        self._scan_thread = _thread.start_new_thread(self._scan, ())
        self._scan_results = None
        
        
    def on(self):
        self.wlan.active(True)
    def off(self):
        self.wlan.active(False)
    
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
    
async def test():
    wifi = Wifi()
    hold = await wifi.scan()
    for item in hold:
        print(item)
    

if __name__ == '__main__':
    asyncio.run(test())
    
    
    
    
    