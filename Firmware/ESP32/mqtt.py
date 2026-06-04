import uasyncio as asyncio
from uasyncio import sleep_ms
from time import sleep, ticks_ms

class MQTT(object):
    def __init__(self, cell=None, wifi=None):
        if cell != None:
            try:
                if cell._version[0] == 1 and cell._version[1] == 1:
                    self.cell = cell
                    self.cell_setup = True
                else:
                    raise RuntimeError("Incorrect cell object version.")
            except AttributeError:
                raise RuntimeError("Incorrect wifi object given. Required module is availible at 'https://github.com/roc-ket-cod-er/Cpeedo-mk5/blob/main/Firmware/ESP32/sim7080g.py'")

        else:
            self.cell = None
            self.cell_setup = False
            
        if wifi != None:
            try:
                if wifi._version[0] == 1:
                    self.wifi = wifi
                    self.wifi_setup = True
                else:
                    raise RuntimeError(f"Incorrect wifi object version: {wifi.version}")
            except AttributeError:
                raise RuntimeError("Incorrect wifi object given. Required module is availible at 'https://github.com/roc-ket-cod-er/Cpeedo-mk5/blob/main/Firmware/ESP32/wifi.py'")
        else:
            self.wifi = None
            self.wifi_setup = False
            
        if self.wifi_setup:
            pass
        elif self.cell_setup:
            pass
        else:
            raise ValueError("No wifi object or cell object given.")
        
    async def _wifimsg(self, msg, feed, qos, show=False):
        if self.wifi_setup:
            if self.wifi.is_on:
                started = 'on'
            else:
                started = 'off'
                self.wifi.on()
            if not self.wifi.is_connected_to_wifi:
                if not await self.wifi.aconnect():
                    return False
            try:
                if show:
                    print(msg)
                self.wifi.msg(msg, feed, qos)
            except AttributeError:
                self.wifi.connect_to_mqtt()
                self.wifi.msg(msg, feed, qos)
                
            if qos == 0:
                await sleep_ms(1500)
            
            if started == 'off':
                self.wifi.off()
            return True
        
    async def _cellmsg(self, msg, feed):
        if not self.cell_setup:
            return False
        if self.cell.is_off:
            await self.cell.aon()
            started='off'
        else:
            started='on'
        await self.cell.aconnect('io')
        await self.cell.amsg(msg, feed)
        return True
        
    async def amsg(self, msg, feed, prefer="wifi", qos=1, debug=False):
        st=ticks_ms()
        if prefer.lower() == "wifi":
            if await self._wifimsg(msg, feed, qos):
                if debug:
                    print(ticks_ms() - st, 'wifi')
                return True
            if debug:
                print(ticks_ms() - st, "cell")
            return await self._cellmsg(msg, feed)
        else:
            if await self._cellmsg(msg, feed):
                if debug:
                    print(ticks_ms() - st, "cell")
                return True
            if debug:
                print(ticks_ms() - st, 'wifi')
            return await self._wifimsg(msg, feed, qos)
            
async def main():
    from wifi import Wifi
    from sim7080g import Cell
    wifi=Wifi(on=True)
    cell = Cell()
    mqtt = MQTT(wifi=wifi, cell=cell)
    st = ticks_ms()
    for i in range(5):
        #asyncio.create_task(mqtt.amsg(f'testing #{i+1}', 'test'))
        await mqtt.amsg(f'testing #{i+1}', 'test')
        await sleep_ms(0)
    print(ticks_ms() - st)
    wifi.off()
    cell.off()

if __name__ == '__main__':
    asyncio.run(main())

