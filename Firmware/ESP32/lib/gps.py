from micropyGPS import MicropyGPS
from machine import UART
from time import sleep_ms

class GPS(object):
    def __init__(self, uart_id=2, rx=5, tx=4, tmzone=-4):
        self.uart = UART(uart_id, baudrate=9600, rx=rx, tx=tx)
        self.gps = MicropyGPS(tmzone, 'dd')
        self.cmd("$PMTK251,115200*1F")
        sleep_ms(100)
        self.uart = UART(uart_id, baudrate=115200, rx=rx, tx=tx)
        self.cmd("$PMTK220,200*2C")
        self.last_given_timestamp = []
        self.last_speed = 0
        
        
    def cmd(self, msg):
        self.uart.write(f"{msg}\r\n")
        
    def update(self):
        if self.uart.any():
            data = self.uart.read()
            for char in data:
                self.gps.update(chr(char))
                
    @property
    def time(self):
        self.rq()
        return self.gps.timestamp
    
    @property
    def pos(self):
        self.rq()
        return [self.gps.longitude, self.gps.latitude]
    @property
    def position(self):
        return self.pos
    
    @property
    def speed(self):
        self.rq()
        self.last_speed = round(self.gps.speed[2], 2)
        return self.last_speed
    @property
    def spd(self):
        return self.speed
    
    @property
    def delta_speed(self):
        last = self.last_speed
        return self.speed-last
    @property
    def delta_spd(self):
        return self.delta_speed
    
    def spd_change_at(self, digit):
        last = self.last_speed
        return True if round(last, digit) != round(self.speed, digit) else False
    
    def rq(self):
        self.update()
        self.last_given_timestamp = self.gps.timestamp
        
    @property
    def new(self):
        return self.gps.timestamp != self.last_given_timestamp
    
    @property
    def sats(self):
        self.rq()
        return [self.gps.satellites_in_use, self.gps.satellites_in_view]
    @property
    def satellites(self):
        return self.sats
    
    def use_sats(self):
        return self.gps.satellites_in_use
    
    def view_sats(self):
        return self.gps.satellites_in_view
    
    @property
    def lock(self):
        long_lock = self.position[0] != 0
        lat_lock =  self.position[1] != 0
        
        return True if lat_lock and long_lock else False