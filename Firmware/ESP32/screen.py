import st7789
import tft_config
import tft.tfont2 as big_font
import tft.tfont1 as speed_font
import tft.normal as font
import tft.small_font as small_font

class TFT:
    def __init__(self):
        self.right = tft_config.config1(2)
        self.left = tft_config.config(2)
        
        self.right.init()
        self.left.init()
        
        self.speed_font = speed_font
        self.big_font = big_font
        self.font = font
        self.small_font = small_font
        
        self.st7789 = st7789
        
    def fill_all(self, color=0):
        self.left.fill(color)
        self.right.fill(color)
        
    def off(self):
        self.left.off()
        self.right.off()
        
    def on(self):
        self.left.on()
        self.right.on()
        
    @property
    def black(self):
        return 0x801
    @property
    def white(self):
        return 0xEE55
    @property
    def sky(self):
        return 0x867E
    @property
    def red(self):
        return st7789.RED
    @property
    def light_green(self):
        return 0x07E0
    
class Button(TFT):
    def __init__(self, tft, x, y, width, height, color, touch_object=None):
        tft.fill_rect(x, y, width, height, color)
        self.min_x = x
        self.min_y = y
        self.max_x = x + width
        self.max_y = y + height
        self.touch_setup = touch_object != None
        self.touch = touch_object
        
    def is_pressed(self):
        if not self.touch_setup:
            raise ValueError("No touch object provided at setup.")
        x, y, touched = self.touch.touch_pos
        
        if not touched:
            return False
        
        if self.min_x < x < self.max_x and self.min_y < y < self.max_y:
            return True
        
        return False
        













