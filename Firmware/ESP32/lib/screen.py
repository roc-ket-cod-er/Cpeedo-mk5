import st7789
import tft_config
import tft.tfont2 as big_font
import tft.tfont1 as speed_font
import vga2_bold_16x32 as font
import vga1_8x16 as small_font

class TFT(object):
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
        
    @property
    def black(self):
        return 0x801
    @property
    def white(self):
        return 0xEE55
    @property
    def sky(self):
        return 0x867E