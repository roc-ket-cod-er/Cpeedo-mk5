"""
    Writes speed
    
"""

import random
import utime
import st7789
import tft_config
import tfont1 as font
import tfont2 as kmh


tft = tft_config.config(1)
tft2 = tft_config.config1(1)
rotation = 0

def center(text):
    length = 1 if isinstance(text, int) else len(text)
    tft.write(
        font,
        text,
        tft.width() // 2 - length // 2 * font.MAX_WIDTH,
        tft.height() // 2 - font.HEIGHT //2,
        st7789.WHITE,
        st7789.RED)
    tft2.write(
        font,
        text,
        tft.width() // 2 - length // 2 * font.MAX_WIDTH,
        tft.height() // 2 - font.HEIGHT //2,
        st7789.WHITE,
        st7789.RED)
    
def main():
    global rotation
    tft.init()
    tft2.init()
    tft.fill(st7789.RED)
    tft2.fill(st7789.RED)
    
    #center(b'\xAE123\xAF')
    #utime.sleep(2)
    tft.rotation(rotation)
    tft2.rotation(rotation)
    col_max = tft.width() - 100
    row_max = tft.height() - font.HEIGHT
    
    tft.write(
        font,
        b'00',
        10, 40,
        st7789.WHITE,
        st7789.RED)
    tft2.write(
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
    tft2.write(
        kmh,
        b'KM/H',
        10, 200,
        st7789.WHITE,
        st7789.RED)
    
    for i in range(10000):
        utime.sleep(0.2)
    
        tft.write(
            font,
            f'{i%100:02d}',
            10, 40,
            st7789.WHITE,
            st7789.RED)
        tft2.write(
            font,
            f'{i%100:02d}',
            10, 40,
            st7789.WHITE,
            st7789.RED)


main()