"""
hello.py

    Writes "Hello!" in random colors at random locations on the display.
"""

import random
import utime
import st7789
import tft_config
import vga2_bold_16x32 as font

tft = tft_config.config(1)
tft2 = tft_config.config1(1)

def center(text):
    length = 1 if isinstance(text, int) else len(text)
    tft.text(
        font,
        text,
        tft.width() // 2 - length // 2 * font.WIDTH,
        tft.height() // 2 - font.HEIGHT //2,
        st7789.WHITE,
        st7789.RED)
    tft2.text(
        font,
        text,
        tft.width() // 2 - length // 2 * font.WIDTH,
        tft.height() // 2 - font.HEIGHT //2,
        st7789.WHITE,
        st7789.RED)
    
def main():
    tft.init()
    tft2.init()
    tft.fill(st7789.RED)
    tft2.fill(st7789.RED)
    
    center(b'\xAEHello\xAF')
    utime.sleep(2)
    tft.fill(st7789.BLACK)
    tft2.fill(st7789.BLACK)
    while True:
        for rotation in range(4):
            tft.rotation(rotation)
            tft.fill(0)
            tft2.rotation(rotation)
            tft2.fill(0)
            col_max = tft.width() - font.WIDTH*6
            row_max = tft.height() - font.HEIGHT

            for _ in range(128):
                tft.text(
                    font,
                    b'Hello!',
                    random.randint(0, col_max),
                    random.randint(0, row_max),
                    st7789.color565(
                        random.getrandbits(8),
                        random.getrandbits(8),
                        random.getrandbits(8)),
                    st7789.color565(
                        random.getrandbits(8),
                        random.getrandbits(8),
                        random.getrandbits(8)))
                tft2.text(
                    font,
                    b'Hello!',
                    random.randint(0, col_max),
                    random.randint(0, row_max),
                    st7789.color565(
                        random.getrandbits(8),
                        random.getrandbits(8),
                        random.getrandbits(8)),
                    st7789.color565(
                        random.getrandbits(8),
                        random.getrandbits(8),
                        random.getrandbits(8)))


main()