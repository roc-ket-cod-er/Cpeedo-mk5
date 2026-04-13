from sim7080g import Cell

test = Cell()

while True:
    test.power_off()
    test.connect('io')
    test.send_message('heya')
    test.power_off()
