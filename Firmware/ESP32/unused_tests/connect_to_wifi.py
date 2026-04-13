# Source - https://stackoverflow.com/a/64203146
# Posted by Lixas, modified by community. See post 'Timeline' for change history
# Retrieved 2026-03-28, License - CC BY-SA 4.0

def do_connect():
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect('169 Wissler', 'Happy-2024')
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())

do_connect()