import network

wifi = network.WLAN(network.STA_IF)
wifi.active(True)

wifi.connect("pocom4pro","101211307")

while not wifi.isconnected():
    pass

if wifi.isconnected():
    print(wifi.ifconfig())