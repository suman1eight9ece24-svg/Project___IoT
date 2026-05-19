import network
import time
import urequests

timeout = 0

wifi = network.WLAN(network.STA_IF)

#Restarting WiFi
wifi.active(False)
time.sleep(0.5)
wifi.active(True)

wifi.connect("pocom4pro","101211307")

if not wifi.isconnected():
    print("connecting.........")
    while (not wifi.isconnected() and timeout<5):
        print(5-timeout)
        timeout = timeout+1
        time.sleep(1)

if(wifi.isconnected()):
    print("connected")
    req=urequests.get("https://example.com/")
    print(req.status_code)
    print(req.text)
else:
    print("Time Out")
    print("NOT connected")
    

        
    
    
