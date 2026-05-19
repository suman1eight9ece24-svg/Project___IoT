import network
import time
import urequests
import machine

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
   print("Conected")
   print("Network Config :", wifi.ifconfig())
else:
    print("Time Out")
    print("NOT connected") 
    
html ='''
<!DOCTYPE html>
<html>
<center><h2>ESP32 Webserver </h2></center>
<form>
<center>
<h3>LED</h3>
<button name="LED" value="ON" type="submit"> ON </button>
<button name="LED" value="OFF" type="submit"> OFF </button>
</center>
'''

#output pin
LED = machine.Pin(2,machine.Pin.OUT)
LED.value(0)

#initialising scket
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM) # AF_INET -> Internet socket,SOCK_STREAM -> TCP protocol

Host = "" # Empty = allow all IP adress
Port = 80 #Host port
s.bind((Host,Port))

s.listen(5) #manage max 5 clint at a time

#working loop
while True:
    connection_socket,address=s.accept() #
    print("Got a connection form",address)
    request=connection_socket.recv(1024)
    print("Connect",request)
    request=str(request)
    
    LED_ON = request.find("/?LED=ON")
    LED_OFF = request.find("/?LED=OFF")
    
    if(LED_ON==6):
        LED.value(1)
        
    if(LED_OFF==6):
        LED.value(0)
        
    #sending HTML document everytime
    responce=html
    connection_socket.send(responce)
    
    #close socket
    connection_socket.close()   


    

        
    
    


