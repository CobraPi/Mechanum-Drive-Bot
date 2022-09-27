import serial
import time
import datetime
import serial.tools.list_ports

ports = list(serial.tools.list_ports.comports())
for p in ports:
    if str(p[0]).startswith("/dev/cu.usbmodem"):
        port = str(p[0])
        print(port)

alarm = serial.Serial(port, 115200)
time.sleep(1)
alarm.write(b'aa')
time.sleep(5)
alarm.write(b'bb')
while True:
    dt = datetime.datetime.now()
    print(dt.hour,dt.minute,dt.second)
    if(dt.hour == 6 and dt.minute == 0):
        alarm.write(b'aa')
        usrInput = input("Enter to shut off alarm...")
        alarm.write(b'bb')
        break
    time.sleep(1)
