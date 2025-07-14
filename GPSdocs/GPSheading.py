print("GPS Heading Calculator for Ubox 9")

from serial import Serial
from pyubx2 import UBXReader,UBXMessage,POLL_LAYER_RAM

stream = Serial('COM4', 115200, timeout=3)

layers = POLL_LAYER_RAM
transaction = 0
cfgData = [("CFG_UART1_BAUDRATE", 9600), (0x40530001, 115200)]
msg = UBXMessage.config_set(layers, transaction, cfgData)
print(msg)
stream.write(msg.serialize())

ubr = UBXReader(stream)

#for i in range(100):
#<UBX(NAV-PVT, iTOW=13:14:36, year=2023, month=4, day=1, hour=13, min=14, second=36, validDate=1, 
# validTime=1, fullyResolved=1, validMag=0, tAcc=33, nano=16086, fixType=3, gnssFixOk=1, difSoln=1, 
# psmState=0, headVehValid=0, carrSoln=0, confirmedAvai=1, confirmedDate=1, confirmedTime=1, numSV=22,
# lon=-105.0976275, lat=40.5170081, height=1545518, hMSL=1566898, hAcc=3742, vAcc=4262, velN=-49, 
# velE=12, velD=-18, gSpeed=50, headMot=0.0, sAcc=226, headAcc=180.0, pDOP=1.14, invalidLlh=0, 
# lastCorrectionAge=0, reserved0=877341956, headVeh=0.0, magDec=0.0, magAcc=0.0)>

while True:
    (raw_data, parsed_data) = ubr.read()
    if parsed_data.identity == "NAV-PVT":
        #print(parsed_data)
        print(f'lat: {parsed_data.lat}')
        print(f'lon: {parsed_data.lon}')
        if parsed_data.headVehValid:
            print(f'headVeh: {headVeh}')