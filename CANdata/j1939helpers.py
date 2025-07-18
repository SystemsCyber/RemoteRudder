#!/usr/bin/python
'''
Helper Functions
'''
import sys
import struct
import can

def selectCANsystem():
    if 'win' in sys.platform:
        device = 'pcan'           # The PCAN Drivers must be installed in Windows
        channel = 'PCAN_USBBUS1'  # Update this to your specific channel
    else:
        device = 'socketcan'
        channel = 'can1' # Change this if there are more than 1 CAN adapter
    bitrate = 250000   # Set the bitrate to 2500000 for all NMEA2000
    return can.interface.Bus(channel=channel, interface=device, bitrate=bitrate)

def parseCANid(id):
    priority =  (0x1C000000 & id) >> 26
    sa = 0xFF & id #Source Address
    PF = (0xFF0000 & id) >> 16 #PDU Format
    if PF < 0xF0: #Type 1 message
        da = (0xFF00 & id) >> 8 #Destination Address
        pgn = (0x3FF0000 & id) >> 8 #Parameter Group Number (18 bits)
    else: #Type 2 Message
        da = 0xFF #Global destination (any node)
        pgn = (0x3FFFF00 & id) >> 8 #Parameter Group Number (18 bits)
    return (priority, pgn, da, sa)

def interpretAddressName(can_data):
    description = "ISO Address Claim"
    print(description)

    ''' From Appendix D.2.3 ISO 1783-5 Conventions in the NMEA 2000 Standard:
+-------------------+----------+--------------+--------+----------+----------+----------+----------+--------------+------------------------+
|   Field 10        |  Field 9 |    Field 8   | Field 7| Field 6  | Field 5  | Field 4  | Field 3  |  Field 2     |      Field 1           |
+-------------------+----------+--------------+--------+----------+----------+----------+----------+--------------+------------------------+
| Self-Configurable | Industry | Device Class | Device | Reserved | Function | Function | ECU      | Manufacturer |      Identity          |
|     Address       |  Group   |   Instance   |  Class |          | Instance |          | Instance | Code         |       Number           |
+-------------------+----------+--------------+--------+----------+----------+----------+----------+--------------+------------------------+
|    1 bit          |  3 bits  |   4 bits     | 7 bits | 1 bit    | 8 bits   |  5 bits  |   3 bits |    11 bits   |       21 bits          |
+-------------------+----------+--------------+--------+----------+----------+----------+----------+---------+---------+---------+---------+
| 8                                      bit 1|8             bit 1|8    bit 1|8               bit 1|8   bit 1|8   bit 1|8   bit 1|8   bit 1|
+------------------------------+--------------+-------------------+----------+---------------------+---------+---------+---------+---------+
|                    Byte 8                   |      Byte 7       |  Byte 6  |        Byte 5       | Byte 4  | Byte 3  | Byte 2  | Byte 1  |
+------------------------------+--------------+-------------------+----------+---------------------+---------+---------+---------+---------+
|     can.message    data[7]                  |       data[6]     |  data[5] |        data[4]      | data[3] | data[2] | data[1] | data[0] |
+------------------------------+--------------+-------------------+----------+---------------------+---------+---------+---------+---------+

        '''

    #Convert the first 4 bytes to an integer for conversion
    unique_num_and_mfg_code = struct.unpack("<L",can_data[0:4])[0]     
    #Field 1: Unique Number
    unique_num = (unique_num_and_mfg_code & 0b00000000000111111111111111111111)
    #Field 2: Manufacturer Code
    mfg_code   = (unique_num_and_mfg_code & 0b11111111111000000000000000000000) >> 21
    
    #Get the device and ECU instance as an integer
    ecu_and_function_instance = can_data[4]
    #Field 3: Device Instance Lower (ISO ECU Instance)
    ecu_instance      = (ecu_and_function_instance & 0b11100000) >> 5
    #Field 4: 
    function_instance = (ecu_and_function_instance & 0b00011111)
    # The combination of fields 3 & 4 make up the 8 bit NMEA Device Instance. NMEA D evice Instance values are not
    # intended to be unique on the network. When NMEA Device Instances are configured, they should be unique within all
    # devices of the same Class & Function code.

    #Field 5: Device Function (NMEA Function Code)
    #Dependent on NMEA Device Class DD170, reference NMEA Class & Function Codes, Appendix B6.
    function_code = can_data[5]
    
    #Field 6: Reserved Bit (usually set to zero)
    #Field 7: Device Class
    device_class =  (can_data[6] & 0b11111110) >> 1

    #Field 8: System Instance (ISO Device Class Instance)
    system_instance = (can_data[7] & 0b00001111)
    
    #Field 9: Industry Group
    industry_group = (can_data[7] & 0b01110000) >> 4
    
    #Field 10: NMEA Reserved (ISO Self Configurable)
    self_configurable = bool(can_data[7] & 0b10000000) 
    
    print(f'''Address claimed:
+-------------------+----------+--------------+--------+----------+----------+----------+--------------+------------------------+
| Self-Configurable | Industry | Device Class | Device | Function | Function | ECU      | Manufacturer |      Identity          |
|     Address       |  Group   |   Instance   |  Class | Instance |          | Instance | Code         |       Number           |
+-------------------+----------+--------------+--------+----------+----------+----------+--------------+------------------------+
|     {str(self_configurable):5s}         |   {industry_group:3d}    |     {system_instance:3d}      |   {device_class:3d}  |   {function_instance:3d}    |   {function_code:3d}    |   {ecu_instance:3d}    |   {mfg_code:4d}       |      {unique_num:7d}           |
+-------------------+----------+--------------+--------+----------+----------+----------+--------------+------------------------+''')


def interpretTemperature(can_data):
    # this is for pgn == 130310
    # Look this up in the NMEA Network Message Database, Appendix B.1
        description = "Environmental Parameters"
        print(description)
        #Field #1
        SID = can_data[0] #Sequence ID
        
        #Field #2
        water_temp = struct.unpack("<H",can_data[1:3])[0] * 1e-2 #degrees Kelvin
        if water_temp > 655.32:
            water_temp = "Out of Range - High"
        
        #Field #3
        air_temp = struct.unpack("<H",can_data[3:5])[0] * 1e-2 #degrees Kelvin
        if air_temp > 655.32:
            air_temp = "Out of Range - High"
        
        #Field #4
        atmos_pressure = struct.unpack("<H",can_data[5:7])[0] * 1e2 #pascals
        if atmos_pressure > 6553200:
            atmos_pressure = "Out of Range - High"
        
        #Show results
        print(f"{SID}: Water Temp = {water_temp} deg K, Outside Ambient Air Temp = {air_temp} deg K, Atmospheric Pressure = {atmos_pressure} Pa")
    