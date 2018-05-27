#!/usr/bin/python
import serial
from time import sleep
import RPi.GPIO as GPIO

GPS_UART='/dev/serial0'
GPS_INITIAL_BAUDRATE=9600
GPS_QUICK_RESET=True

GPS_GPIO_MAP = {
    "ENABLE"    : 4, # PULSE PIN LOW TO RESET DEVICE
    "PULSE"     : 18
}

UBLOX_CMD = {
    'SET_GPS_10Hz'      : 'B562060806006400010001007A12B562060800000E30',
    'SET_GPS_p33Hz'     : 'B56206080600B80B01000100D941B562060800000E30',
    'SET_BAUD_115200'   : 'b5620600140001000000d008000000c201000700070000000000c496b56206000100010822',
    'SET_BAUD_9600'     : 'b5620600140001000000d0080000802500000700070000000000a6cdb56206000100010822',
    # DEVICE CONTROL
    'ISSUE_RESET'       : 'B56206040400FF87010095F7',
    'ISSUE_COLD_START'  : 'B56206040400FFFF02000E61',
    'ISSUE_HOT_START'   : 'B56206040400000002001068',
    'ISSUE_SLEEP'       : 'B56206040400000008001674',
    # EEPROM FUNCTIONS
    'EEPROM_SAVE'       : 'B56206090D0000000000FFFF0000000000001731BF',
    'EEPROM_DEFAULT'    : 'B56206090D00FFFF000000000000FFFF0000071F9E',
    # DISABLE OUTPUTS
    "DISABLE_GPDTM" : "2445494750512c44544d2a33420d0ab56206010300f00a000423",
    "DISABLE_GPGBS" : "2445494750512c4742532a33300d0ab56206010300f009000321",
    "DISABLE_GPGGA" : "2445494750512c4747412a32370d0ab56206010300f00000fa0f",
    "DISABLE_GPGLL" : "2445494750512c474c4c2a32310d0ab56206010300f00100fb11",
    "DISABLE_GPGRS" : "2445494750512c4752532a32300d0ab56206010300f00600001b",
    "DISABLE_GPGSA" : "2445494750512c4753412a33330d0ab56206010300f00200fc13",
    "DISABLE_GPGST" : "2445494750512c4753542a32360d0ab56206010300f00700011d",
    "DISABLE_GPGSV" : "2445494750512c4753562a32340d0ab56206010300f00300fd15",
    "DISABLE_GPRMC" : "2445494750512c524d432a33410d0ab56206010300f00400fe17",
    "DISABLE_GPVTG" : "2445494750512c5654472a32330d0ab56206010300f00500ff19",
    "DISABLE_GPZDA" : "2445494750512c5a44412a33390d0ab56206010300f00800021f"
}

ser_gps = None;
gps_command_count = 0

def reset_gps_device(ser):
    sent_command_count = 0
    GPIO.output(GPS_GPIO_MAP['ENABLE'], GPIO.LOW)
    sleep(0.3)
    GPIO.output(GPS_GPIO_MAP['ENABLE'], GPIO.HIGH)
    sleep(0.3)
#

def _getKeyByVal(mydict,myval):
    return mydict.keys()[mydict.values().index(myval)]
#

def send_ublox_command(ser,command):
    if(command == UBLOX_CMD['ISSUE_RESET']):
        print "RESETTING %s" % GPS_UART
        reset_gps_device(ser)
        if(GPS_QUICK_RESET):
            return 0
        #
    #
    ser.write("\r\n")
    sleep(0.3)
    print "Sending UBLOX command: %s" % _getKeyByVal(UBLOX_CMD,command)
    ser.write(command.decode("hex"))
    ser.write("\r\n")
    #
    sleep(0.1)
    return 0
    
#

def connect_gps(uartString,initialBaudrate):
    gpsSerial = serial.Serial(uartString, initialBaudrate, timeout=1, parity=serial.PARITY_NONE, rtscts=0)
    gpsSerial.baudrate=9600
    send_ublox_command(gpsSerial,UBLOX_CMD['ISSUE_RESET'])
    #
    send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPDTM"])
    send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPGBS"])
    #send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPGGA"]) 
    #send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPGLL"]) 
    send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPGRS"]) 
    send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPGSA"]) 
    send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPGST"]) 
    send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPGSV"]) 
    send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPRMC"]) 
    send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPVTG"]) 
    send_ublox_command(gpsSerial,UBLOX_CMD["DISABLE_GPZDA"]) 
    #
    send_ublox_command(gpsSerial,UBLOX_CMD['SET_BAUD_115200'])
    gpsSerial.baudrate=115200
    #
    send_ublox_command(gpsSerial,UBLOX_CMD['SET_GPS_10Hz'])
    #
    ser_gps = gpsSerial;
    return gpsSerial
    #
#

def configure_gpios():
    # Disable annoying messages
    GPIO.setwarnings(False)
    # Set GPIO mode to the BCM numbering scheme.
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPS_GPIO_MAP['ENABLE'], GPIO.OUT)
    GPIO.setup(GPS_GPIO_MAP['PULSE'],  GPIO.IN)
#

def main():
    configure_gpios()
    with connect_gps(GPS_UART,GPS_INITIAL_BAUDRATE) as ser_gps:
        ser_gps.flushInput()
        while(True):
            if(ser_gps.inWaiting() > 0):
                line = ser_gps.readline()
                print line.rstrip()
            #
            sleep(0.001)
        #
    #
    ser_gps.close()
    return 0
#

if __name__ == "__main__":
    main()
#

"""
GGA: time, position, position type 
GLL: latitude, longitude, UTC 
GSA: GPS receiver operating mod, satellites for positioning, DOP value 
GSV: Available GPS satellites information, azimuth, elevation, SNR 
RMC: time, date, position, speed 
VTG: the speed information on ground MSS: signal strength
"""
keys = "INDEX","LAPINDEX","DATE","TIME","TIME_LAP","LATITUDE","LONGITUDE","SPEED_KPH","SPEED_MPH","HEIGHT_M","HEIGHT_FT","HEADING_DEG","GPSDIFFERENTIAL[UNKNOWN/2D3D/DGPS/INVALID]","GPSFIX[NOFIX/2D/3D/UNKNOWN]","SATELLITES","HDOP","ACCURACY_M","DISTANCE_KM","DISTANCE_MILE","ACCELERATIONSOURCE[CALCULATED/MEASURED/UNDEFINED]","LATERALG","LINEALG","LEAN","RPM","MAF","WHEEL_SPEED_KPH","WHEEL_SPEED_MPH","THROTTLE","GEAR","FUEL","COOLANT_CELSIUS","OIL_CELSIUS","IAT_CELSIUS","MAP"
values = "10975","37","17-MAY-18","16:16:05.63","0.000000","47.254634","-123.193048","69.100000","42.936749","96.400000","316.272966","90.4","0","3","8","0.660000","12.0","0.000000","0.000000","0","0.09","0.38","5.100000","3624","113.270000","74.000000","45.981468","0.700000","0","0.970000","86.000000","0.000000","25.000000","1.430000"
