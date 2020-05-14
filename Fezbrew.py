import serial
import os
import glob
import sys
import time
from influxdb import InfluxDBClient


# Create the InfluxDB client object
client = InfluxDBClient(host='localhost', port=8086, database='brewDB')

# Influxdb schema
measurement = 'Brew1'
location = 'Brew Fridge'

# Serial settings Arduino
ser = serial.Serial('/dev/ttyACM0', 9600)
ser.flushInput()
print('Beginning Brewing Program')

def read_temp():

    print('Waiting for Arduino')

    #Read Serial from Arduino
    line = ser.readline()
    line1 = str(line).encode("utf-8")
    
    print(line1)

    #Split into an array of two temperature readings
    temps = line1.split(',')
    temp_1_deg = float(temps[0][6:])/10
    temp_2_deg = float(temps[1][6:])/10
    ser.flushInput()
    
    return [temp_1_deg, temp_2_deg]

def ser_write():

    print('Writing to Arduino')

while True:
    try:                
        iso = time.ctime()

        readings = read_temp()
        # Create the JSON data structure
        data = [{
            "measurement": measurement,
                "tags": {
                    "location": location,
                    "units" : 'Degrees'
                },
                "time": iso,
                "fields": {
                    "temperature1" : readings[0],
                    "temperature2" : readings[1],
                }
            }]
        # Send the JSON data to InfluxDB
        client.write_points(data)
        
        print(iso + ' - Data Sent to InfluxDB')	
        time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("Exiting")
        sys.exit(0)

    except Exception as e:
        print("Received exception!")
        print(e)
        time.sleep(10)

