import os
import glob
import time
from influxdb import InfluxDBClient

#Set the GPIO pins on the pi
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

#Path for DS18B20 Sensors
temp1Path = '/sys/bus/w1/devices/28-0416a4347eff/w1_slave'
temp2Path = '/sys/bus/w1/devices/28-0516a4c621ff/w1_slave'

# Create the InfluxDB client object
client = InfluxDBClient(host='localhost', port=8086, database='brewDB')

# Influxdb schema
measurement = 'Brew1'
location = 'Brew Fridge'

device_file = [temp1Path, temp2Path]
print(device_file)


def read_temp_raw():
        f_1 = open(device_file[0], 'r')
	lines_1 = f_1.readlines()
	f_1.close()
	f_2 = open(device_file[1], 'r')
	lines_2 = f_2.readlines()
	f_2.close()
	return lines_1 + lines_2
 
def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES' and lines[2].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    temp1pos = lines[1].find('t=')
    temp2pos = lines[3].find('t=')
    
    temp_1_deg = float(lines[1][temp1pos+2:])/1000.0
    temp_2_deg = float(lines[3][temp2pos+2:])/1000.0
    return [temp_1_deg, temp_2_deg]

try:
        while True:
                iso = time.ctime()
                # Create the JSON data structure
                data = [
                {
                  "measurement": measurement,
                      "tags": {
                          "location": location,
                          "units" : 'Degrees'
                      },
                      "time": iso,
                      "fields": {
                          "temperature1" : read_temp()[0],
                          "temperature2" : read_temp()[1],
                      }
                  }
                ]
                # Send the JSON data to InfluxDB
                client.write_points(data)
                
                print(read_temp())	
                time.sleep(5)
                
except KeyboardInterrupt:
        pass