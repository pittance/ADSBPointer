#!/usr/bin/env python3
import serial
import time
import json
import urllib.request
import math
import time


def haversine(lat1, lon1, lat2, lon2):
    R = 6372784.15424 # this is in metres.  For Earth radius in kilometers use 6372.8 km
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    a = math.sin(dLat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dLon/2)**2
    c = 2*math.asin(math.sqrt(a))
    return R * c

def haversine2(lat1,lon1,lat2,lon2):
    R = 6371e3 ## metres
    rLat1 = math.radians(lat1)
    rLat2 = math.radians(lat2)
    dLat = math.radians(lat2-lat1)
    dLon = math.radians(lon2-lon1)
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(rLat1) * math.cos(rLat2) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def elevation(altFt,dist):
    elev = math.degrees(math.atan2(altFt*0.3048,dist))
    return elev

def calculateDistance(x1,y1,x2,y2):
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist

def calculateBearing(x1,y1,x2,y2):
    bearing = math.atan2(y1-y2,x1-x2)
    bearing = (-bearing + math.pi/2)%(2*math.pi)
    return bearing

def calculateBearing2(lat1, lon1, lat2, lon2):
    ##assumes inputs in degrees, convert to radians
    ##from lat1/lon1 to lat2/lon2
    rLat1 = math.radians(lat1)
    rLon1 = math.radians(lon1)
    rLat2 = math.radians(lat2)
    rLon2 = math.radians(lon2)
    y = math.sin(rLon2-rLon1) * math.cos(rLat2)
    x = math.cos(rLat1)*math.sin(rLat2) - math.sin(rLat1) * math.cos(rLat2) * math.cos(rLon2-rLon1)
    theta = math.atan2(y, x)
    brng = (theta*180/math.pi + 360) % 360
    ##return is in degrees
    return brng

def runIt(myLat,myLon):
    ##myLat = 51.46063
    ##myLon = -2.53518

    ##load the aircraft data from the 1090 json source
    ##with urllib.request.urlopen("http://192.168.0.52/dump1090-fa/data/aircraft.json") as url
    
    try:
        data = urllib.request.urlopen("http://127.0.0.1/dump1090-fa/data/aircraft.json")
        jsonData = json.loads(data.read().decode())
        ##loop through aircraft in the JSON list
        minDist = 100000
        minAc = ""
        foundAc = False
        noLatLon = False
        foundAcAge = 1000
        for ac in data['aircraft']:
            try:
                thisDist = calculateDistance(ac['lon'],ac['lat'],myLon,myLat)
                ##print(ac['hex'],ac['seen'],ac['lat'],ac['lon'],thisDist)
                if (thisDist < minDist):
                    foundAc = True
                    minAc = ac
                    minDist = thisDist
                    foundAcAge = ac['seen']
            except:
                noLatLon = True
        if foundAc:
            ##print(minAc)
            ##print("haversine formula result: ",haversine(minAc['lon'],minAc['lat'],myLon,myLat),"m")
            br = calculateBearing2(myLat,myLon,minAc['lat'],minAc['lon'])
            el = elevation(minAc['alt_baro'],haversine2(myLat,myLon,minAc['lat'],minAc['lon']))
            print("icao bearing & elevation: ",minAc['hex'],br,"/",el)
            return str(br) + "," + str(el) + "," + str(minAc['hex'])
        else:
            print("no aircraft with lat/lon data present")
            return "0.0,0.0,aaaaa"


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5)
    ser.flush()
	##set default GPS coords to our house
	gpsLat = 51.46063
	gpsLon = -2.53518
    ##check for the connection
    connected = False
    while not connected:
        try:
            urllib.request.urlopen("http://127.0.0.1/dump1090-fa/data/aircraft.json")
            connected = True
            print('Connection to dump1090 found')
        except urllib.error.URLError as e:
            print(e.reason)
            time.sleep(3)

    while True:
		##runs continuously (speed limited by sleep)
		##	ardData is sent to Arduino, the data comes from the runIt() function
        ardData = runIt(gpsLat,gpsLon) + "\n"
		## write the data to serial (USB)
        ser.write(ardData.encode())
		## read any data coming back from Arduino
        line = ser.readline().decode('utf-8').rstrip()
        lineArray = line.split(",")
        gpsLat = float(lineArray[0])
        gpsLon = float(lineArray[1])
        time.sleep(2)
