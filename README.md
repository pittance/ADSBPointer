# ADSBPointer

Files for use on a Raspberry Pi in conjunction with FlightAware's RPi implementation PiAware

The script takes data from the server on the Pi and processes it to look for the nearest aircraft that can be detected from the ADSB-Out receiver
Once the nearest aircraft is determined the elevation and bearing to the aircraft are calculated
A serial connection to an attached Arduino Mega is established
To calculate bearing and elevation the position of the receiver is required - this is found from a GPS receiver attached to the Arduino or hard-coded in the file
Once the angles are established they are sent to the Arduino every 2 seconds, this controls stepper motors to point at the aircraft
