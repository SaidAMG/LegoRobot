#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialiseer de EV3 Brick.
ev3 = EV3Brick()

# Initialiseer de ultrasonic sensor. Het wordt gebruikt om obstakels te detecteren terwijl de robot rondrijdt.
# Initialiseer de kleuren sensor, om de vrije doorgangen naar links te detecteren.
# Initialiseer de gyro sensor. Het wordt gebruikt om een exacte 90 graden draai te creëren.
obstacle_sensor = UltrasonicSensor(Port.S4)
side_sensor = ColorSensor(Port.S2)
gyro_sensor = GyroSensor(Port.S1)

# Initialiseer twee motoren met standaardinstellingen op poort A en poort D. Dit zijn de linker- en rechtermotoren van de aandrijfbasis.
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

# De DriveBase bestaat uit twee motoren, met op elke motor een wiel.
# De waarden wheel_diameter en axle_track worden gebruikt om de motoren te laten beweeg met de juiste snelheid als je een motorcommando geeft.
# De axle_track is de afstand tussen de 2 wielen.
# De turn_rate beslist de snelheid van de draai.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=154.5)
robot.settings(turn_rate=30)

# Speel een geluid af om aan te geven dat die gaat beginnen met rijden.
ev3.speaker.beep()

#------------------------------------------------------ Start code door Haci Candan
# Hier wordt een textbestand aangemaakt die zal bijhouden welke stappen de robot heeft gemaakt zodat het later terug gelezen kan worden.
kaart = open('kaart.txt','w')
kaart.write('Ingang.\n')
#------------------------------------------------------ Eind code door Haci Candan

gyro_sensor.reset_angle(0) # reset gyro sensor naar 0 grade.

#------------------------------------------------------ Start code door Saïd Amghar
def RijVooruit():
    kaart.write('Ga rechtdoor.\n') # Toegevoegd door Haci Candan
    while True:
        robot.drive(200,0)

        while obstacle_sensor.distance() < 150 or side_sensor.reflection() == 0:
            wait(10)
            if obstacle_sensor.distance() < 150:
                kaart.write('Obstakel Gedetecteerd.\n') # Toegevoegd door Haci Candan
                ObstakelMijden() # Functie door Saïd Amghar

            elif side_sensor.reflection() == 0:
                kaart.write('Pad naar links gevonden.\n')
                WegNaarLinks() # Functie door Haci Candan
#------------------------------------------------------ Eind code door Saïd Amghar

#------------------------------------------------------ Start code door Dennis Buckers
def gyro_draai():
    if side_sensor.reflection() == 0:
        while gyro_sensor.angle() != -90:
            robot.turn(-90-gyro_sensor.angle())
    else:
        while gyro_sensor.angle() != 90:
            robot.turn(90-gyro_sensor.angle())
    gyro_sensor.reset_angle(0)
#------------------------------------------------------ Eind code door Dennis Buckers

#------------------------------------------------------ Start code door Saïd Amghar
def ObstakelMijden():
    if side_sensor.reflection() == 0:
        kaart.write('Draai naar 90 grade naar links.\n') # Toegevoegd door Haci Candan
        kaart.write('Ga naar links.\n') # Toegevoegd door Haci Candan
        gyro_draai() # Functie door Dennis Buckers 

    else:
        kaart.write('Draai naar 90 grade naar rechts.\n') # Toegevoegd door Haci Candan
        kaart.write('Ga naar rechts.\n') # Toegevoegd door Haci Candan
        gyro_draai() # Functie door Dennis Buckers
        
#------------------------------------------------------ Eind code door Saïd Amghar

#------------------------------------------------------ Start code door Haci Candan
def WegNaarLinks():
    if obstacle_sensor.distance() > 150:
        kaart.write('Ga rechtdoor.\n') # Toegevoegd door Haci Candan
        robot.straight(140)
        while obstacle_sensor.distance() > 160:
            robot.drive(100,0)
    gyro_draai() # Functie door Dennis Buckers 
    kaart.write('Draai naar 90 grade naar links.\n') # Toegevoegd door Haci Candan
    kaart.write('Ga naar links.\n') # Toegevoegd door Haci Candan

    if obstacle_sensor.distance() > 150:
        kaart.write('Ga rechtdoor.\n') # Toegevoegd door Haci Candan
        robot.straight(300)
#------------------------------------------------------ Eind code door Haci Candan

RijVooruit() # Functie door Saïd Amghar 