#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait

ev3 = EV3Brick()

# Hardwarezuordnung aus Sicht des fahrenden Roboters:
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
color_sensor = ColorSensor(Port.S1)

# PD-Regler:
# e(k) = y(k(gemessen)) - y_soll
# u(k) = -(KP * e(k) + KD * de(k)/dt) konstanter anteil mal fehler plus derivater fehler
#
# P-Anteil: korrigiert den aktuellen Abstand zur Linienkante.
#           Größeres KP = stärkere Rücklenkung.
# D-Anteil: reagiert darauf, wie schnell der Fehler wächst.
#           Er dämpft Pendeln und hilft beim Einlenken in Kurven.
#
# Das Minuszeichen gilt für unsere Konvention:
# positive turn_speed = Linkskurve, negative turn_speed = Rechtskurve.

# Kalibrierung des Farbsensors:
BLACK = 8
WHITE = 62
TARGET = 25

# Anfangsparameter: bewusst langsam und vorsichtig
BASE_SPEED = 200
KP = 3.0
KD = 0.25

LOOP_TIME_MS = 10
MAX_SPEED = 300

# Glättung für den D-Anteil:
DERIVATIVE_FILTER = 0.7


def drive(base_speed, turn_speed):
    # turn_speed = 0: geradeaus
    # turn_speed > 0: Linkskurve
    # turn_speed < 0: Rechtskurve

    left_turn_speed = base_speed - turn_speed
    right_turn_speed = base_speed + turn_speed

    left_turn_speed = max(-MAX_SPEED, min(MAX_SPEED, left_turn_speed))
    right_turn_speed = max(-MAX_SPEED, min(MAX_SPEED, right_turn_speed))

    left_motor.run(left_turn_speed)
    right_motor.run(right_turn_speed)


def stop():
    left_motor.stop()
    right_motor.stop()


ev3.speaker.beep()

reflection = color_sensor.reflection()
last_error = reflection - TARGET
filtered_derivative = 0

try:
    while Button.CENTER not in ev3.buttons.pressed():

        # 1. Sensorwert einlesen
        reflection = color_sensor.reflection()

        # 2. Fehler zur gewünschten Linienkante bestimmen
        error = reflection - TARGET

        # 3. D-Anteil: Wie schnell verändert sich der Fehler?
        raw_derivative = (error - last_error) / (LOOP_TIME_MS / 1000)

        # Messrauschen etwas glätten
        filtered_derivative = (
            DERIVATIVE_FILTER * filtered_derivative
            + (1 - DERIVATIVE_FILTER) * raw_derivative
        )

        # 4. PD-Regler berechnen
        # Minuszeichen: Wir folgen der rechten Kante der weißen Linie.
        turn_speed = -(KP * error + KD * filtered_derivative)

        # 5. Motoren ansteuern
        drive(BASE_SPEED, turn_speed)

        last_error = error
        wait(LOOP_TIME_MS)

finally:
    stop()

ev3.speaker.beep()