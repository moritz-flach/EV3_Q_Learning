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

# Kalibrierung:
BLACK = 8
WHITE = 62

# Rechte Kante der weißen Linie:
TARGET = 32

# PD-Parameter:
KP = 3.0
KD = 0.25

# Nichtlinearer Zusatz:
K_NONLINEAR = 0.03

# Variable Geschwindigkeit:
MAX_BASE_SPEED = 200
MIN_BASE_SPEED = 90

SPEED_REDUCTION_ERROR = 0.30
TURN_SPEED_REDUCTION = 0.20

# Regelzyklus und D-Filter:
LOOP_TIME_MS = 20
DERIVATIVE_FILTER = 0.7

# Sicherheitsgrenzen:
MAX_MOTOR_SPEED = 300
MAX_TURN_SPEED = 120


def drive(base_speed, turn_speed):
    # turn_speed = 0: geradeaus
    # turn_speed > 0: Linkskurve
    # turn_speed < 0: Rechtskurve

    left_turn_speed = base_speed - turn_speed
    right_turn_speed = base_speed + turn_speed

    left_turn_speed = max(
        -MAX_MOTOR_SPEED,
        min(MAX_MOTOR_SPEED, left_turn_speed)
    )

    right_turn_speed = max(
        -MAX_MOTOR_SPEED,
        min(MAX_MOTOR_SPEED, right_turn_speed)
    )

    left_motor.run(left_turn_speed)
    right_motor.run(right_turn_speed)


def stop():
    left_motor.stop()
    right_motor.stop()


ev3.speaker.beep()

# Initialisierung ohne künstlichen D-Sprung beim Start:
reflection = color_sensor.reflection()
last_error = reflection - TARGET
filtered_derivative = 0

try:
    while Button.CENTER not in ev3.buttons.pressed():

        # 1. Sensorwert und Fehler bestimmen
        reflection = color_sensor.reflection()
        error = reflection - TARGET

        # 2. Fehleränderung bestimmen und glätten
        raw_derivative = (error - last_error) / (LOOP_TIME_MS / 1000)

        filtered_derivative = (
            DERIVATIVE_FILTER * filtered_derivative
            + (1 - DERIVATIVE_FILTER) * raw_derivative
        )

        # 3. PD-Regler mit nichtlinearem Zusatz:
        #
        # e = reflection - TARGET
        # turn_speed = -(KP * e + KD * de/dt + K_NONLINEAR * e * |e|)
        #
        # P: korrigiert den aktuellen Fehler.
        # D: dämpft schnelles Weglaufen und Pendeln.
        # Nichtlinearer Anteil: starkes Gegenlenken bei großem Fehler.
        p_part = KP * error
        d_part = KD * filtered_derivative
        nonlinear_part = K_NONLINEAR * error * abs(error)

        turn_speed = -(p_part + d_part + nonlinear_part)

        turn_speed = max(
            -MAX_TURN_SPEED,
            min(MAX_TURN_SPEED, turn_speed)
        )

        # 4. Variable Geschwindigkeit:
        # Große Fehler oder starke Lenkung bedeuten Kurve -> langsamer.
        base_speed = (
            MAX_BASE_SPEED
            - SPEED_REDUCTION_ERROR * abs(error)
            - TURN_SPEED_REDUCTION * abs(turn_speed)
        )

        base_speed = max(
            MIN_BASE_SPEED,
            min(MAX_BASE_SPEED, base_speed)
        )

        # 5. Motoren ansteuern
        drive(base_speed, turn_speed)

        last_error = error
        wait(LOOP_TIME_MS)

finally:
    stop()

ev3.speaker.beep()