#!/usr/bin/env pybricks-micropython

from math import pi

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch, DataLog

ev3 = EV3Brick()

# Hardwarezuordnung aus Sicht des fahrenden Roboters:
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
color_sensor = ColorSensor(Port.S1)

# Sensor-Kalibrierung für den einfachen PD-Regler:
BLACK = 8
WHITE = 62
TARGET = 35

# Einfacher PD-Regler:
BASE_SPEED = 90
KP = 3.0
KD = 0.25

LOOP_TIME_MS = 20
DERIVATIVE_FILTER = 0.7
MAX_MOTOR_SPEED = 300


# ------------------------------------------------------------
# AUSWERTUNGS-LOG
# ------------------------------------------------------------

# Beim Abstimmen auf False lassen.
# Für offizielle Vergleichsfahrten auf True setzen.
ENABLE_SUMMARY_LOG = True

# Standard-EV3-Rad. Bei anderen Rädern hier anpassen.
WHEEL_DIAMETER_MM = 50

# Exakte Länge deiner standardisierten Messstrecke eintragen.
# Beispiel: 2000 für eine 2-m-Strecke.
TRACK_LENGTH_MM = 4800

CONTROLLER_NAME = "pd_basic"


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


# Warten, bis eine beim Programmstart noch gedrückte Taste losgelassen wird.
while ev3.buttons.pressed():
    wait(10)

ev3.speaker.beep()
wait(1000)

# Startwerte des Reglers:
reflection = color_sensor.reflection()
last_error = reflection - TARGET
filtered_derivative = 0

# Startwerte für die Messung:
completed = 0

if ENABLE_SUMMARY_LOG:
    summary_log = DataLog(
        "controller",
        "target",
        "kp",
        "kd",
        "base_speed",
        "track_length_mm",
        "completed",
        "duration_ms",
        "distance_mm",
        "mean_speed_mm_s",
        "track_speed_mm_s",
        "extra_distance_mm",
        "mean_abs_error",
        "rms_error",
        "max_abs_error",
        "mean_abs_turn",
        "mean_turn_change",
        "turn_direction_changes",
        name="pd_basic_summary",
        timestamp=False,
        append=True
    )

    watch = StopWatch()

    previous_left_angle = left_motor.angle()
    previous_right_angle = right_motor.angle()

    left_distance_deg = 0
    right_distance_deg = 0

    sample_count = 0
    sum_abs_error = 0
    sum_squared_error = 0
    max_abs_error = 0

    sum_abs_turn = 0
    sum_turn_change = 0
    previous_turn_speed = 0

    last_turn_sign = 0
    turn_direction_changes = 0


try:
    while True:

        # Mitte = Strecke erfolgreich beendet.
        if Button.CENTER in ev3.buttons.pressed():
            completed = 1
            break

        # Linke Taste = Linienverlust / Fahrt abgebrochen.
        if Button.LEFT in ev3.buttons.pressed():
            completed = 0
            break

        # 1. Sensorwert und Fehler bestimmen
        reflection = color_sensor.reflection()
        error = reflection - TARGET

        # 2. D-Anteil bestimmen und glätten
        raw_derivative = (
            (error - last_error) / (LOOP_TIME_MS / 1000)
        )

        filtered_derivative = (
            DERIVATIVE_FILTER * filtered_derivative
            + (1 - DERIVATIVE_FILTER) * raw_derivative
        )

        # 3. Klassischer PD-Regler:
        #
        # e = reflection - TARGET
        # turn_speed = -(KP * e + KD * de/dt)
        p_part = KP * error
        d_part = KD * filtered_derivative

        turn_speed = -(p_part + d_part)

        # 4. Motoren ansteuern
        drive(BASE_SPEED, turn_speed)

        # 5. Nur bei aktivierter Auswertung Kennwerte sammeln
        if ENABLE_SUMMARY_LOG:

            current_left_angle = left_motor.angle()
            current_right_angle = right_motor.angle()

            left_distance_deg += abs(
                current_left_angle - previous_left_angle
            )

            right_distance_deg += abs(
                current_right_angle - previous_right_angle
            )

            previous_left_angle = current_left_angle
            previous_right_angle = current_right_angle

            abs_error = abs(error)

            sample_count += 1
            sum_abs_error += abs_error
            sum_squared_error += error * error

            if abs_error > max_abs_error:
                max_abs_error = abs_error

            sum_abs_turn += abs(turn_speed)
            sum_turn_change += abs(
                turn_speed - previous_turn_speed
            )

            # Kleine Werte ignorieren, damit Messrauschen nicht zählt.
            if turn_speed > 10:
                current_turn_sign = 1
            elif turn_speed < -10:
                current_turn_sign = -1
            else:
                current_turn_sign = 0

            if (
                current_turn_sign != 0
                and last_turn_sign != 0
                and current_turn_sign != last_turn_sign
            ):
                turn_direction_changes += 1

            if current_turn_sign != 0:
                last_turn_sign = current_turn_sign

            previous_turn_speed = turn_speed

        last_error = error
        wait(LOOP_TIME_MS)

finally:
    stop()

    if ENABLE_SUMMARY_LOG and sample_count > 0:

        duration_ms = watch.time()

        # Durchschnittlicher Encoderweg beider Räder:
        mean_motor_angle = (
            left_distance_deg + right_distance_deg
        ) / 2

        distance_mm = (
            mean_motor_angle / 360
        ) * pi * WHEEL_DIAMETER_MM

        duration_s = duration_ms / 1000

        mean_speed_mm_s = distance_mm / duration_s

        mean_abs_error = sum_abs_error / sample_count
        rms_error = (sum_squared_error / sample_count) ** 0.5

        mean_abs_turn = sum_abs_turn / sample_count
        mean_turn_change = sum_turn_change / sample_count

        # Nur bei erfolgreicher Fahrt sinnvoll:
        if completed == 1 and TRACK_LENGTH_MM > 0:
            track_speed_mm_s = TRACK_LENGTH_MM / duration_s
            extra_distance_mm = distance_mm - TRACK_LENGTH_MM
        else:
            track_speed_mm_s = -1
            extra_distance_mm = -1

        summary_log.log(
            CONTROLLER_NAME,
            TARGET,
            KP,
            KD,
            BASE_SPEED,
            TRACK_LENGTH_MM,
            completed,
            duration_ms,
            distance_mm,
            mean_speed_mm_s,
            track_speed_mm_s,
            extra_distance_mm,
            mean_abs_error,
            rms_error,
            max_abs_error,
            mean_abs_turn,
            mean_turn_change,
            turn_direction_changes
        )

        print("Ergebnis gespeichert")

ev3.speaker.beep()