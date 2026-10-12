#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait
import random


# =================================================
# 1. HARDWARE
# =================================================

ev3 = EV3Brick()

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

sensor = ColorSensor(Port.S1)


# =================================================
# 2. GRUNDEINSTELLUNGEN
# =================================================

TARGET = 35

BASE_SPEED = 100
TURN_LIGHT = 40
TURN_STRONG = 80

ACTION_TIME = 50       # ms

N_STATES = 7
N_ACTIONS = 5

ALPHA = 0.1
GAMMA = 0.9

# 0.0 = keine zufälligen Aktionen
EPSILON = 0.0

# True  = auf dem echten EV3 weiterlernen
# False = Tabelle nur benutzen
LEARN = True


# =================================================
# 3. LINIENVERLUST
# =================================================

# So viele extreme Sensorwerte hintereinander
# -> Linie gilt als verloren
LINE_LOST_COUNT = 10

# Strafe beim Abflug
LINE_LOST_REWARD = -100


# =================================================
# 4. DATEI FÜR GELERNTE Q-TABELLE
# =================================================

Q_FILE = "/home/robot/q_table.txt"


# =================================================
# 5. START-Q-TABELLE AUS DER SIMULATION
# =================================================

# Spalten:
# 0 = stark links
# 1 = leicht links
# 2 = gerade
# 3 = leicht rechts
# 4 = stark rechts

Q = [
    [-37.77036890214506,
     -36.93927020385523,
     -42.791244456495505,
     -37.77036890214506,
     10.629940411908205],

    [0.0,
     0.0,
     0.0,
     0.0,
     53.93693912012508],

    [0.0,
     0.0,
     0.0,
     0.0,
     56.72919294797906],

    [0.0,
     57.502983569725544,
     0.0,
     0.0,
     0.0],

    [0.0,
     0.0,
     26.075255556711756,
     -0.10731180047813593,
     0.0],

    [0.0,
     9.62783497360035,
     0.0,
     0.0,
     0.0],

    [-12.514467722890968,
     -32.68676730585121,
     -32.64365119124008,
     -31.873127511266027,
     -32.346262402386046]
]


# =================================================
# 6. Q-TABELLE LADEN
# =================================================

def load_q_table():

    try:

        file = open(Q_FILE, "r")

        loaded_q = []

        for line in file:

            values = line.strip().split(",")

            row = []

            for value in values:
                row.append(float(value))

            loaded_q.append(row)

        file.close()


        # Prüfen, ob Tabelle richtige Größe hat
        if len(loaded_q) == N_STATES:

            valid = True

            for row in loaded_q:

                if len(row) != N_ACTIONS:
                    valid = False


            if valid:

                print("Gespeicherte Q-Tabelle geladen")

                return loaded_q


    except:

        pass


    print("Keine gespeicherte Q-Tabelle gefunden")
    print("Simulationstabelle wird verwendet")


    # Kopie von Q_START erstellen
    q = []

    for row in Q_START:
        q.append(row[:])

    return q


# =================================================
# 7. Q-TABELLE SPEICHERN
# =================================================

def save_q_table():

    file = open(Q_FILE, "w")


    for row in Q:

        line = ""

        for i in range(len(row)):

            line = line + str(row[i])

            if i < len(row) - 1:
                line = line + ","


        file.write(line + "\n")


    file.close()

    print("Q-Tabelle gespeichert")


# =================================================
# 8. Q-TABELLE BEIM START LADEN
# =================================================

#Q = load_q_table()


# =================================================
# 9. ZUSTAND
# =================================================

def get_state(error):

    if error < -20:
        return 0

    elif error < -10:
        return 1

    elif error < -3:
        return 2

    elif error <= 3:
        return 3

    elif error <= 10:
        return 4

    elif error <= 20:
        return 5

    else:
        return 6


# =================================================
# 10. REWARD
# =================================================

def get_reward(error):

    reward = 10 - abs(error)

    return reward


# =================================================
# 11. AKTION AUSWÄHLEN
# =================================================

def choose_action(state):

    # Exploration
    if random.random() < EPSILON:

        action = random.randint(
            0,
            N_ACTIONS - 1
        )

    # Exploitation
    else:

        action = Q[state].index(
            max(Q[state])
        )

    return action


# =================================================
# 12. AKTION AUSFÜHREN
# =================================================

def execute_action(action):

    if action == 0:

        turn_speed = TURN_STRONG


    elif action == 1:

        turn_speed = TURN_LIGHT


    elif action == 2:

        turn_speed = 0


    elif action == 3:

        turn_speed = -TURN_LIGHT


    elif action == 4:

        turn_speed = -TURN_STRONG


    else:

        turn_speed = 0


    left_speed = (
        BASE_SPEED
        - turn_speed
    )

    right_speed = (
        BASE_SPEED
        + turn_speed
    )


    left_motor.run(left_speed)
    right_motor.run(right_speed)


    wait(ACTION_TIME)


# =================================================
# 13. HAUPTSCHLEIFE
# =================================================

lost_counter = 0


while True:

    # ---------------------------------------------
    # Manuelles Beenden mit Mitteltaste
    # ---------------------------------------------

    if Button.CENTER in ev3.buttons.pressed():

        print("Manuell beendet")

        break


    # ---------------------------------------------
    # Aktuellen Zustand bestimmen
    # ---------------------------------------------

    reflection = sensor.reflection()

    error = TARGET - reflection

    state = get_state(error)


    # ---------------------------------------------
    # Aktion auswählen
    # ---------------------------------------------

    action = choose_action(state)


    # ---------------------------------------------
    # Aktion ausführen
    # ---------------------------------------------

    execute_action(action)


    # ---------------------------------------------
    # Neuen Zustand bestimmen
    # ---------------------------------------------

    next_reflection = sensor.reflection()

    next_error = (
        TARGET
        - next_reflection
    )

    next_state = get_state(
        next_error
    )


    # ---------------------------------------------
    # Linienverlust prüfen
    # ---------------------------------------------

    if (
        next_reflection <= 10
        or
        next_reflection >= 60
    ):

        lost_counter += 1

    else:

        lost_counter = 0


    # ---------------------------------------------
    # Linie verloren
    # ---------------------------------------------

    if lost_counter >= LINE_LOST_COUNT:

        print("Linie verloren")


        # Terminales Q-Learning-Update
        if LEARN:

            old_q = Q[state][action]

            Q[state][action] = (
                old_q
                + ALPHA
                * (
                    LINE_LOST_REWARD
                    - old_q
                )
            )


        break


    # ---------------------------------------------
    # Normales Q-Learning
    # ---------------------------------------------

    if LEARN:

        reward = get_reward(
            next_error
        )


        old_q = Q[state][action]


        max_next_q = max(
            Q[next_state]
        )


        target_q = (
            reward
            + GAMMA
            * max_next_q
        )


        Q[state][action] = (
            old_q
            + ALPHA
            * (
                target_q
                - old_q
            )
        )


# =================================================
# 14. MOTOR STOPPEN
# =================================================

left_motor.stop()
right_motor.stop()

ev3.speaker.beep()


# =================================================
# 15. Q-TABELLE SPEICHERN
# =================================================

if LEARN:

    save_q_table()


# =================================================
# 16. Q-TABELLE AUSGEBEN
# =================================================

print("")
print("Q-Tabelle nach der Fahrt:")

for row in Q:
    print(row)