import math
import matplotlib.pyplot as plt
import random


# =================================================
# 1. GRUNDEINSTELLUNGEN
# =================================================

BLACK = 8
WHITE = 62
TARGET = 35

SENSOR_TRANSITION = 1.0   # cm

BASE_SPEED = 100
TURN_LIGHT = 40
TURN_STRONG = 80

ACTION_TIME = 0.05   # Sekunden


# =================================================
# Q-LEARNING EINSTELLUNGEN
# =================================================

N_ERROR_STATES = 7
N_TRENDS = 3

N_STATES = N_ERROR_STATES * N_TRENDS   # 7 * 3 = 21

TURN_VALUES = [
    80, 60, 40, 20, 0,
    -20, -40, -60, -80
]

SPEED_VALUES = [
    100,
    150
]

ACTIONS = []

for speed in SPEED_VALUES:
    for turn in TURN_VALUES:
        ACTIONS.append((speed, turn))

N_ACTIONS = len(ACTIONS)

DERIVATIVE_THRESHOLD = 2

EPSILON = 0.05
ALPHA = 0.1
GAMMA = 0.9

NUM_EPISODES = 1000
MAX_STEPS = 10000

LINE_LOST_REWARD = -100
SPEED_REWARD_WEIGHT = 2.0
MAX_SPEED = 150


# =================================================
# BAHN
# =================================================

TRACK_AMPLITUDE = 10
TRACK_WAVELENGTH = 100


def line_y(x):

    y = TRACK_AMPLITUDE * math.sin(
        2 * math.pi * x / TRACK_WAVELENGTH
    )

    return y


def line_y_derivate(x):

    slope = (
        TRACK_AMPLITUDE
        * (2 * math.pi / TRACK_WAVELENGTH)
        * math.cos(
            2 * math.pi * x / TRACK_WAVELENGTH
        )
    )

    return slope


# =================================================
# 2. ROBOTERGEOMETRIE
# =================================================

WHEEL_RADIUS = 2.8
WHEEL_DISTANCE = 18.0

SENSOR_DISTANCE = 6.5

LINE_LOST_DISTANCE = 3.0


# =================================================
# 3. STARTPOSITION
# =================================================

sensor_x = TRACK_WAVELENGTH / 4
sensor_y = line_y(sensor_x)

angle = 0.0

x = sensor_x - SENSOR_DISTANCE
y = sensor_y


path_x = [x]
path_y = [y]

track_x = []
track_y = []

sensor_path_x = []
sensor_path_y = []

reflection_history = []


# =================================================
# 4. Q-LEARNING: FEHLERZUSTAND
# =================================================

def get_error_state(error):

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
# 5. Q-LEARNING: FEHLERTREND
# =================================================

def get_trend(error, previous_error):

    delta_error = error - previous_error

    if delta_error < -DERIVATIVE_THRESHOLD:
        return 0

    elif delta_error > DERIVATIVE_THRESHOLD:
        return 2

    else:
        return 1


# =================================================
# 6. GESAMTZUSTAND
# =================================================

def get_state(error, previous_error):

    error_state = get_error_state(error)

    trend = get_trend(
        error,
        previous_error
    )

    state = (
        error_state * N_TRENDS
        + trend
    )

    return state


# =================================================
# 7. REWARD
# =================================================


def get_reward(error, speed):

    line_reward = 10 - abs(error)

    speed_reward = (
        SPEED_REWARD_WEIGHT
        * speed / MAX_SPEED
    )

    reward = line_reward + speed_reward

    return reward


# =================================================
# 8. AKTION AUSFÜHREN
# =================================================

def execute_action(action):

    base_speed, turn_speed = ACTIONS[action]

    left_motor_speed = base_speed - turn_speed
    right_motor_speed = base_speed + turn_speed

    move_robot(
        left_motor_speed,
        right_motor_speed
    )

    return base_speed #aktueller speed aus der tabelle 


# =================================================
# 9. Q-TABELLE
# =================================================

# Spalten:
# 0 = stark links
# 1 = leicht links
# 2 = gerade
# 3 = leicht rechts
# 4 = stark rechts

Q = []

for state in range(N_STATES):

    row = []

    for action in range(N_ACTIONS):

        row.append(0.0)

    Q.append(row)


# =================================================
# 10. AKTION AUSWÄHLEN
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

        max_q = max(Q[state])

        best_actions = []

        for action in range(N_ACTIONS):

            if Q[state][action] == max_q:

                best_actions.append(action)

        action = random.choice(
            best_actions
        )

    return action


# =================================================
# 11. SENSORPOSITION
# =================================================

def sensor_position():

    sensor_x = (
        x
        + SENSOR_DISTANCE
        * math.cos(angle)
    )

    sensor_y = (
        y
        + SENSOR_DISTANCE
        * math.sin(angle)
    )

    return sensor_x, sensor_y


# =================================================
# 12. LINIE VERLOREN
# =================================================

def line_lost():

    distance = sensor_distance_to_line()

    if abs(distance) > LINE_LOST_DISTANCE:

        return True

    else:

        return False


# =================================================
# 13. ROBOTER ZURÜCKSETZEN
# =================================================

def reset_robot():

    global x, y, angle

    global path_x, path_y
    global sensor_path_x, sensor_path_y
    global reflection_history


    sensor_x = TRACK_WAVELENGTH / 4
    sensor_y = line_y(sensor_x)


    # zufälliger seitlicher Startversatz
    sensor_y += (
        random.randint(-10, 10)
        / 10
    )


    # zufälliger Startwinkel
    angle = math.radians(
        random.randint(-10, 10)
    )


    # Roboter hinter den Sensor setzen
    x = (
        sensor_x
        - SENSOR_DISTANCE
        * math.cos(angle)
    )

    y = (
        sensor_y
        - SENSOR_DISTANCE
        * math.sin(angle)
    )


    path_x = [x]
    path_y = [y]

    sensor_path_x = []
    sensor_path_y = []

    reflection_history = []


# =================================================
# 14. SENSORABSTAND ZUR LINIE
# =================================================

def sensor_distance_to_line():

    sensor_x, sensor_y = sensor_position()

    line_position = line_y(sensor_x)

    derivate = line_y_derivate(sensor_x)

    delta_y = (
        sensor_y
        - line_position
    )

    distance = (
        delta_y
        / math.sqrt(
            1 + derivate**2
        )
    )

    return distance


# =================================================
# 15. SENSORWERT
# =================================================

def sensor_reflection():

    distance = sensor_distance_to_line()

    half_transition = (
        SENSOR_TRANSITION / 2
    )


    if distance <= -half_transition:

        reflection = BLACK


    elif distance >= half_transition:

        reflection = WHITE


    else:

        factor = (
            distance
            + half_transition
        ) / SENSOR_TRANSITION


        reflection = (
            BLACK
            + factor
            * (WHITE - BLACK)
        )


    return reflection


# =================================================
# 16. MOTORDREHZAHL -> GESCHWINDIGKEIT
# =================================================

def motor_to_velocity(motor_speed):

    angular_velocity = math.radians(
        motor_speed
    )

    velocity = (
        angular_velocity
        * WHEEL_RADIUS
    )

    return velocity


# =================================================
# 17. ROBOTERBEWEGUNG
# =================================================

def move_robot(
    left_motor_speed,
    right_motor_speed
):

    global x, y, angle


    left_velocity = motor_to_velocity(
        left_motor_speed
    )

    right_velocity = motor_to_velocity(
        right_motor_speed
    )


    velocity = (
        left_velocity
        + right_velocity
    ) / 2


    angular_velocity = (
        right_velocity
        - left_velocity
    ) / WHEEL_DISTANCE


    angle = (
        angle
        + angular_velocity
        * ACTION_TIME
    )


    x = (
        x
        + velocity
        * math.cos(angle)
        * ACTION_TIME
    )

    y = (
        y
        + velocity
        * math.sin(angle)
        * ACTION_TIME
    )


    sensor_x, sensor_y = sensor_position()

    sensor_path_x.append(sensor_x)
    sensor_path_y.append(sensor_y)


    path_x.append(x)
    path_y.append(y)


    reflection = sensor_reflection()

    reflection_history.append(
        reflection
    )


# =================================================
# 18. Q-LEARNING TRAINING
# =================================================

episode_lengths = []


for episode in range(NUM_EPISODES):

    reset_robot()


    # -----------------------------------------
    # Anfangszustand
    # -----------------------------------------

    reflection = sensor_reflection()

    error = TARGET - reflection


    # Beim Start kennen wir noch keinen
    # vorherigen Fehler
    previous_error = error


    state = get_state(
        error,
        previous_error
    )


    # -----------------------------------------
    # Episode
    # -----------------------------------------

    for step in range(MAX_STEPS):


        # =====================================
        # 1. Aktion auswählen
        # =====================================

        action = choose_action(state)


        # =====================================
        # 2. Aktion ausführen
        # =====================================

        current_speed = execute_action(action) #Methode wird ausgeführt und aktueller speed als current_speed zurückgegeben


        # =====================================
        # 3. neuen Fehler bestimmen
        # =====================================

        next_reflection = (
            sensor_reflection()
        )

        next_error = (
            TARGET
            - next_reflection
        )


        # =====================================
        # 4. neuen Zustand bestimmen
        # =====================================

        next_state = get_state(
            next_error,
            error
        )


        # =====================================
        # 5. prüfen ob Linie verloren
        # =====================================

        if line_lost():

            reward = (
                LINE_LOST_REWARD
            )

            # terminaler Zustand
            target_q = reward

            done = True


        else:

            reward = get_reward(
                next_error, current_speed
            )


            max_next_q = max(
                Q[next_state]
            )


            target_q = (
                reward
                + GAMMA
                * max_next_q
            )


            done = False


        # =====================================
        # 6. Q-Learning Update
        # =====================================

        old_q = (
            Q[state][action]
        )


        Q[state][action] = (
            old_q
            + ALPHA
            * (
                target_q
                - old_q
            )
        )


        # =====================================
        # 7. Episode beenden
        # =====================================

        if done:

            break


        # =====================================
        # 8. Zustand weiterschieben
        # =====================================

        previous_error = error

        error = next_error

        state = next_state


    # -----------------------------------------
    # Episodenlänge speichern
    # -----------------------------------------

    episode_lengths.append(
        step + 1
    )


    # -----------------------------------------
    # Fortschritt
    # -----------------------------------------

    if (episode + 1) % 100 == 0:

        average_length = (
            sum(
                episode_lengths[-100:]
            )
            /
            len(
                episode_lengths[-100:]
            )
        )

        print(
            "Episode:",
            episode + 1,
            "Mittlere Länge:",
            average_length
        )


# =================================================
# 19. AUSGABE
# =================================================

print("")
print("Training beendet")

print("Position:")
print("x =", x, "cm")
print("y =", y, "cm")

print(
    "angle =",
    math.degrees(angle),
    "Grad"
)


print("")
print("Q-Tabelle nach Training:")


for row in Q:

    print(row)


# =================================================
# 20. Q-TABELLE ALS COPY-PASTE AUSGEBEN
# =================================================

print("")
print("===================================")
print("Q-TABELLE ZUM COPY-PASTEN")
print("===================================")
print("")

print("Q = [")

for i in range(len(Q)):

    print(
        "    "
        + str(Q[i])
        + (
            ","
            if i < len(Q) - 1
            else ""
        )
    )

print("]")


# =================================================
# 21. BAHN PLOTTEN
# =================================================

for i in range(3000):

    current_x = i * 0.1

    track_x.append(
        current_x
    )

    track_y.append(
        line_y(current_x)
    )


plt.plot(
    track_x,
    track_y,
    label="Schwarz-Weiß-Kante"
)

plt.plot(
    path_x,
    path_y,
    label="Roboterbahn"
)

plt.plot(
    sensor_path_x,
    sensor_path_y,
    label="Sensorbahn"
)

plt.xlabel("x [cm]")
plt.ylabel("y [cm]")

plt.axis("equal")

plt.legend()

plt.show()