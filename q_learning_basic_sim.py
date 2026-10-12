import math
import matplotlib.pyplot as plt
import random


# =================================================
# 1. GRUNDEINSTELLUNGEN
# =================================================

LEARN = False

BLACK = 8
WHITE = 62
TARGET = 35
SENSOR_TRANSITION = 1.0   # cm

BASE_SPEED = 100
TURN_LIGHT = 40
TURN_STRONG = 80

ACTION_TIME = 0.05   # Sekunden

N_STATES = 7
N_ACTIONS = 5

EPSILON = 0.0
ALPHA = 0.1
GAMMA = 0.9

NUM_EPISODES = 1
MAX_STEPS = 100

LINE_LOST_REWARD = -100
#-----------------------------------------------------------
#Bahn Bestimmen
def line_y(x): #einfach f(x) = A * sin(2 * pi * x / wavelength) für die Linie

    # Sinusförmige Schwarz-Weiß-Kante
    y = TRACK_AMPLITUDE * math.sin(
        2 * math.pi * x / TRACK_WAVELENGTH
    )

    return y

TRACK_AMPLITUDE = 10      # maximale seitliche Auslenkung [cm]
TRACK_WAVELENGTH = 100   # Länge einer kompletten Welle [cm]
#--------------------------------------------------------------



# =================================================
# 2. ROBOTERGEOMETRIE
# =================================================

WHEEL_RADIUS = 2.8      # cm
WHEEL_DISTANCE = 18.0   # cm

SENSOR_DISTANCE = 6.5   # Abstand Sensor vor der Radachse [cm]
LINE_LOST_DISTANCE = 3.0   # cm


# =================================================
# 3. STARTPOSITION
# =================================================

# Sensor startet direkt auf der Bahn
sensor_x = TRACK_WAVELENGTH / 4
sensor_y = line_y(sensor_x)

# Bahn ist dort waagerecht
angle = 0.0

# Roboter steht SENSOR_DISTANCE hinter dem Sensor
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
# Q-Learning-Teil
# =================================================

def get_state(error): #error in states umwandeln

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

def get_reward(error):

    # kleine Abweichung -> hoher Reward
    # große Abweichung -> niedriger Reward
    reward = 10 - abs(error)

    return reward


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
        raise ValueError("Ungültige Aktion")

    left_motor_speed = BASE_SPEED - turn_speed
    right_motor_speed = BASE_SPEED + turn_speed

    move_robot(left_motor_speed, right_motor_speed)


# =================================================
# Q-TABELLE
# =================================================

# Spalten:
# [stark links, leicht links, gerade, leicht rechts, stark rechts]
#Tabelle mit nur nullen initialisieren
Q = []

for state in range(N_STATES):

    row = []

    for action in range(N_ACTIONS):
        row.append(0.0)

    Q.append(row)

#Q = [
#    [0.0, 0.0, 0.0, 1.0, 3.0],  # State 0
#    [0.0, 0.0, 0.0, 3.0, 2.0],  # State 1
#    [0.0, 0.0, 2.0, 3.0, 1.0],  # State 2
#    [0.0, 1.0, 3.0, 1.0, 0.0],  # State 3
#    [1.0, 3.0, 2.0, 0.0, 0.0],  # State 4
#    [2.0, 3.0, 0.0, 0.0, 0.0],  # State 5
#    [3.0, 1.0, 0.0, 0.0, 0.0]   # State 6
#]

def choose_action(state): #wählt beste aktion im State aus, wenn alle gleich sind wählt zufällig eine der besten aktionen

    # Exploration
    if random.random() < EPSILON:
        action = random.randint(0, N_ACTIONS - 1)

    # Exploitation
    else: #wenn alle werte gleich sind nehme zufällig eine der besten aktionen
        max_q = max(Q[state])

        best_actions = []

        for action in range(N_ACTIONS):
            if Q[state][action] == max_q:
                best_actions.append(action)

        action = random.choice(best_actions)

    return action


# =================================================
# Simulations-Teil
# =================================================
def sensor_position(): #Sensorposition einfach weiter vorne als räder
    """
    Berechnet die aktuelle Position des Farbsensors.
    """

    sensor_x = x + SENSOR_DISTANCE * math.cos(angle)
    sensor_y = y + SENSOR_DISTANCE * math.sin(angle)

    return sensor_x, sensor_y 

def line_lost():

    distance = sensor_distance_to_line()

    if abs(distance) > LINE_LOST_DISTANCE:
        return True

    else:
        return False


def reset_robot():

    global x, y, angle
    global path_x, path_y
    global sensor_path_x, sensor_path_y
    global reflection_history

    sensor_x = TRACK_WAVELENGTH / 4
    sensor_y = line_y(sensor_x)

    # zufälliger Startversatz von -1.0 bis +1.0 cm
    sensor_y += random.randint(-10, 10) / 10

    # zufälliger Startwinkel von -10 bis +10 Grad
    angle = math.radians(random.randint(-10, 10))

    # Roboter steht hinter dem Sensor
    x = sensor_x - SENSOR_DISTANCE * math.cos(angle)
    y = sensor_y - SENSOR_DISTANCE * math.sin(angle)

    path_x = [x]
    path_y = [y]

    sensor_path_x = []
    sensor_path_y = []

    reflection_history = []


def line_y_derivate(x):#Ableitung der Bahn an stelle x 

    slope = (
        TRACK_AMPLITUDE
        * (2 * math.pi / TRACK_WAVELENGTH)
        * math.cos(
            2 * math.pi * x / TRACK_WAVELENGTH
        )
    )

    return slope

def sensor_distance_to_line():

    sensor_x, sensor_y = sensor_position()

    line_position = line_y(sensor_x) #was ist der y wert der linie an der stelle des sensors

    derivate = line_y_derivate(sensor_x) #also wir haben Punkt P(xs,yl) und die Tangentensteigung m

    # vertikaler Abstand
    delta_y = sensor_y - line_position #ys minus yl

    # senkrechter Abstand zur lokalen Bahnrichtung
    distance = delta_y / math.sqrt(1 + derivate**2) #Formel für Abstand von Punkt zu Gerade: d = |Ax + By + C| / sqrt(A^2 + B^2) wobei A = -m, B = 1, C = -yl
                                                    #ist zwar auch approximiert, aber passt erstmal
    return distance


def sensor_reflection(): #wandelt distanz von sensor zu linie in einen reflektionswert um

    distance = sensor_distance_to_line()

    half_transition = SENSOR_TRANSITION / 2

    # Sensor vollständig auf Schwarz
    if distance <= -half_transition:
        reflection = BLACK #8

    # Sensor vollständig auf Weiß
    elif distance >= half_transition:
        reflection = WHITE #62

    # Sensor befindet sich im Übergangsbereich
    else: #erzeugt gleitkommawert im bereich zwischen schwarz und weiß abhängig von der position des sensors zur linie
        factor = (
            distance + half_transition
        ) / SENSOR_TRANSITION

        reflection = BLACK + factor * (WHITE - BLACK)

    return reflection



# =================================================
# 4. Steuerung des Roboters
# =================================================

def motor_to_velocity(motor_speed):
    """
    Wandelt Motordrehzahl [Grad/s]
    in Radgeschwindigkeit [cm/s] um.
    """

    angular_velocity = math.radians(motor_speed)

    # v = omega * r
    velocity = angular_velocity * WHEEL_RADIUS

    return velocity




# =================================================
# 5. Ein Iterationsschritt des Roboters
# =================================================

def move_robot(left_motor_speed, right_motor_speed):

    global x, y, angle

    left_velocity = motor_to_velocity(left_motor_speed)
    right_velocity = motor_to_velocity(right_motor_speed)

    # Vorwärtsgeschwindigkeit
    # v = (vL + vR) / 2
    velocity = (left_velocity + right_velocity) / 2

    # Drehgeschwindigkeit
    # omega = (vR - vL) / b
    angular_velocity = (
        right_velocity - left_velocity
    ) / WHEEL_DISTANCE

    # Winkel aktualisieren
    angle = angle + angular_velocity * ACTION_TIME

    # Position aktualisieren
    x = x + velocity * math.cos(angle) * ACTION_TIME
    y = y + velocity * math.sin(angle) * ACTION_TIME
    
    sensor_x, sensor_y = sensor_position()

    sensor_path_x.append(sensor_x)
    sensor_path_y.append(sensor_y)

    # Position für späteren Plot speichern
    path_x.append(x)
    path_y.append(y)

    reflection = sensor_reflection()
    reflection_history.append(reflection)



# =================================================
# 6. Anwenden des Q-Learning-Algorithmus
# =================================================

episode_lengths = []


for episode in range(NUM_EPISODES):

    reset_robot()

    # Anfangszustand
    reflection = sensor_reflection()
    error = TARGET - reflection
    state = get_state(error)


    for step in range(MAX_STEPS):

        # Aktion auswählen
        action = choose_action(state)

        # Aktion ausführen
        execute_action(action)


        # neuen Zustand bestimmen
        next_reflection = sensor_reflection()
        next_error = TARGET - next_reflection
        next_state = get_state(next_error)


        # prüfen, ob Linie verloren
        if line_lost():

            reward = LINE_LOST_REWARD

            # am Episodenende gibt es keinen zukünftigen Q-Wert
            target_q = reward

            done = True

        else:

            reward = get_reward(next_error)

            max_next_q = max(Q[next_state])

            target_q = (
                reward
                + GAMMA * max_next_q
            )

            done = False


        # Q-Learning-Update
        old_q = Q[state][action]

        Q[state][action] = (
            old_q
            + ALPHA * (target_q - old_q)
        )


        # Episode beenden
        if done:
            break


        # neuer Zustand wird aktueller Zustand
        state = next_state


    # Länge der Episode speichern
    episode_lengths.append(step + 1)


    # gelegentlich Fortschritt ausgeben
    if (episode + 1) % 100 == 0:

        average_length = (
            sum(episode_lengths[-100:])
            / len(episode_lengths[-100:])
        )

        print(
            "Episode:", episode + 1,
            "Mittlere Länge:", average_length
        )

# =================================================
# 7. AUSGABE
# =================================================
#Roboterposition nach 1000 Schritten ausgeben
print("Position nach 1000 Schritten:")
print("x =", x, "cm")
print("y =", y, "cm")
print("angle =", math.degrees(angle), "Grad")
#Sensorwerte ausgeben
print("Start Sensorwert:", reflection_history[0])
print("Ende Sensorwert:", reflection_history[-1])
#print("Sensorwerte Verlauf:", reflection_history)

print("Q-Tabelle nach Training:")

for row in Q:
    print(row)



# =================================================
# 8. BAHN PLOTTEN
# =================================================
#Strecke für den Plot erzeugen
for i in range(3000):
    current_x = i * 0.1

    track_x.append(current_x)
    track_y.append(line_y(current_x))

plt.plot(track_x, track_y, label="Schwarz-Weiß-Kante")

plt.plot(path_x, path_y, label="Roboterbahn")

plt.plot(sensor_path_x,sensor_path_y, label="Sensorbahn")

plt.xlabel("x [cm]")
plt.ylabel("y [cm]")

plt.axis("equal")
plt.legend()
plt.show()
