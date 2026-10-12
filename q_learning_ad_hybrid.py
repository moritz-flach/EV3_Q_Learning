#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
import random

import os

#try:
#    os.remove("/home/robot/q_table.txt")
#    print("Gespeicherte Q-Tabelle gelöscht")
#except OSError:
#    print("Keine gespeicherte Q-Tabelle vorhanden")


# =================================================
# 1. HARDWARE
# =================================================

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

sensor = ColorSensor(Port.S1)


# =================================================
# 2. GRUNDEINSTELLUNGEN
# =================================================

# Sollwert des Farbsensors
LEARN = False

TARGET = 35

# Anzahl Zustände und Aktionen
N_ERROR_STATES = 7
N_TRENDS = 3

N_STATES = N_ERROR_STATES * N_TRENDS   # 7 * 3 = 21

TURN_VALUES = [
    80,
    60,
    40,
    20,
    0,
    -20,
    -40,
    -60,
    -80
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

#Q-Tabelle in Datei speichern
Q_FILE = "/home/robot/q_table.txt"

# Q-Learning-Parameter
EPSILON = 0     # 20 % Exploration
ALPHA = 0.1        # Lernrate
GAMMA = 0.9        # Gewichtung zukünftiger Rewards

# Fahrparameter
BASE_SPEED = 100
TURN_LIGHT = 40
TURN_STRONG = 80

# Dauer einer Aktion in Millisekunden
ACTION_TIME = 50

LINE_LOST_ERROR = 25
LINE_LOST_COUNT = 10
LINE_LOST_REWARD = -100
SPEED_REWARD_WEIGHT = 2.0
MAX_SPEED = 150

# =================================================
# 3. Q-TABELLE INITIALISIEREN
# =================================================
# Aktionen:
# [stark links, leicht links, gerade, leicht rechts, stark rechts]

Q = [
    [5.598693202981564, 5.9259539511522075, 5.918912213462661, -0.9231161835984616, 5.048257046013971, 2.150656489357876, 5.3017036900314505, -4.588996318833626, 62.24376659190674, 8.698819282368916, 1.5258162268832192, 16.57281986145363, 2.528937815920761, -2.709560662426453, 7.223004060295496, 4.528875359139844, -0.16226207361033484, 12.858346634539924],
    [-41.62481819489094, -29.956081392222277, -5.272518416330433, -29.31078747367988, -20.72840864745585, -44.406804511082825, -33.53381980270926, -21.448899773364808, -33.06218677913641, -35.61776799118899, -35.445586779262605, -28.62001285835842, -12.914066920410875, -35.12440912901313, -26.303406485931816, -17.410515075266435, -23.97292868791544, 38.83602583348871],
    [6.563584138464539, 5.750305291459473, -1.5393011106786267, 0.0, -2.3596220841602085, 5.246107769989317, -0.4737561919263664, 32.848264758847975, 13.366683475253769, 6.583860220744745, 7.150211625677246, 1.9310558899047927, 6.718026259933922, -1.5, 4.25060726338641, 3.4756630405855815, 44.77206877355985, 13.29997705568151],
    [45.21453853663137, 50.446927949556716, 57.70033765456601, 68.63654007429255, 67.95097282819951, 75.25028127069488, 78.19553570449777, 82.50951145191704, 85.75822881665599, 44.45021587492653, 50.068024024809446, 59.06013278085679, 62.779334499950494, 67.69168353556091, 73.35383070219295, 75.58196290490699, 80.66592771310961, 83.70218926500525],
    [17.18696034530652, 26.28106623628539, 6.694509038938223, 24.799529459068953, 23.50780864467607, 28.446783176418045, 30.610817061765847, 84.77298488327943, 48.13386731184741, 11.69987158302677, 6.689564809812699, 29.46302450752564, 23.675227753375395, 17.61158642861742, 32.463036992375194, 27.466269120577213, 30.513674741436553, 48.66653211703943],
    [40.39808763185045, 49.82989535152791, 64.06122593153748, 65.75803024649022, 71.08269473325275, 70.97700045656485, 73.05233990060904, 80.43265152090754, 85.63475900162096, 39.397448472979995, 44.14962316303059, 57.66762277398128, 60.219014116904255, 69.97012846803663, 69.11172461883534, 71.98699863912529, 77.98442257560386, 83.67111706193603],
    [76.43634646667105, 75.99573622085327, 81.0919909426075, 84.27320968749483, 88.26669863008777, 89.4439800246919, 90.90930348317484, 90.93753260790896, 91.46077804170584, 74.58954629374672, 75.99444009780099, 78.8222224527888, 83.12555118666971, 85.86116880590163, 90.59451622480718, 90.93492954341217, 90.9533549241446, 92.19080779840867],
    [76.65444856676748, 80.03591498478737, 85.30065387750007, 87.6451747650226, 89.22547135235432, 90.24230872488612, 91.04910214876388, 92.68332183459208, 90.68678573249117, 76.95165637671664, 78.2404670284475, 84.29500015451652, 87.8035327977496, 90.2930916606089, 90.31301411907562, 89.80313996225028, 90.820771410871, 90.54204792079165],
    [73.83101659231224, 75.34554028541316, 79.85685273640969, 81.93299052483313, 87.14013778107812, 88.13083565719221, 90.43595229425749, 91.14247359115244, 91.644359003635, 70.03493862859294, 75.3879391086385, 78.43367131139405, 79.83870203627839, 83.62818675441727, 88.76720592305358, 90.55903917179488, 91.06496223889161, 91.65803866287655],
    [87.34401494721936, 87.81522498716711, 90.00694319675091, 89.71474188112533, 91.62457393787211, 89.64066605608444, 89.34178411624276, 86.91018361671989, 83.29281374266657, 88.38988809862958, 88.59264903792119, 88.56208479322343, 89.72454109769056, 89.70379703474025, 89.78920086947339, 88.12058259214686, 87.99384483739101, 84.59508150502178],
    [82.97931170430125, 88.73340899901308, 89.56645572015147, 89.65963122361205, 90.48713939156843, 89.73249432911761, 89.4868012979695, 87.87300661624225, 85.7808701594915, 86.28097251084884, 89.43943968886981, 90.01519629603473, 90.20641208721305, 92.16849505374547, 89.56443731693551, 90.23593470318248, 89.30831746355949, 84.64329015225736],
    [83.78774983838952, 88.60002410423421, 89.55993470104227, 89.62068959319222, 90.372643658088, 89.51588291362776, 89.6816826918256, 86.41592158974731, 86.56953344481013, 80.15512311855451, 86.42289148770186, 88.65987167946182, 89.63029758903429, 89.66832285250945, 89.52588396682971, 89.02169825254514, 88.56185377234472, 86.75812553930628],
    [91.03252301472092, 90.84965917001577, 90.54628344171495, 89.36593417185424, 87.6228308326635, 83.10134948274522, 78.76455937649243, 76.58892951070084, 73.96391026398629, 92.6892411899898, 90.23068863328615, 90.264985933874, 87.5227985465793, 83.32994747809252, 84.06217963106572, 76.49068113600171, 75.0833709177772, 69.96971293547091],
    [90.80683065281485, 92.68120480986107, 91.00128973016182, 90.51758437783732, 89.51807410529348, 87.0012446683367, 83.2731950864213, 79.7159959601749, 75.91314893500954, 91.17738542012454, 90.69543810189404, 90.98186928851364, 90.85498658929829, 90.05671725493431, 87.24544618814512, 82.86501663035955, 76.75959679313371, 75.85088519240445],
    [92.72895889857813, 91.11255717625582, 90.71982093747788, 89.9909707338906, 88.67479099053926, 85.16518761869027, 79.56035379414209, 76.10452555607223, 73.34982587168649, 90.7331873026536, 90.8150623304872, 90.74735880586546, 89.2063069770457, 87.61870947687817, 82.80741440735363, 77.47435500198577, 74.9236816712635, 74.37604604932928],
    [86.28110631325823, 82.9841524587854, 73.67522475135864, 68.24209490554038, 71.3945884830424, 68.3240498807995, 64.85262788396817, 54.58118173156761, 48.45549089930111, 81.68239913758984, 71.93319784460434, 69.3734775155387, 69.10974229027703, 69.98351558030666, 63.48028735682768, 56.33335051414011, 52.01257350804351, 44.48129441914648],
    [85.94335811109889, 0.0, 13.179777356862587, 5.240580634698059, 11.530569234590867, 33.06398553993739, 39.907251943271554, 9.094524179616217, 12.498902063250801, 35.07785303158734, 36.7542595448308, 29.085479228651455, 16.199127976391868, 43.655047318644925, 15.79271005270081, 9.903845026313324, 13.43722606068529, 10.170100741186388],
    [87.08203822543697, 82.11890120142455, 78.46488702128818, 74.78349549130606, 73.05777264552715, 69.14767115377757, 66.45227821963907, 55.81415904929063, 50.941392476138645, 82.7204648688407, 81.69530989566975, 78.15937871932408, 72.44887186376533, 72.70834561945657, 60.62344904576883, 58.71139171167391, 55.85525216729171, 49.46010269574597],
    [72.17333157869368, 19.722079800569308, 0.0, 8.079558346953826, 7.947355622146757, 8.09428721119619, 4.386522887415896, 4.032723494082844, 0.0, 7.220911847855186, -0.001373332187653134, 0.0, 2.027672023793664, 4.286325781360714, 3.8690649503963304, 3.7231695324001493, 8.65543043308283, 2.9324502719865633],
    [-30.770233526767214, 36.78671462982874, -25.12843906229942, -31.422717720126464, -22.296723851454203, -26.319796806690498, -22.56364707097046, -9.618349338254067, -32.29517308036242, -18.70296225912161, -21.240162101299568, -28.86487704157111, -12.91978470606319, -16.802179667966488, -34.427209942533345, -20.195646520482295, -24.777596509415325, -22.955810538208045],
    [66.32599520338779, 10.040594616306585, -2.123826162362028, 9.407881365753978, 15.45667704548146, 15.148529119600015, 1.7686101753366632, 5.383681309363681, 7.236278902482605, 6.973533017355175, -3.0356575010717224, 6.616185293032463, 7.550887808475597, 19.504625535446724, 6.669042121327591, 3.502525417138332, 1.3921241187581526, 5.026212909681288]
]

# Zeilen = Zustände
# Spalten = Aktionen
# Zellen = Q-Werte, also Bewertungen der jeweiligen
#          Aktion im jeweiligen Zustand

#Q = [ keine dynmaischen states
#    [0.0, 0.0, 0.0, 1.0, 2.0],  # Zustand 0
#    [0.0, 0.0, 0.0, 2.0, 1.0],  # Zustand 1
#    [0.0, 0.0, 1.0, 2.0, 0.0],  # Zustand 2
#    [0.0, 1.0, 2.0, 1.0, 0.0],  # Zustand 3
#    [0.0, 2.0, 1.0, 0.0, 0.0],  # Zustand 4
#    [1.0, 2.0, 0.0, 0.0, 0.0],  # Zustand 5
#    [2.0, 1.0, 0.0, 0.0, 0.0]   # Zustand 6
#]


# Alternative:
# komplette Q-Tabelle mit 0 initialisieren
#
# Q = []
#
# for state in range(N_STATES):
#     row = []
#
#     for action in range(N_ACTIONS):
#         row.append(0.0)
#
#     Q.append(row)

# =================================================
# Q-TABELLE SPEICHERN
# =================================================

def save_q_table():

    with open(Q_FILE, "w") as file:

        for row in Q:

            line = ""

            for value in row:
                line += str(value) + " "

            file.write(line + "\n")
        
    
# =================================================
# Q-TABELLE LADEN
# =================================================

def load_q_table():

    global Q

    try:
        with open(Q_FILE, "r") as file:

            loaded_q = []

            for line in file:

                values = line.split()
                row = []

                for value in values:
                    row.append(float(value))

                loaded_q.append(row)

        # Prüfen, ob die gespeicherte Tabelle
        # wirklich 21 Zustände x 5 Aktionen besitzt
        if len(loaded_q) == N_STATES:

            valid = True

            for row in loaded_q:
                if len(row) != N_ACTIONS:
                    valid = False

            if valid:
                Q = loaded_q
                print("Q-Tabelle geladen")

            else:
                print("Q-Tabelle hat falsche Groesse")

        else:
            print("Q-Tabelle hat falsche Groesse")

    except OSError:

        # Beim ersten Start existiert die Datei noch nicht.
        # Dann wird einfach die oben definierte Start-Q-Tabelle benutzt.
        print("Keine gespeicherte Q-Tabelle gefunden")



# =================================================
# 4. MOTORANSTEUERUNG
# =================================================

def drive(base_speed, turn_speed):

    left_speed = base_speed - turn_speed
    right_speed = base_speed + turn_speed

    left_motor.run(left_speed)
    right_motor.run(right_speed)



def execute_action(action):

    base_speed, turn_speed = ACTIONS[action]

    drive(base_speed, turn_speed)

    return base_speed


# =================================================
# 5. ZUSTAND BESTIMMEN
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


def get_trend(error, previous_error):

    delta_error = error - previous_error

    if delta_error < -DERIVATIVE_THRESHOLD:
        return 0

    elif delta_error > DERIVATIVE_THRESHOLD:
        return 2

    else:
        return 1


def get_state(error, previous_error):

    error_state = get_error_state(error)
    trend = get_trend(error, previous_error)

    state = error_state * N_TRENDS + trend

    return state


# =================================================
# 6. AKTION AUSWÄHLEN
# =================================================

def choose_action(state):

    # Exploration:
    # mit Wahrscheinlichkeit EPSILON
    # eine zufällige Aktion ausprobieren
    if random.random() < EPSILON:

        action = random.randint(0, N_ACTIONS - 1)

    # Exploitation:
    # ansonsten die bisher beste Aktion auswählen
    else:

        action = Q[state].index(max(Q[state]))

    return action


# =================================================
# 8. REWARD BERECHNEN
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
# Line Lost Error TRUE/FALSE gibt abbruch für while schleife (definiert episoden)
# =================================================
def line_lost(error):

    if abs(error) > LINE_LOST_ERROR:
        return True
    else:
        return False


# =================================================
# 9. Q-LEARNING TRAINING
# =================================================

# -----------------------------------------
# Anfangszustand bestimmen
# -----------------------------------------

#load_q_table()
print("Q-Tabelle start:")

for row in Q:
    print(row)

print("--------------------")

state_visits = [0] * N_STATES

lost_counter = 0

sensor_value = sensor.reflection()

error = TARGET - sensor_value   # gleiche Vorzeichenkonvention wie beim PD-Regler

# Beim allerersten Messwert kennen wir noch keinen
# vorherigen Fehler. Deshalb setzen wir ihn gleich.
previous_error = error

state = get_state(error, previous_error)


# -----------------------------------------
# Trainingsschleife
# -----------------------------------------

while True:

    state_visits[state] += 1
    # -------------------------------------
    # 1. Aktion a auswählen
    # -------------------------------------

    action = choose_action(state)


    # -------------------------------------
    # 2. Aktion ausführen
    # -------------------------------------

    current_speed = execute_action(action)

    # Aktion kurz wirken lassen
    wait(ACTION_TIME)


    # -------------------------------------
    # 3. Neuen Zustand s' bestimmen
    # -------------------------------------

    next_sensor_value = sensor.reflection()

    next_error = TARGET - next_sensor_value

    next_state = get_state(next_error, error)

    # -------------------------------------
    # Abbruchbedingung: Linie verloren
    # -------------------------------------  
    if line_lost(next_error):
        lost_counter += 1
    else:
        lost_counter = 0

    if lost_counter >= LINE_LOST_COUNT:
        linelost = True
    else:
        linelost = False

    if linelost:
        reward = LINE_LOST_REWARD
    else:
        reward = get_reward(next_error, current_speed)


    # -------------------------------------
    # 5. Q-Learning-Update
    # -------------------------------------

    if LEARN:

        # bisheriger Q-Wert Q(s,a)
        old_q = Q[state][action]

        if linelost:        #episodenende, kein zukünftiger Reward
            target_q = reward

        else:
            # maximaler Q-Wert im neuen Zustand s'
            max_next_q = max(Q[next_state])

            # Zielwert für Q(s,a)
            target_q = reward + GAMMA * max_next_q

        # TD-Fehler
        #Neue schätzung mit der alten schätzung
        q_error = target_q - old_q

        #Q-Wert aktualisieren
        Q[state][action] = old_q + ALPHA * q_error


        #Episodenende, wenn Linie verloren
        if linelost:
            left_motor.stop()
            right_motor.stop()

            save_q_table()

            print("Linie verloren")
            print("Q-Tabelle gespeichert")

            print("State Visits:")

            for i in range(N_STATES):
                print("State", i, ":", state_visits[i])

            break


    #ansonsten weiterfahren und neuen Zustand s' als aktuellen Zustand s übernehmen
    error = next_error
    state = next_state


    

