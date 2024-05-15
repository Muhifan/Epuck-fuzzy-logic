import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from controller import Robot, DistanceSensor, Motor
from skfuzzy import control as ctrl

# Inisialisasi robot
robot = Robot()

# Mendefinisikan waktu siklus dan kecepatan maksimum
TIME_STEP = 64
MAX_SPEED = 6.28

# Mendefinisikan sensor jarak
ps7 = robot.getDistanceSensor("ps7")
ps7.enable(TIME_STEP)
ps0 = robot.getDistanceSensor("ps0")
ps0.enable(TIME_STEP)
ps10 = robot.getDistanceSensor("ps10")
ps10.enable(TIME_STEP)

# Mendefinisikan motor
left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

ps0 = np.arange(0, 21, 0.1)  # Perubahan rentang
ps7 = np.arange(0, 21, 0.1)    # Perubahan rentang
x_sensor_tengah = np.arange(0, 21, 0.1)  # Perubahan rentang

sensor_kanan_dekat = fuzz.trimf(ps0, [0, 5, 10])
sensor_kanan_sedang = fuzz.trimf(ps0, [5, 10, 15])
sensor_kanan_jauh = fuzz.trimf(ps0, [10, 15, 20])

sensor_kiri_dekat = fuzz.trimf(ps7, [0, 5, 10])       # Perubahan rentang
sensor_kiri_sedang = fuzz.trimf(ps7, [5, 10, 15])     # Perubahan rentang
sensor_kiri_jauh = fuzz.trimf(ps7, [10, 15, 20])      # Perubahan rentang

sensor_tengah_dekat = fuzz.trimf(x_sensor_tengah, [0, 5, 10])
sensor_tengah_sedang = fuzz.trimf(x_sensor_tengah, [5, 10, 15])
sensor_tengah_jauh = fuzz.trimf(x_sensor_tengah, [10, 15, 20])

#Pendefinisian Membership Function
sensor_kanan_dkt = fuzz.interp_membership(ps0, sensor_kanan_dekat)
sensor_kanan_sdg = fuzz.interp_membership(ps0, sensor_kanan_sedang)
sensor_kanan_jh = fuzz.interp_membership(ps0, sensor_kanan_jauh)

sensor_kiri_dkt = fuzz.interp_membership(ps7, sensor_kiri_dekat)
sensor_kiri_sdg = fuzz.interp_membership(ps7, sensor_kiri_sedang)
sensor_kiri_jh = fuzz.interp_membership(ps7, sensor_kiri_jauh)

sensor_tengah_dkt = fuzz.interp_membership(x_sensor_tengah, sensor_tengah_dekat)
sensor_tengah_sdg = fuzz.interp_membership(x_sensor_tengah, sensor_tengah_sedang)
sensor_tengah_jh = fuzz.interp_membership(x_sensor_tengah, sensor_tengah_jauh)


#RULE BASE
rules_1 = np.fmin(sensor_kanan_dkt, sensor_kanan_dekat)
rules_2 = np.fmin(sensor_kanan_dkt,sensor_kanan_sedang)
rules_3 = np.fmin(sensor_kanan_dkt,sensor_kanan_jauh)
rules_4 = np.fmin(sensor_kanan_sdg,sensor_kanan_dekat)
rules_5 = np.fmin(sensor_kanan_sdg,sensor_kanan_sedang)
rules_6 = np.fmin(sensor_kanan_sdg,sensor_kanan_jauh)
rules_7 = np.fmin(sensor_kanan_jh,sensor_kanan_dekat)
rules_8 = np.fmin(sensor_kanan_jh,sensor_kanan_sedang)
rules_9 = np.fmin(sensor_kanan_jh,sensor_kanan_jauh)
rules_10 = np.fmin(sensor_kiri_dkt,sensor_kiri_dekat)
rules_11 = np.fmin(sensor_kiri_dkt,sensor_kiri_sedang)
rules_12 = np.fmin(sensor_kiri_dkt,sensor_kiri_jauh)
rules_13 = np.fmin(sensor_kiri_sdg,sensor_kiri_dekat)
rules_14 = np.fmin(sensor_kiri_sdg,sensor_kiri_sedang)
rules_15 = np.fmin(sensor_kiri_sdg,sensor_kiri_jauh)
rules_16 = np.fmin(sensor_kiri_jh,sensor_kiri_dekat)
rules_17 = np.fmin(sensor_kiri_jh,sensor_kiri_sedang)
rules_18 = np.fmin(sensor_kiri_jh,sensor_kiri_jauh)
rules_19 = np.fmin(sensor_tengah_dkt,sensor_tengah_dekat)
rules_20 = np.fmin(sensor_tengah_dkt,sensor_tengah_sedang)
rules_21 = np.fmin(sensor_tengah_dkt,sensor_tengah_jauh)
rules_22 = np.fmin(sensor_tengah_sdg,sensor_tengah_dekat)
rules_23 = np.fmin(sensor_tengah_sdg,sensor_tengah_sedang)
rules_24 = np.fmin(sensor_tengah_sdg,sensor_tengah_jauh)
rules_25 = np.fmin(sensor_tengah_jh,sensor_tengah_dekat)
rules_26 = np.fmin(sensor_tengah_jh,sensor_tengah_sedang)
rules_27 = np.fmin(sensor_tengah_jh,sensor_tengah_jauh)

# Define activation rules based on the defined rules
activation_sensor_kanan = np.fmax(rules_1, np.fmax(rules_2, np.fmax(rules_3, np.fmax(rules_4, np.fmax(rules_5, np.fmax(rules_7, np.fmax(rules_9, np.fmax(rules_11, np.fmax(rules_13, np.fmax(rules_15, np.fmax(rules_17, np.fmax(rules_19, np.fmax(rules_21, np.fmax(rules_23, rules_25)))))))))))))))
activation_sensor_kiri = np.fmax(rules_6, np.fmax(rules_8, np.fmax(rules_10, np.fmax(rules_12, np.fmax(rules_14, np.fmax(rules_16, np.fmax(rules_18, np.fmax(rules_20, np.fmax(rules_22, rules_24))))))))))
activation_sensor_tengah = np.fmax(rules_26, np.fmax(rules_27))

# Visualize activation rules
plt.figure(figsize=(8, 6))

plt.plot(ps0, activation_sensor_kanan, 'b', linewidth=1.5, label='sensor_kanan')
plt.plot(ps0, activation_sensor_kiri, 'g', linewidth=1.5, label='sensor_kiri')
plt.plot(ps0, activation_sensor_tengah, 'r', linewidth=1.5, label='sensor_tengah')

plt.title('Activation Rules')
plt.legend()
plt.xlabel('Input (Sensor Value)')
plt.ylabel('Membership')
plt.ylim(0, 1.05)
plt.grid(True)
plt.tight_layout()
plt.show()

# Visualize these universes and membership functions
fig, (ax0, ax1, ax2) = plt.subplots(nrows=3, figsize=(8, 10))

ax0.plot(ps0, sensor_kanan_dekat, 'b', linewidth=1.5, label='Dekat')
ax0.plot(ps0, sensor_kanan_sedang, 'y', linewidth=1.5, label='Sedang')
ax0.plot(ps0, sensor_kanan_jauh, 'g', linewidth=1.5, label='Jauh')
ax0.set_title('Sensor Kanan')
ax0.legend()

ax1.plot(ps7, sensor_kiri_dekat, 'b', linewidth=1.5, label='Dekat')
ax1.plot(ps7, sensor_kiri_sedang, 'y', linewidth=1.5, label='Sedang')
ax1.plot(ps7, sensor_kiri_jauh, 'g', linewidth=1.5, label='Jauh')
ax1.set_title('Sensor Kiri')
ax1.legend()

ax2.plot(x_sensor_tengah, sensor_tengah_dekat, 'b', linewidth=1.5, label='Dekat')
ax2.plot(x_sensor_tengah, sensor_tengah_sedang, 'y', linewidth=1.5, label='Sedang')
ax2.plot(x_sensor_tengah, sensor_tengah_jauh, 'g', linewidth=1.5, label='Jauh')
ax2.set_title('Sensor Tengah')
ax2.legend()

# Turn off top/right axes
for ax in (ax0, ax1, ax2):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()
plt.tight_layout()
plt.show()