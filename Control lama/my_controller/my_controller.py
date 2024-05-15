from controller import Robot, DistanceSensor, Motor
import numpy as np
import skfuzzy as fuzz
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

# Mendefinisikan motor
left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Mendefinisikan variabel input dan output fuzzy
jarak_kanan = ctrl.Antecedent(np.arange(0, 1.5, 0.1), 'jarak_kanan')
jarak_kiri = ctrl.Antecedent(np.arange(0, 1.5, 0.1), 'jarak_kiri')
kecepatan = ctrl.Consequent(np.arange(0, MAX_SPEED + 0.1, 0.1), 'kecepatan')

# Fungsi keanggotaan untuk variabel input dan output
jarak_kanan['dekat'] = fuzz.trimf(jarak_kanan.universe, [0, 0, 0.5])
jarak_kanan['sedang'] = fuzz.trimf(jarak_kanan.universe, [0, 0.5, 1.0])
jarak_kanan['jauh'] = fuzz.trimf(jarak_kanan.universe, [0.5, 1.0, 1.5])

jarak_kiri['dekat'] = fuzz.trimf(jarak_kiri.universe, [0, 0, 0.5])
jarak_kiri['sedang'] = fuzz.trimf(jarak_kiri.universe, [0, 0.5, 1.0])
jarak_kiri['jauh'] = fuzz.trimf(jarak_kiri.universe, [0.5, 1.0, 1.5])

kecepatan['lambat'] = fuzz.trimf(kecepatan.universe, [0, 0, MAX_SPEED/2])
kecepatan['cepat'] = fuzz.trimf(kecepatan.universe, [0, MAX_SPEED/2, MAX_SPEED])

# Aturan fuzzy
rule1 = ctrl.Rule(jarak_kanan['dekat'] & jarak_kiri['dekat'], kecepatan['lambat'])
rule2 = ctrl.Rule(jarak_kanan['jauh'] & jarak_kiri['jauh'], kecepatan['cepat'])
rule3 = ctrl.Rule(jarak_kanan['sedang'] & jarak_kiri['sedang'], kecepatan['cepat'])

# Membuat sistem kontrol
kecepatan_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
kecepatan_simulasi = ctrl.ControlSystemSimulation(kecepatan_ctrl)

# Loop utama
while robot.step(TIME_STEP) != -1:
    # Membaca jarak dari sensor ultrasonik
    jarak_kanan_value = ps7.getValue()
    jarak_kiri_value = ps0.getValue()

    # Memberikan input ke dalam sistem kontrol fuzzy
    kecepatan_simulasi.input['jarak_kanan'] = jarak_kanan_value
    kecepatan_simulasi.input['jarak_kiri'] = jarak_kiri_value

    # Melakukan perhitungan
    kecepatan_simulasi.compute()

    # Mendapatkan output kecepatan
    output_kecepatan = kecepatan_simulasi.output['kecepatan']

    # Mengatur kecepatan motor
    left_motor.setVelocity(MAX_SPEED - output_kecepatan)
    right_motor.setVelocity(MAX_SPEED - output_kecepatan)
