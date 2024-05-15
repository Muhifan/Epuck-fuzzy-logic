from controller import Robot, DistanceSensor, Motor
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Inisialisasi robot
robot = Robot()

# Mendefinisikan waktu siklus dan kecepatan maksimum
TIME_STEP = 64
MAX_SPEED = 5

# Mendefinisikan sensor jarak
ps11 = robot.getDistanceSensor("ps11")
ps11.enable(TIME_STEP)
ps9 = robot.getDistanceSensor("ps9")
ps9.enable(TIME_STEP)
ps10 = robot.getDistanceSensor("ps10")
ps10.enable(TIME_STEP)

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
    jarak_kiri_value = ps9.getValue()
    jarak_kanan_value = ps11.getValue()
    jarak_tengah_value = ps10.getValue()
    
    # Cetak pembacaan sensor
    print(f"sensor PS9 (depan kiri): {jarak_kiri_value:.2f} || sensor PS10 (depan tengah): {jarak_tengah_value:.2f} || sensor PS11 (depat kanan): {jarak_kanan_value:.2f}")
    # print("Pembacaan sensor PS9 (sebelah kiri):", jarak_kiri_value)
    # print("Pembacaan sensor PS11 (sebelah kanan):", jarak_kanan_value)
    # print("Pembacaan sensor PS10 (sebelah tengah):", jarak_tengah_value)

    # Memberikan input ke dalam sistem kontrol fuzzy
    kecepatan_simulasi.input['jarak_kanan'] = jarak_kanan_value
    kecepatan_simulasi.input['jarak_kiri'] = jarak_kiri_value

    # Melakukan perhitungan
    kecepatan_simulasi.compute()

    # Mendapatkan output kecepatan
    output_kecepatan = kecepatan_simulasi.output['kecepatan']

    # Mengatur kecepatan motor
    if jarak_tengah_value <= 750:  # Jika ada objek di depan (PS10 membaca 502)
        # Memperlambat secara bertahap hingga berhenti
        left_motor.setVelocity(left_motor.getVelocity() * 0.9)
        right_motor.setVelocity(right_motor.getVelocity() * 0.9)
        # Jika kecepatan sudah cukup rendah, berhenti total
        if abs(left_motor.getVelocity()) < 0.1 and abs(right_motor.getVelocity()) < 0.1:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
    elif jarak_kanan_value < 600:   # Jika ada objek di sebelah kanan (PS11 membaca < 1.0)
        left_motor.setVelocity(-MAX_SPEED * 1.5)
        right_motor.setVelocity(MAX_SPEED * 1.5)
    elif jarak_kiri_value < 600:    # Jika ada objek di sebelah kiri (PS9 membaca < 1.0)
        left_motor.setVelocity(MAX_SPEED * 1.5)
        right_motor.setVelocity(-MAX_SPEED * 1.5)
    else:                            # Jika tidak ada halangan, teruskan bergerak sesuai kecepatan fuzzy
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)
