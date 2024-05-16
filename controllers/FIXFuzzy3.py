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


def calculate_motor(parameter): 
    return(parameter/10) * 6.28

normal_speed = 1

# Loop utama
while robot.step(TIME_STEP) != -1:
    # Membaca jarak dari sensor ultrasonik
    jarak_kiri_value = ps9.getValue()
    jarak_kanan_value = ps11.getValue()
    jarak_tengah_value = ps10.getValue()

    sensor_depan_kiri = jarak_kiri_value
    sensor_depan_tengah = jarak_tengah_value
    sensor_depan_kanan = jarak_kanan_value

    # Range sensor
    x_sensor_depan_kiri = np.arange(0, 1010, 0.1)
    x_sensor_depan_tengah = np.arange(0, 1010, 0.1)
    x_sensor_depan_kanan = np.arange(0, 1010, 0.1)
    # Range motor
    x_kategori_kanan = np.arange(0, 6.28, 0.1)
    x_kategori_kiri = np.arange(0, 6.28, 0.1)

    # Sensor depan kiri
    sensor_depan_kiri_dekat = fuzz.trapmf(x_sensor_depan_kiri, [0, 0, 600, 700])
    sensor_depan_kiri_sedang = fuzz.trapmf(x_sensor_depan_kiri, [650, 700, 800, 850])
    sensor_depan_kiri_jauh = fuzz.trapmf(x_sensor_depan_kiri, [800, 850, 1005, 1005])

    # Sensor depan tengah
    sensor_depan_tengah_dekat = fuzz.trapmf(x_sensor_depan_tengah, [0, 0, 600, 700])
    sensor_depan_tengah_sedang = fuzz.trapmf(x_sensor_depan_tengah, [650, 700, 800, 850])
    sensor_depan_tengah_jauh = fuzz.trapmf(x_sensor_depan_tengah, [800, 850, 1005, 1005])

    # Sensor depan kanan
    sensor_depan_kanan_dekat = fuzz.trapmf(x_sensor_depan_kanan, [0, 0, 650, 700])
    sensor_depan_kanan_sedang = fuzz.trapmf(x_sensor_depan_kanan, [650, 700, 800, 850])
    sensor_depan_kanan_jauh = fuzz.trapmf(x_sensor_depan_kanan, [800, 850, 1005, 1005])

    #Kecepatan kanan 
    stop_kanan = fuzz.trapmf(x_kategori_kanan, [0, 0, 0.2, 0.5])
    lambat_kanan = fuzz.trapmf(x_kategori_kanan, [0, 1, 2, 3])
    normal_kanan = fuzz.trapmf(x_kategori_kanan, [2, 3, 4, 5])
    cepat_kanan = fuzz.trapmf(x_kategori_kanan, [4, 5, 6.28, 6.28])

    #kecepatan Kiri
    stop_kiri = fuzz.trapmf(x_kategori_kiri, [0, 0, 0.2, 0.5])
    lambat_kiri = fuzz.trapmf(x_kategori_kiri, [0, 1, 2, 3])
    normal_kiri = fuzz.trapmf(x_kategori_kiri, [2, 3, 4, 5])
    cepat_kiri = fuzz.trapmf(x_kategori_kiri, [4, 5, 6.28, 6.28])
    
    # Membership function untuk sensor depan kiri
    
    sensor_depan_kiri_dkt = fuzz.interp_membership(x_sensor_depan_kiri, sensor_depan_kiri_dekat, sensor_depan_kiri)
    sensor_depan_kiri_sdg = fuzz.interp_membership(x_sensor_depan_kiri, sensor_depan_kiri_sedang, sensor_depan_kiri)
    sensor_depan_kiri_jh = fuzz.interp_membership(x_sensor_depan_kiri, sensor_depan_kiri_jauh, sensor_depan_kiri)

    # Membership function untuk sensor depan tengah
    
    sensor_depan_tengah_dkt = fuzz.interp_membership(x_sensor_depan_tengah, sensor_depan_tengah_dekat, sensor_depan_tengah)
    sensor_depan_tengah_sdg = fuzz.interp_membership(x_sensor_depan_tengah, sensor_depan_tengah_sedang, sensor_depan_tengah)
    sensor_depan_tengah_jh = fuzz.interp_membership(x_sensor_depan_tengah, sensor_depan_tengah_jauh, sensor_depan_tengah)

    # Membership function untuk sensor depan kanan
    
    sensor_depan_kanan_dkt = fuzz.interp_membership(x_sensor_depan_kanan, sensor_depan_kanan_dekat, sensor_depan_kanan)
    sensor_depan_kanan_sdg = fuzz.interp_membership(x_sensor_depan_kanan, sensor_depan_kanan_sedang, sensor_depan_kanan)
    sensor_depan_kanan_jh = fuzz.interp_membership(x_sensor_depan_kanan, sensor_depan_kanan_jauh, sensor_depan_kanan)

    # Rules 
    rules_1 = np.fmin(sensor_depan_kiri_dkt, np.fmin(sensor_depan_tengah_dkt, sensor_depan_kanan_dkt))
    rules_2 = np.fmin(sensor_depan_kiri_dkt, np.fmin(sensor_depan_tengah_dkt, sensor_depan_kanan_sdg))
    rules_3 = np.fmin(sensor_depan_kiri_dkt, np.fmin(sensor_depan_tengah_dkt, sensor_depan_kanan_jh))
    rules_4 = np.fmin(sensor_depan_kiri_dkt, np.fmin(sensor_depan_tengah_sdg, sensor_depan_kanan_dkt))
    rules_5 = np.fmin(sensor_depan_kiri_dkt, np.fmin(sensor_depan_tengah_sdg, sensor_depan_kanan_sdg))
    rules_6 = np.fmin(sensor_depan_kiri_dkt, np.fmin(sensor_depan_tengah_sdg, sensor_depan_kanan_jh))
    rules_7 = np.fmin(sensor_depan_kiri_dkt, np.fmin(sensor_depan_tengah_jh, sensor_depan_kanan_dkt))
    rules_8 = np.fmin(sensor_depan_kiri_dkt, np.fmin(sensor_depan_tengah_jh, sensor_depan_kanan_sdg))
    rules_9 = np.fmin(sensor_depan_kiri_dkt, np.fmin(sensor_depan_tengah_jh, sensor_depan_kanan_jh))
    rules_10 = np.fmin(sensor_depan_kiri_sdg, np.fmin(sensor_depan_tengah_dkt, sensor_depan_kanan_dkt))
    rules_11 = np.fmin(sensor_depan_kiri_sdg, np.fmin(sensor_depan_tengah_dkt, sensor_depan_kanan_sdg))
    rules_12 = np.fmin(sensor_depan_kiri_sdg, np.fmin(sensor_depan_tengah_dkt, sensor_depan_kanan_jh))
    rules_13 = np.fmin(sensor_depan_kiri_sdg, np.fmin(sensor_depan_tengah_sdg, sensor_depan_kanan_dkt))
    rules_14 = np.fmin(sensor_depan_kiri_sdg, np.fmin(sensor_depan_tengah_sdg, sensor_depan_kanan_sdg))
    rules_15 = np.fmin(sensor_depan_kiri_sdg, np.fmin(sensor_depan_tengah_sdg, sensor_depan_kanan_jh))
    rules_16 = np.fmin(sensor_depan_kiri_sdg, np.fmin(sensor_depan_tengah_jh, sensor_depan_kanan_dkt))
    rules_17 = np.fmin(sensor_depan_kiri_sdg, np.fmin(sensor_depan_tengah_jh, sensor_depan_kanan_sdg))
    rules_18 = np.fmin(sensor_depan_kiri_sdg, np.fmin(sensor_depan_tengah_jh, sensor_depan_kanan_jh))
    rules_19 = np.fmin(sensor_depan_kiri_jh, np.fmin(sensor_depan_tengah_dkt, sensor_depan_kanan_dkt))
    rules_20 = np.fmin(sensor_depan_kiri_jh, np.fmin(sensor_depan_tengah_dkt, sensor_depan_kanan_sdg))
    rules_21 = np.fmin(sensor_depan_kiri_jh, np.fmin(sensor_depan_tengah_dkt, sensor_depan_kanan_jh))
    rules_22 = np.fmin(sensor_depan_kiri_jh, np.fmin(sensor_depan_tengah_sdg, sensor_depan_kanan_dkt))
    rules_23 = np.fmin(sensor_depan_kiri_jh, np.fmin(sensor_depan_tengah_sdg, sensor_depan_kanan_sdg))
    rules_24 = np.fmin(sensor_depan_kiri_jh, np.fmin(sensor_depan_tengah_sdg, sensor_depan_kanan_jh))
    rules_25 = np.fmin(sensor_depan_kiri_jh, np.fmin(sensor_depan_tengah_jh, sensor_depan_kanan_dkt))
    rules_26 = np.fmin(sensor_depan_kiri_jh, np.fmin(sensor_depan_tengah_jh, sensor_depan_kanan_sdg))
    rules_27 = np.fmin(sensor_depan_kiri_jh, np.fmin(sensor_depan_tengah_jh, sensor_depan_kanan_jh))

    # Pemggabungan motor kanan
    Persentase_stop_kanan = np.fmin(rules_1, np.fmax(rules_9, stop_kanan))
    Persentase_lambat_kanan = np.fmin(np.fmax(rules_3, np.fmax(rules_4, np.fmax(rules_5, np.fmax(rules_6, np.fmax(rules_7, rules_8))))), lambat_kanan)
    Persentase_normal_kanan = np.fmin(np.fmax(rules_10, np.fmax(rules_11, np.fmax(rules_12, np.fmax(rules_13, np.fmax(rules_14, np.fmax(rules_15, np.fmax(rules_16, np.fmax(rules_17, rules_18)))))))), normal_kanan)
    Persentase_cepat_kanan = np.fmin(np.fmax(rules_19, np.fmax(rules_20, np.fmax(rules_21, np.fmax(rules_22, np.fmax(rules_23, np.fmax(rules_24, np.fmax(rules_25, np.fmax(rules_26, rules_27)))))))), cepat_kanan)
    
    # Pemggabungan motor kiri
    Persentase_stop_kiri = np.fmin(rules_1, np.fmax(rules_25, stop_kiri))
    Persentase_lambat_kiri = np.fmin(np.fmax(rules_4, np.fmax(rules_7, np.fmax(rules_10, np.fmax(rules_13, np.fmax(rules_16, np.fmax(rules_19, rules_22)))))), lambat_kiri)
    Persentase_normal_kiri = np.fmin(np.fmax(rules_2, np.fmax(rules_5, np.fmax(rules_8, np.fmax(rules_11, np.fmax(rules_14, np.fmax(rules_17, np.fmax(rules_20, np.fmax(rules_23, rules_26)))))))), normal_kiri)
    Persentase_cepat_kiri = np.fmin(np.fmax(rules_3, np.fmax(rules_6, np.fmax(rules_9, np.fmax(rules_12, np.fmax(rules_15, np.fmax(rules_18, np.fmax(rules_21, np.fmax(rules_24, rules_27)))))))), cepat_kiri)
    
    hasil_kanan = np.zeros_like(x_kategori_kanan)
    hasil_kiri = np.zeros_like(x_kategori_kiri)

    # Defuzzifikasi kanan
    aggregated_kanan = np.fmax(Persentase_stop_kanan, np.fmax(Persentase_lambat_kanan, np.fmax(Persentase_normal_kanan, Persentase_cepat_kanan)))
    kecepatan_motor_kanan = fuzz.defuzz(x_kategori_kanan, aggregated_kanan, 'centroid')
    hasil_vic = fuzz.interp_membership(x_kategori_kanan, aggregated_kanan, kecepatan_motor_kanan)

    # Defuzzifikasi kiri
    aggregated_kiri = np.fmax(Persentase_stop_kiri, np.fmax(Persentase_lambat_kiri, np.fmax(Persentase_normal_kiri, Persentase_cepat_kiri)))
    kecepatan_motor_kiri = fuzz.defuzz(x_kategori_kiri, aggregated_kiri, 'centroid')
    hasil_vic = fuzz.interp_membership(x_kategori_kiri, aggregated_kiri, kecepatan_motor_kiri)

    motor_kanan_fix = calculate_motor (kecepatan_motor_kanan)
    motor_kiri_fix = calculate_motor (kecepatan_motor_kiri)

    motor_kiri = normal_speed + motor_kiri_fix
    motor_kanan = normal_speed + motor_kanan_fix

    left_motor.setVelocity(motor_kiri)
    right_motor.setVelocity(motor_kanan)
    
    if jarak_tengah_value <= 700:  # Jika ada objek di depan (PS10 membaca 502)
        # Memperlambat secara bertahap hingga berhenti
        left_motor.setVelocity(left_motor.getVelocity() - 2)
        right_motor.setVelocity(right_motor.getVelocity() - 2)
        # Jika kecepatan sudah cukup rendah, berhenti total
        if abs(left_motor.getVelocity()) < 1.5 and abs(right_motor.getVelocity()) < 1.5:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)

    # Cetak pembacaan sensor
    print (motor_kiri)
    print (motor_kanan)
    # print (aggregated_kanan)
    # print (aggregated_kiri)
    print(f"sensor PS9 (depan kiri): {sensor_depan_kiri:.2f} || sensor PS10 (depan tengah): {sensor_depan_tengah:.2f} || sensor PS11 (depat kanan): {sensor_depan_kanan:.2f}")