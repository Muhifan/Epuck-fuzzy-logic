from controller import Robot, DistanceSensor, Motor
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt

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

# List untuk menyimpan data kecepatan
motor_kiri_data = []
motor_kanan_data = []

def calculate_motor(parameter): 
    return(parameter/10) * 6.28

normal_speed = 3

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
    x_sensor_depan_kiri = np.arange(0, 1000, 0.1)
    x_sensor_depan_tengah = np.arange(0, 1000, 0.1)
    x_sensor_depan_kanan = np.arange(0, 1000, 0.1)
    # Range motor
    x_kategori_kanan = np.arange(0, 6.28, 0.1)
    x_kategori_kiri = np.arange(0, 6.28, 0.1)

    # Sensor depan kiri
    sensor_depan_kiri_dekat = fuzz.trapmf(x_sensor_depan_kiri, [0, 0, 400, 500])
    sensor_depan_kiri_sedang = fuzz.trapmf(x_sensor_depan_kiri, [400, 500, 750, 850])
    sensor_depan_kiri_jauh = fuzz.trapmf(x_sensor_depan_kiri, [750, 850, 1000, 1000])

    # Sensor depan tengah
    sensor_depan_tengah_dekat = fuzz.trapmf(x_sensor_depan_tengah, [0, 0, 400, 500])
    sensor_depan_tengah_sedang = fuzz.trapmf(x_sensor_depan_tengah, [400, 500, 750, 850])
    sensor_depan_tengah_jauh = fuzz.trapmf(x_sensor_depan_tengah, [750, 850, 1000, 1000])

    # Sensor depan kanan
    sensor_depan_kanan_dekat = fuzz.trapmf(x_sensor_depan_kanan, [0, 0, 400, 500])
    sensor_depan_kanan_sedang = fuzz.trapmf(x_sensor_depan_kanan, [400, 500, 750, 850])
    sensor_depan_kanan_jauh = fuzz.trapmf(x_sensor_depan_kanan, [750, 850, 1000, 1000])

    #Kecepatan kanan 
    sangat_lambat_kanan = fuzz.trapmf(x_kategori_kanan, [-0.28, -0.28, 0, 1])
    lambat_kanan = fuzz.trapmf(x_kategori_kanan, [0, 1, 1.5, 2.5])
    normal_kanan = fuzz.trapmf(x_kategori_kanan, [1.5, 2.5, 3.5, 4.5])
    cepat_kanan = fuzz.trapmf(x_kategori_kanan, [3.5, 4.5, 5, 6])
    sangat_cepat_kanan = fuzz.trapmf(x_kategori_kanan, [5, 6, 6.28, 6.28])

    #kecepatan Kiri
    sangat_lambat_kiri = fuzz.trapmf(x_kategori_kiri, [-0.28, -0.28, 0, 1])
    lambat_kiri = fuzz.trapmf(x_kategori_kiri, [0, 1, 1.5, 2.5])
    normal_kiri = fuzz.trapmf(x_kategori_kiri, [1.5, 2.5, 3.5, 4.5])
    cepat_kiri = fuzz.trapmf(x_kategori_kiri, [3.5, 4.5, 5, 6])
    sangat_cepat_kiri = fuzz.trapmf(x_kategori_kiri, [5, 6, 6.28, 6.28])
    
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
    Persentase_sangat_lambat_kanan = np.fmin(np.fmax(rules_1, rules_2), sangat_lambat_kanan)
    Persentase_lambat_kanan = np.fmin(np.fmax(rules_3, np.fmax(rules_4, np.fmax(rules_7, np.fmax(rules_11, np.fmax(rules_12, np.fmax(rules_20, rules_21)))))), lambat_kanan)
    Persentase_normal_kanan = np.fmin(np.fmax(rules_5, np.fmax(rules_6, np.fmax(rules_8, np.fmax(rules_9, np.fmax(rules_14, np.fmax(rules_15, np.fmax(rules_17, np.fmax(rules_18, np.fmax(rules_23, np.fmax(rules_24, rules_16)))))))))), normal_kanan)
    Persentase_cepat_kanan = np.fmin(np.fmax(rules_13, np.fmax(rules_16, np.fmax(rules_19, np.fmax(rules_22, rules_25)))), cepat_kanan)
    Persentase_sangat_cepat_kanan = np.fmin(np.fmax(rules_27, rules_10), sangat_cepat_kanan)

    # Pemggabungan motor kiri
    Persentase_sangat_lambat_kiri = np.fmin(np.fmax(rules_1, rules_10), sangat_lambat_kiri)
    Persentase_lambat_kiri = np.fmin(np.fmax(rules_4, np.fmax(rules_7,  np.fmax(rules_11, np.fmax(rules_12, np.fmax(rules_16, np.fmax(rules_19, np.fmax(rules_20, rules_21))))))), lambat_kiri)
    Persentase_normal_kiri = np.fmin(np.fmax(rules_13, np.fmax(rules_14, np.fmax(rules_15, np.fmax(rules_17, np.fmax(rules_18, np.fmax(rules_22, np.fmax(rules_23, np.fmax(rules_24, np.fmax(rules_25, rules_26))))))))), normal_kiri)
    Persentase_cepat_kiri = np.fmin(np.fmax(rules_2, np.fmax(rules_3, np.fmax(rules_5, np.fmax(rules_6, np.fmax(rules_8, rules_9))))), cepat_kiri)
    Persentase_sangat_cepat_kiri = np.fmin(np.fmax(rules_27, rules_2), sangat_cepat_kiri)

    hasil_kanan = np.zeros_like(x_kategori_kanan)
    hasil_kiri = np.zeros_like(x_kategori_kiri)

    # Defuzzifikasi kanan
    aggregated_kanan = np.fmax(Persentase_sangat_lambat_kanan, np.fmax(Persentase_lambat_kanan, np.fmax(Persentase_normal_kanan, np.fmax(Persentase_cepat_kanan, Persentase_sangat_cepat_kanan))))
    kecepatan_motor_kanan = fuzz.defuzz(x_kategori_kanan, aggregated_kanan, 'centroid')
    hasil_vic = fuzz.interp_membership(x_kategori_kanan, aggregated_kanan, kecepatan_motor_kanan)

    # Defuzzifikasi kiri
    aggregated_kiri = np.fmax(Persentase_sangat_lambat_kiri, np.fmax(Persentase_lambat_kiri, np.fmax(Persentase_normal_kiri, np.fmax(Persentase_cepat_kiri, Persentase_sangat_cepat_kiri))))
    kecepatan_motor_kiri = fuzz.defuzz(x_kategori_kiri, aggregated_kiri, 'centroid')
    hasil_vic = fuzz.interp_membership(x_kategori_kiri, aggregated_kiri, kecepatan_motor_kiri)

    motor_kanan_fix = calculate_motor (kecepatan_motor_kanan)
    motor_kiri_fix = calculate_motor (kecepatan_motor_kiri)

    motor_kiri = normal_speed + motor_kiri_fix
    motor_kanan = normal_speed + motor_kanan_fix

    left_motor.setVelocity(motor_kiri)
    right_motor.setVelocity(motor_kanan)
    
    if sensor_depan_tengah <= 500:  # Jika ada objek di depan (PS10 membaca 700)
        # Memperlambat secara bertahap hingga berhenti
        left_motor.setVelocity(left_motor.getVelocity() - 2)
        right_motor.setVelocity(right_motor.getVelocity() - 2)
        # Jika kecepatan sudah cukup rendah, berhenti total
        if abs(left_motor.getVelocity()) < 1.5 and abs(right_motor.getVelocity()) < 1.5:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
    
    # Simpan data kecepatan untuk plot
    motor_kiri_data.append(motor_kiri)
    motor_kanan_data.append(motor_kanan)

    # Cetak pembacaan sensor
    print(f"Motor kiri: {motor_kiri_fix:.2f} || Motor kanan: {motor_kanan_fix:.2f} || sensor PS9 (depan kiri): {sensor_depan_kiri:.2f} || sensor PS10 (depan tengah): {sensor_depan_tengah:.2f} || sensor PS11 (depat kanan): {sensor_depan_kanan:.2f}")

# # Menggunakan moving average untuk smooth data
# def moving_average(data, window_size):
#     return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# window_size = 5
# motor_kiri_smooth = moving_average(motor_kiri_data, window_size)
# motor_kanan_smooth = moving_average(motor_kanan_data, window_size)

# # Plotting data kecepatan motor
# plt.figure(figsize=(12, 6))
# # plt.plot(motor_kiri_data, label='Kecepatan Motor Kiri (Original)')
# plt.plot(range(window_size-1, len(motor_kiri_data)), motor_kiri_smooth, label='Kecepatan Motor Kiri (Smoothed)')
# # plt.plot(motor_kanan_data, label='Kecepatan Motor Kanan (Original)')
# plt.plot(range(window_size-1, len(motor_kanan_data)), motor_kanan_smooth, label='Kecepatan Motor Kanan (Smoothed)')
# plt.legend()
# plt.xlabel('Time Step')
# plt.ylabel('Kecepatan Motor')
# plt.title('Grafik Kecepatan Motor')
# plt.show()