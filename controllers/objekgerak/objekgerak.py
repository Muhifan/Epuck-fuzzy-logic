from controller import Robot, Motor, DistanceSensor
import numpy as np
import time

# Inisialisasi robot
robot = Robot()

# Mendefinisikan waktu siklus dan kecepatan maksimum
TIME_STEP = 64
MAX_SPEED = 4

# Mendefinisikan motor
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Loop utama
while robot.step(TIME_STEP) != -1:
    # Set kecepatan motor
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)
    
    # Berhenti mendadak setelah 10 detik
    if robot.getTime() > 5:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        
    # Lanjutkan setelah 10 detik
    if robot.getTime() > 12:
        left_motor.setVelocity(MAX_SPEED * 1.5)
        right_motor.setVelocity(MAX_SPEED * 1.5)

    # Lanjutkan setelah 10 detik
    if robot.getTime() > 17:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)