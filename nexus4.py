from gyronexus3 import *
from confignexus3 import *

def main():
#Primeira volta
    gyro_universal("dois_pretos", 50)
    gyro_universal("angulo", 40, 0.5)
    wait(100)

    for _ in range(6):
        gyro_universal("dois_pretos", 45)
        wait(100)
        gyro_universal("angulo", -60, 0.02)
        wait(100)
        GyroTurn(90)
        print(hub.imu.heading())
        wait(100)

    gyro_universal("distancia", 40, 40)
    wait(100)
    gyro_universal("angulo", -40, 0.3)
    wait(500)
    GyroTurn(90)
    wait(100)
    gyro_universal("angulo", -40, 0.1)
    wait(100)
    GyroTurn(90)
    wait(100)
    gyro_universal("angulo", -40, 0.09)
    garra_motor.run_angle(-200, 180)
    wait(100)
    gyro_universal("angulo", 40, 0.1)

    for _ in range(6):
        gyro_universal("dois_pretos", 45)
        wait(100)
        gyro_universal("angulo", -60, 0.02)
        wait(100)
        GyroTurn(-90)
        wait(100)

    gyro_universal("dois_pretos", 45)
    wait(100)
    GyroTurn(180)
    wait(100)
    gyro_universal("angulo", 60, 0.04)
    wait(100)
    garra_motor.run_angle(200, 180)
    wait(100)



main()
