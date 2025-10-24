from gyronexus2 import *
from confignexus2 import *

def main():
#Primeira volta
    gyro_universal("dois_pretos", 70)
    gyro_universal("angulo", 60, 0.5)

    for _ in range(6):
        gyro_universal("dois_pretos", 70)
        wait(100)
        gyro_universal("angulo", -60, 0.1)
        wait(100)
        GyroTurn(90)
        wait(100)

    gyro_universal("distancia", 70, 40)
    wait(100)
    GyroTurn(90)
    wait(100)
    gyro_universal("dois_pretos", 70)
    wait(100)
    GyroTurn(90)
    wait(100)
    garra_motor.run_time(-400, 800)
    wait(100)

    for _ in range(6):
        gyro_universal("dois_pretos", 70)
        wait(100)
        gyro_universal("angulo", -60, 0.2)
        wait(100)
        GyroTurn(-90)
        wait(100)

    gyro_universal("dois_pretos", 70)
    wait(100)
    GyroTurn(180)
    wait(100)
    gyro_universal("angulo", 60, 1)
    wait(100)
    garra_motor.run_time(400, 800)   
    wait(1000)

#Segunda volta
    for _ in range(6):
        gyro_universal("dois_pretos", 70)
        wait(100)
        gyro_universal("angulo", -60, 0.8)
        wait(100)
        GyroTurn(90)
        wait(100)

    gyro_universal("distancia", 70, 40)
    wait(100)
    GyroTurn(180)
    wait(100)
    garra_motor.run_time(-400, 800)
    wait(100)

    for _ in range(6):
        gyro_universal("dois_pretos", 70)
        wait(100)
        gyro_universal("angulo", -60, 0.8)
        wait(100)
        GyroTurn(-90)
        wait(100)

    gyro_universal("dois_pretos", 70)
    wait(100)
    GyroTurn(180)
    wait(100)
    gyro_universal("angulo", 60, 1)
    wait(100)
    garra_motor.run_time(400, 800)
    wait(100)

main()
