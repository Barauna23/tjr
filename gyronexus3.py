# Importa as bibliotecas necessárias da Pybricksfrom pybricks.hubs import PrimeHub
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Stop, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from confignexus2 import *

    # --- INICIALIZAÇÃO DO HARDWARE ---


def redefinir():
    global last_error, integral, derivado, correcao, erro, methodstop
    last_error = 0
    integral = 0
    derivado = 0
    correction = 0
    erro = 0
    methodstop = 0

erro = 0
correction = 0
integral = 0
derivado = 0
last_error = 0

# Parâmetros do GyroTurn
GYRO_TOL = 0.05
GYRO_MIN = 30
GYRO_MAX = 50

def PID(kp, ki, kd, erro, integral, last_error, wait_func):
    integral += erro
    derivado = erro - last_error
    correction = kp * erro + ki * integral + kd * derivado
    last_error = erro
    wait_func(0)
    return correction, integral, last_error




def GyroTurn(graus):
    integral = 0
    last_error = 0
    correction = 0

    kp = 2.1
    ki = 0.0008
    kd = 0.08

    hub.imu.reset_heading(0)
    alvo = graus

    def erro_angular(alvo, atual):
        return ((alvo - atual + 540) % 360) - 180

    erro = erro_angular(alvo, hub.imu.heading())
    while abs(erro) > GYRO_TOL:
        correction, integral, last_error = PID(kp, ki, kd, erro, integral, last_error, wait)
        potencia = int(min(GYRO_MAX, max(GYRO_MIN, abs(correction))))
        if erro > 0:
            right_motor.dc(-potencia)
            left_motor.dc(potencia)
        else:
            right_motor.dc(potencia)
            left_motor.dc(-potencia)
        erro = erro_angular(alvo, hub.imu.heading())

    right_motor.brake()
    left_motor.brake()

    # Correção fina
    erro_residual = erro_angular(alvo, hub.imu.heading())
    if abs(erro_residual) > 0.2:
        pulso_pot = 5
        if erro_residual > 0:
            right_motor.dc(pulso_pot)
            left_motor.dc(-pulso_pot)
        else:
            right_motor.dc(-pulso_pot)
            left_motor.dc(pulso_pot)
        right_motor.brake()
        left_motor.brake()


def gyro_move(angulacao, velocidade , distancia):
        # Move o robô para frente por uma distância, corrigindo com o giroscópio
    erro = 0
    integral = 0
    last_error = 0
    kp = 6.5
    ki = 0.001
    kd = 0.8
    angulo_inicial = angulacao
    hub.imu.reset_heading(0)
    robot.reset(0, 0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    rot_atual = 0
    while True:
        rot_atual = abs(left_motor.angle() / 360) + abs(right_motor.angle() / 360) / 2
        if rot_atual >= abs(distancia):
            break
        angulo_atual = hub.imu.heading()
        erro = angulo_atual - angulo_inicial
        PID(kp, ki, kd, erro, integral, last_error, wait)
       # ajuste o ganho conforme necessário
        left_motor.dc(velocidade + correction)
        right_motor.dc(velocidade - correction)
        last_error = erro
        wait(10)
    left_motor.brake()
    right_motor.brake()
def movimentation(kp, ki , kd, erro, integral, last_error, wait, speed):
    angulo_atual = hub.imu.heading()
    erro = angulo_atual - 0
    PID(kp, ki, kd, erro, integral, last_error, wait)
       # ajuste o ganho conforme necessário
    left_motor.dc(speed + correction)
    right_motor.dc(speed - correction)
    last_error = erro
    wait(10)

# Método universal para movimentação com diferentes critérios de parada
def gyro_universal(mode, velocidade, parametro=None):
    redefinir()
    erro = 0
        # Move o robô para frente por uma distância, corrigindo com o giroscópio
    erro = 0
    integral = 0
    last_error = 0
    kp = 1.5
    ki = 0.001
    kd = 0.8
    angulo_inicial = 0
    hub.imu.reset_heading(0)
    robot.reset(0, 0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
 
    if mode == "dois_pretos": # ... lógica de movimentação ...
        while color_sensor_esquerdo.reflection() > 10 and color_sensor_direito.reflection() > 10:
            movimentation(kp, ki, kd, erro, integral, last_error, wait, velocidade)
        left_motor.brake()
        right_motor.brake()


    elif mode == "um_preto":  # ... lógica de movimentação ...
        while color_sensor_esquerdo.reflection() > 20 or color_sensor_direito.reflection() > 20:
            movimentation(kp, ki, kd, erro, integral, last_error, wait, velocidade)
        left_motor.brake()
        right_motor.brake()


    elif mode == "distancia":  # ... lógica de movimentação ...
        while True:
            movimentation(kp, ki, kd, erro, integral, last_error, wait, velocidade)
            if ultra.distance() <= parametro:
                break
        left_motor.brake()
        right_motor.brake()


    elif mode == "angulo":
        while True:
            movimentation(kp, ki, kd, erro, integral, last_error, wait, velocidade)
            rot_atual = abs(left_motor.angle() / 360) + abs(right_motor.angle() / 360) / 2
            if rot_atual >= abs(parametro):
                break