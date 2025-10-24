from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import multitask, wait, StopWatch, run_task


# -----------------------------------------
# CONSTANTES E PARÂMETROS GLOBAIS
# -----------------------------------------
hub = PrimeHub()
motoresquerdo = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motordireito = Motor(Port.A, Direction.CLOCKWISE)
garra_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
distancia_sensor = UltrasonicSensor(Port.F)
sensor_dir = ColorSensor(Port.D)
sensor_esq = ColorSensor(Port.C)
wheel_diameter = 56  # Ajuste este valor para o diâmetro real da sua roda
axle_track = 163     # Ajuste este valor para a distância real entre as rodas
drive_base = DriveBase(motoresquerdo, motordireito, wheel_diameter, axle_track)


# PID padrão do seguidor
pid_p = 3.30
pid_i = 0.002
pid_d = 0.016

# Parâmetros do gyro_turn
GYRO_TOL = 0.5   # Tolerância para giro (graus)
GYRO_MIN = 50   # Potência mínima
GYRO_MAX = 70     # Potência máxima
GYRO_RESET = True

# -----------------------------------------
# VARIÁVEIS GLOBAIS MUTÁVEIS
# -----------------------------------------
erro = 0
correcao = 0
integral = 0
derivada = 0
erro_final = 0
guinadarefmove = 0
guinadacalc = 0
method_stop = 0
Recuo = 0 
trilho = 0
angulo_garra = 0
fator_calibracao = 1.0
todas_cores = ["cinza", "amarelo", "verde", "azul"]
carrinhos_vistos = []

# -----------------------------------------
# PID E UTILITÁRIOS
# -----------------------------------------
async def redefinir_pid():
    global erro_final, integral, derivada, correcao, erro
    erro_final = 0
    integral = 0
    derivada = 0
    correcao = 0
    erro = 0
    await wait(0)

async def PID(kp, ki, kd):
    global integral, derivada, correcao, erro_final
    integral += erro
    derivada = erro - erro_final
    correcao = erro * kp + (integral * ki + derivada * kd)
    erro_final = erro
    await wait(0)

async def redefinir():
    global erro_final, integral, derivada, correcao, erro, method_stop, pid_p, pid_i, pid_d, guinadacalc, guinadarefmove
    await wait(0)
    erro_final = 0
    integral = 0
    derivada = 0
    correcao = 0
    erro = 0
    method_stop = 0
    pid_p = 3.30
    pid_i = 0.002
    pid_d = 0.016
    guinadacalc = hub.imu.tilt()[1]
    guinadarefmove = hub.imu.tilt()[1]

# -----------------------------------------
# GIRO UNIVERSAL COM RESIDUAL
# -----------------------------------------
# Parâmetros do GyroTurn
GYRO_TOL = 0.05
GYRO_MIN = 30
GYRO_MAX = 60

async def PID(kp, ki, kd, erro, integral, erro_final):
    integral += erro
    derivado = erro - erro_final
    correcao = kp * erro + ki * integral + kd * derivado
    erro_final = erro
    return correcao, integral, erro_final




async def GyroTurn(graus):
    integral = 0
    erro_final = 0
    correcao = 0

    kp = 4
    ki = 0.0008
    kd = 0.16

    hub.imu.reset_heading(0)
    alvo = graus

    def erro_angular(alvo, atual):
        return ((alvo - atual + 540) % 360) - 180

    erro = erro_angular(alvo, hub.imu.heading())
    while abs(erro) > GYRO_TOL:
        correcao, integral, erro_final = PID(kp, ki, kd, erro, integral, erro_final, wait)
        potencia = int(min(GYRO_MAX, max(GYRO_MIN, abs(correcao))))
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



# GYRO_MOVE COM AJUSTE RESIDUAL NO FINAL
# -----------------------------------------
async def Gyro_Move(rotacoes, velocidade_final, angle, reverso=False):
    global erro, correcao
    await redefinir_pid()

    # Reset dos dois motores (importante quando usamos média)
    motoresquerdo.reset_angle(0)
    motordireito.reset_angle(0)
    await wait(0)
    
    alvo_heading = angle
    rotacoes_corrigidas = rotacoes * fator_calibracao

    # Configuração de aceleração
    velocidade_final = abs(velocidade_final)
    velocidade_atual = 40
    incremento = 2
    desacelera_a_partir = 0.8 * abs(rotacoes_corrigidas)

    sinal = -1 if reverso else 1
    corr_sinal = -1 if reverso else 1

    # Inicia movimento
    motoresquerdo.dc(sinal * (velocidade_atual - corr_sinal * correcao))
    motordireito.dc(sinal * (velocidade_atual + corr_sinal * correcao))

    while True:
        # usa a média dos dois motores
        rot_atual_esq = abs(motoresquerdo.angle() / 360)
        rot_atual_dir = abs(motordireito.angle() / 360)
        rot_media = (rot_atual_esq + rot_atual_dir) / 2

        # debug opcional
        # print(f"rot_esq={rot_atual_esq:.3f}, rot_dir={rot_atual_dir:.3f}, rot_media={rot_media:.3f}")

        if rot_media >= abs(rotacoes_corrigidas):
            break

        erro = ((alvo_heading - hub.imu.heading() + 540) % 360) - 180

        # PID (mesmo para frente e ré aqui)
        await PID(8.9, 0.009, 0.05)

        # Aqui USEI rot_media (antes estava rot_atual -> causava NameError)
        if rot_media < 0.3 * abs(rotacoes_corrigidas):
            if velocidade_atual < velocidade_final:
                velocidade_atual += incremento
        elif rot_media > desacelera_a_partir:
            velocidade_atual -= incremento
            if velocidade_atual < 50:
                velocidade_atual = 50

        if reverso:
            pot_esq = sinal * (velocidade_atual - correcao)
            pot_dir = sinal * (velocidade_atual + correcao)
        else:
            pot_esq = sinal * (velocidade_atual+ correcao)
            pot_dir = sinal * (velocidade_atual - correcao)

        motoresquerdo.dc(pot_esq)
        motordireito.dc(pot_dir)

    motoresquerdo.brake()
    motordireito.brake()

    # Correção fina de heading (igual à sua versão)
    erro_residual = ((alvo_heading - hub.imu.heading() + 540) % 360) - 180
    if abs(erro_residual) > 0.03:
        pulso_pot = 20
        pulso_tempo = 80
        print(f"Corrigindo heading final: erro = {erro_residual:.2f}°")

        if erro_residual > 0:
            motordireito.dc(-pulso_pot)
            motoresquerdo.dc(pulso_pot)
        else:
            motordireito.dc(pulso_pot)
            motoresquerdo.dc(-pulso_pot)

        await wait(pulso_tempo)
        motoresquerdo.brake()
        motordireito.brake()

    print("✅ Gyro_Move concluído. Heading atual:", hub.imu.heading())


# =========================================================
# GYRO MOVE INFINITO CORRIGIDO
# =========================================================
# =========================================================
# GYRO MOVE INFINITO CORRIGIDO FINAL
# =========================================================
async def GyroMoveInfinito(velocidade_final, angle):

    global erro, correcao

    # 🔧 Inicialização
    await redefinir()
    motoresquerdo.reset_angle(0)
    motordireito.reset_angle(0)
    await wait(0)

    alvo_heading = angle

    #Parâmetros de aceleração
    velocidade_final = abs(velocidade_final)
    velocidade_atual = 30
    incremento = 2
    velocidade_minima = 35

    # Loop principal
    while True:
        distancia_cm = await distancia_sensor.distance()

        #Condição de parada:
        # - Ambos sensores detectam preto (reflexão baixa)
        # - Obstáculo a <= 45 cm
        if (await sensor_dir.reflection() < 20 and await sensor_esq.reflection() < 20) or distancia_cm <= 45:
            break

        #Correção de direção com base no heading
        erro = ((alvo_heading - hub.imu.heading() + 540) % 360) - 180
        await PID(7.0, 0.080, 0.0070)

        #Aceleração progressiva
        if velocidade_atual < velocidade_final:
            velocidade_atual += incremento
        velocidade_atual = max(velocidade_minima, min(velocidade_atual, velocidade_final))

        #Movimento corrigido
        motoresquerdo.dc(velocidade_atual + correcao)
        motordireito.dc(velocidade_atual - correcao)

        await wait(10)

    #Parada suave
    motoresquerdo.hold()
    motordireito.hold()

    #Correção de heading final (pequeno ajuste se necessário)
    erro_residual = ((alvo_heading - hub.imu.heading() + 540) % 360) - 180
    if abs(erro_residual) > 0.05:
        pulso_pot = 15
        pulso_tempo = 80

        if erro_residual > 0:
            motoresquerdo.dc(pulso_pot)
            motordireito.dc(-pulso_pot)
        else:
            motoresquerdo.dc(-pulso_pot)
            motordireito.dc(pulso_pot)

        await wait(pulso_tempo)

    motoresquerdo.brake()
    motordireito.brake()

    print(f"[✅] GyroMoveInfinito concluído. Heading final: {hub.imu.heading():.2f}")






async def main():
#IDA 1
    await Gyro_Move(0.3, 80, 0)  
    await GyroMoveInfinito(65, 0)
    await Gyro_Move(0.3, 80, 0, reverso= True)
    await GyroTurn(90)    #GIRAR 90 PRA DIREITA (1)
    await GyroMoveInfinito(65, 90)
    await Gyro_Move(0.3, 80, 90, reverso= True)
    await GyroTurn(180)   #GIRAR MAIS 90 PRA DIREITA (2)
    await GyroMoveInfinito(65, 180)
    await Gyro_Move(0.3, 80, 180, reverso= True)
    await GyroTurn(270)   #GIRAR MAIS 90 PRA DIREITA (3)
    await GyroMoveInfinito(65, 270)
    await Gyro_Move(0.3, 80, 270, reverso= True)
    await GyroTurn(360)   #GIRAR MAIS 90 PRA DIREITA (4)
    await GyroMoveInfinito(65, 360)
    await Gyro_Move(0.3, 80, 360, reverso= True)
    await GyroTurn(450)   #GIRAR MAIS 90 PRA DIREITA (5)
    await GyroMoveInfinito(65, 450)
    await Gyro_Move(0.3, 80, 450, reverso= True)
    await GyroTurn(540)   #GIRAR MAIS 90 PRA DIREITA (6)
    await GyroMoveInfinito(65, 540)
    await Gyro_Move(0.3, 80, 540, reverso= True)
    await GyroTurn(720)   #GIRAR 180 PRA DIREITA
    garra_motor.run_time(-300, 500) #GARRA DESCE
    hub.imu.reset_heading(0)
# #ENTREGA/VOLTA 1
#     await GyroMoveInfinito(100, 0)
#     await gyro_turn(-90)  #GIRAR 90 PRA ESQUERDA (1)
#     await GyroMoveInfinito(100, -90)
#     await gyro_turn(-180)  #GIRAR 90 PRA ESQUERDA (2)
#     await GyroMoveInfinito(100, -180)
#     await gyro_turn(-270)  #GIRAR 90 PRA ESQUERDA (3)
#     await GyroMoveInfinito(100, -270)
#     await gyro_turn(-360)  #GIRAR 90 PRA ESQUERDA (4)
#     await GyroMoveInfinito(100, -360)
#     await gyro_turn(-450)  #GIRAR 90 PRA ESQUERDA (5)
#     await GyroMoveInfinito(100, -450)
#     await gyro_turn(-540)  #GIRAR 90 PRA ESQUERDA (6)
#     await GyroMoveInfinito(100, -540)
#     await gyro_turn(720)       #180
#     garra_motor.run_time(300, 500) #GARRA Sobe
#     hub.imu.reset_heading(0)

# #IDA 2
#     await Gyro_Move(0.3, 80, 0)  
#     await GyroMoveInfinito(100, 0)
#     await gyro_turn(90)    #GIRAR 90 PRA DIREITA (1)
#     await GyroMoveInfinito(100, 90)
#     await gyro_turn(180)   #GIRAR MAIS 90 PRA DIREITA (2)
#     await GyroMoveInfinito(100, 180)
#     await gyro_turn(270)   #GIRAR MAIS 90 PRA DIREITA (3)
#     await GyroMoveInfinito(100, 270)
#     await gyro_turn(360)   #GIRAR MAIS 90 PRA DIREITA (4)
#     await GyroMoveInfinito(100, 360)
#     await gyro_turn(450)   #GIRAR MAIS 90 PRA DIREITA (5)
#     await GyroMoveInfinito(100, 450)
#     await gyro_turn(540)   #GIRAR MAIS 90 PRA DIREITA (6)
#     await GyroMoveInfinito(100, 540)
#     await gyro_turn(720)   #GIRAR 180 PRA DIREITA
#     garra_motor.run_time(-300, 500) #GARRA DESCE
#     hub.imu.reset_heading(0)
# #ENTREGA/VOLTA 2
#     await GyroMoveInfinito(100, 0)
#     await gyro_turn(-90)  #GIRAR 90 PRA ESQUERDA (1)
#     await GyroMoveInfinito(100, -90)
#     await gyro_turn(-180)  #GIRAR 90 PRA ESQUERDA (2)
#     await GyroMoveInfinito(100, -180)
#     await gyro_turn(-270)  #GIRAR 90 PRA ESQUERDA (3)
#     await GyroMoveInfinito(100, -270)
#     await gyro_turn(-360)  #GIRAR 90 PRA ESQUERDA (4)
#     await GyroMoveInfinito(100, -360)
#     await gyro_turn(-450)  #GIRAR 90 PRA ESQUERDA (5)
#     await GyroMoveInfinito(100, -450)
#     await gyro_turn(-540)  #GIRAR 90 PRA ESQUERDA (6)
#     await GyroMoveInfinito(100, -540)
#     await gyro_turn(720)       #180
#     garra_motor.run_time(300, 500) #GARRA Sobe
#     hub.imu.reset_heading(0)

    #print(await sensor_dir.hsv(), await sensor_esq.hsv())
    


run_task(main())