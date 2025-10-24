# Importa as bibliotecas necessárias da Pybricks from pybricks.hubs import PrimeHub
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Stop, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

    # --- INICIALIZAÇÃO DO HARDWARE ---
    # Inicializa o Hub
hub = PrimeHub()
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A, Direction.CLOCKWISE)
garra_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
ultra = UltrasonicSensor(Port.F)
color_sensor_direito = ColorSensor(Port.D)
color_sensor_esquerdo = ColorSensor(Port.C)
wheel_diameter = 55  # Ajuste este valor para o diâmetro real da sua roda
axle_track = 164     # Ajuste este valor para a distância real entre as rodas
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)