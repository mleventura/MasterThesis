from controller  import Robot, DistanceSensor, Motor, Camera    #, GPS
from math import cos, sin
import csv
import numpy



# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28 #rad/s

#variáveis para leitura no arquivo "pontos" no csv
arquivo = open('pontos.csv')
linhas = csv.reader(arquivo)

with open('Leitura.csv', 'w',newline='') as csv_file:
     writer = csv.writer(csv_file)
     writer.writerow([1,2,3,4,5])
#    writer.writerows(psValues)




#variáveis iniciais do epuck
r = 20.5e-3 #raio da roda do epuck
l = 53e-3 #distância entre uma roda e outra do epuck
teta = 0 #angulo de direção do epuck sobre o plano
Teta = 0 #angulo de direção do epuck estimado pelo gyroscopio
x = 0
y = 0 


#variáveis iniciais da posição do epuck na arena

x1 = -42000
z1 = -42000

# create the Robot instance.
robot = Robot()

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
accelerometer = robot.getDevice('accelerometer')
accelerometer.enable(TIME_STEP)
gyro = robot.getDevice('gyro')
gyro.enable(TIME_STEP)
#gps = robot.getDevice('GPS')
#gps.enable(TIME_STEP)

# robot.batterySensorEnable(TIME_STEP)


# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]




for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
       

    # detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.85 * MAX_SPEED
    rightSpeed = 0.85 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0.85 * MAX_SPEED
        rightSpeed = -0.85 * MAX_SPEED
    elif right_obstacle:
       # turn left
        leftSpeed  = -0.85 * MAX_SPEED
        rightSpeed = 0.85 * MAX_SPEED

    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    #modelo cinemático do E-puck
    teta += - 1/l*(rightSpeed-leftSpeed)*r*TIME_STEP*1e-3
    x +=1/2*(rightSpeed+leftSpeed)*r*cos(teta)*TIME_STEP*1e-3
    y +=1/2*(rightSpeed+leftSpeed)*r*sin(teta)*TIME_STEP*1e-3
    Z = -x #nome em relacao ao mundo do webots
    X = y
    voltas = numpy.round(teta/(2*numpy.pi))
    teta = teta - voltas*2*numpy.pi
    
    
    aceleracao = accelerometer.getValues()
    giroscopio = gyro.getValues()
   # posicao = gps.getValues()
   # batteria = robot.batterySensorGetValue()
    Teta = Teta + TIME_STEP*1e-3*giroscopio[1]
    voltas = numpy.round(Teta/(2*numpy.pi))
    Teta = Teta - voltas*2*numpy.pi   
    
    print('Distancia: ', psValues)
    print('Aceleração: ', aceleracao)
    print('Rotação: ', giroscopio)
   # print('Posição: ', posicao)
    print('Posição direção X: ', Z)
    print('Posição direção Y: ', X)
    print('Ângulo teta: ', teta)
    print('Ângulo Teta giroscópio: ', Teta)
    
    #print(robot.transform)
  #  for linha in linhas:
   #     print(linha)
    
    

  #   for i in range(8):
  #  writer.writerows([psValues])

   
    
  #  print('Translacao: ', robot.translation)