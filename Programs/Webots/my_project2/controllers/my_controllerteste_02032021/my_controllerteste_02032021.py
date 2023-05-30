from controller  import Robot, DistanceSensor, Motor, Camera    #, GPS
from math import cos, sin, atan2
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

#variaveis de controle de velocidade média e controle do angulo
kc = 8  #constante da posicao
kt = 0.8 #constante do erro do angulo
vxref = [ 0, 0.5, 1, 1, 1.5]  #vetor da trajetoria em x
vyref = [0.5, 0.5, 1, 1.5, 1]  #vetor da trajetoria em y
j = 1 #indice da trajetoria
xref = vxref[0] 
yref = vyref[0]
x = 0 #posicao inicial
y = 0 #posicao inicial


# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
       

    # detect obstacles
    #right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    #left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    #leftSpeed  = 0.85 * MAX_SPEED
    #rightSpeed = 0.85 * MAX_SPEED
    # modify speeds according to obstacles
    #if left_obstacle:
        # turn right
    #    leftSpeed  = 0.85 * MAX_SPEED
    #    rightSpeed = -0.85 * MAX_SPEED
    #elif right_obstacle:
    #   # turn left
    #    leftSpeed  = -0.85 * MAX_SPEED
    #    rightSpeed = 0.85 * MAX_SPEED

    # sinal de erro de posição
    dx = xref-x
    dy = yref-y

    #velocidade média entre as 2 rodas 
    erro= numpy.sqrt(dx**2 + dy**2)
    m = kc*erro   #quanto aplica de velocidade média   
    
    
    #angulo desejado da trajetoria -> na direção que deveria ir
    alfa= atan2(dy,dx) 

   # erro angular = alfa - teta     
    deltav = ((alfa-teta))*kt

    if erro < 0.03: #quando esta proximo da posição desejada, trocar para o proximo ponto da trajetoria
        leftSpeed = 0
        rightSpeed= 0
        deltav = 0
        xref= vxref[j]
        yref= vyref[j]
        j=j+1
   
   #dividir a velocidade nas duas rodas
    leftSpeed = m - deltav
    rightSpeed = m + deltav
   
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    #modelo cinemático do E-puck para estimar o teta e a posicao
    teta += + 1/l*(rightSpeed-leftSpeed)*r*TIME_STEP*1e-3
    x +=1/2*(rightSpeed+leftSpeed)*r*cos(teta)*TIME_STEP*1e-3
    y +=1/2*(rightSpeed+leftSpeed)*r*sin(teta)*TIME_STEP*1e-3
   
    voltas = numpy.round(teta/(2*numpy.pi))
    teta = teta - voltas*2*numpy.pi
    





   
    aceleracao = accelerometer.getValues()
    giroscopio = gyro.getValues()
    # batteria = robot.batterySensorGetValue()
    Teta = Teta + TIME_STEP*1e-3*giroscopio[1]
    voltas = numpy.round(Teta/(2*numpy.pi))
    Teta = Teta - voltas*2*numpy.pi
   # print('Distancia: ', psValues)
   # print('Aceleração: ', aceleracao)
   # print('Rotação: ', giroscopio)
   # print('Posição: ', posicao)
    print('Posição direção X: ', x)
    print('Posição direção Y: ', y)
    print('Ângulo teta: ', teta)
   # print('Ângulo Teta giroscópio: ', Teta)
    #print(robot.transform)
    #for linha in linhas:
    #    print(linha)
    print('Posição direção dx: ', dx)
    print('Posição direção dy: ', dy)
    print('Velocidade média: ', m)
    print('delta V: ', deltav)
    print('erro angular alfa: ', alfa)
  #   for i in range(8):
  #  writer.writerows([psValues])

   
   
  #  print('Translacao: ', robot.translation)