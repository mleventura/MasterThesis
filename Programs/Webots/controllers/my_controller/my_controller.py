from controller import Robot, DistanceSensor, Motor,  GPS, Compass
from math import cos, sin, atan2
#from sympy import * #para variaveis simbolicas 
from scipy.spatial.transform import Rotation as Rot
import csv
import numpy as np




#qual a matriz que da a rotação a partir de dois vetores 
def rot_from_vectors(vector_orig, vector_fin):
   
    from numpy import cross, dot
    from math import sin, cos, acos
    from numpy.linalg import norm
    import numpy as np
    
    # Convert the vectors to unit vectors.
    vector_orig = vector_orig / norm(vector_orig)
    vector_fin = vector_fin / norm(vector_fin)

    # The rotation axis (normalised).
    axis = cross(vector_orig, vector_fin)
    axis_len = norm(axis)
    if axis_len != 0.0:
        axis = axis / axis_len

    # Alias the axis coordinates.
    x = axis[0]
    y = axis[1]
    z = axis[2]

    # The rotation angle. Dot -> faz o produto escalar entre vetores
    angle = acos(dot(vector_orig, vector_fin))

    # Trig functions (only need to do this maths once!).
    ca = cos(angle)
    sa = sin(angle)

   
    # Calculate the rotation matrix elements.
    R = np.zeros([3,3])
    R[0,0] = 1.0 + (1.0 - ca)*(x**2 - 1.0)
    R[0,1] = -z*sa + (1.0 - ca)*x*y
    R[0,2] = y*sa + (1.0 - ca)*x*z
    R[1,0] = z*sa+(1.0 - ca)*x*y
    R[1,1] = 1.0 + (1.0 - ca)*(y**2 - 1.0)
    R[1,2] = -x*sa+(1.0 - ca)*y*z
    R[2,0] = -y*sa+(1.0 - ca)*x*z
    R[2,1] = x*sa+(1.0 - ca)*y*z
    R[2,2] = 1.0 + (1.0 - ca)*(z**2 - 1.0)


    return R




def run_robot(robot):
    
      
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
    vel_global = np.zeros(3)
    posicao_global =np.zeros(3)


    TIME_STEP = 64
    MAX_SPEED = 6.28 #rad/s
  

    #variáveis iniciais do robo
    r = 1110e-3 #raio da roda do robô
    R = 3400e-3 #raio de giro da roda do robô
    l = 3810e-3 #distância entre uma roda e outra do robô mais proximo
    L = 4970e-3 #distância entre uma roda e outra do robô mais proximo


    teta = 0 #ângulo de direção do robo sobre o plano
    Teta = 0 #ângulo de direção do robo estimado pelo gyroscopio
    x = 0
    y = 0
    z = 0
    t=0
  
  
    camera = robot.getDevice('camera')
    camera.enable(TIME_STEP)
    accelerometer = robot.getDevice('accelerometer')
    accelerometer.enable(TIME_STEP)
    gyro = robot.getDevice('gyro')
    gyro.enable(TIME_STEP)
    gps = robot.getDevice('gps')
    gps.enable(TIME_STEP)
    compass = robot.getDevice('compass')
    compass.enable(TIME_STEP)

    robot.batterySensorEnable(TIME_STEP)
        
    

#Create instance of motor

    frontLeftWheel = robot.getDevice('motor_3')
    frontRightWheel = robot.getDevice('motor_2')
    backLeftWheel = robot.getDevice('motor_4')
    backRightWheel = robot.getDevice('motor_1')

    frontLeftWheel.setPosition(float('inf'))
    frontRightWheel.setPosition(float('inf'))
    backLeftWheel.setPosition(float('inf'))
    backRightWheel.setPosition(float('inf'))

    frontLeftWheel.setVelocity(0)
    frontRightWheel.setVelocity(0)
    backLeftWheel.setVelocity(0)
    backRightWheel.setVelocity(0)

   

 
    
    #matriz de aceleracao e do giroscopio
    M_acel_global = np.empty((0,3)) 
    M_giro_global = np.empty((0,3)) 
    
    while robot.step(TIME_STEP) != -1:
        
        # sinal de erro de posição
        dx = xref-x
        dy = yref-y

        #velocidade média entre as 2 rodas 
        erro= np.sqrt(dx**2 + dy**2)
        m = kc*erro   #quanto aplica de velocidade média   

    
        #angulo desejado da trajetoria -> na direção que deveria ir
        alfa= atan2(dy,dx) 

       # erro angular = alfa - teta     
        deltav = ((alfa-teta))*kt

        if j>= len(vxref):
            break
            csv_file.close()
        
        if erro < 0.03: #quando esta proximo da posição desejada, trocar para o proximo ponto da trajetoria
            leftSpeed = 0
            rightSpeed= 0
            deltav = 0
            xref= vxref[j]
            yref= vyref[j]
            j=j+1
        
        leftSpeed = m - deltav
        rightSpeed = m + deltav
        print(leftSpeed)        
        frontLeftWheel.setVelocity(MAX_SPEED)
        frontRightWheel.setVelocity(-MAX_SPEED)
        backLeftWheel.setVelocity(0.2*MAX_SPEED)
        backRightWheel.setVelocity(-0.2*MAX_SPEED)
        
        
        t = t+TIME_STEP*1e-3
        
        aceleracao = accelerometer.getValues()
        aceleracao = np.array(aceleracao).reshape([3,1]) # transformou lista de números em vetor de números e o reshape fez 3 linhas e 1 coluna
        
        giroscopio = gyro.getValues()
        giroscopio = np.array(giroscopio).reshape([3,1])
        
        gps_robo = gps.getValues()
        
        if t<= TIME_STEP*1e-3:
            compass_0 = compass.getValues()
            compass_0 = np.array(compass_0)
        
        
        compass_robo = compass.getValues()
        compass_robo = np.array(compass_robo)
       
        rot = rot_from_vectors(compass_0,compass_robo)
        
        Aceleracao_global = np.squeeze(rot@aceleracao)
        vel_global += Aceleracao_global*TIME_STEP*1e-3
        posicao_global += vel_global*TIME_STEP*1e-3
        x = posicao_global[0]
        y = posicao_global[2]
    
                      
        Giroscopio_global = np.squeeze(rot@giroscopio)


        M_acel_global = np.vstack((M_acel_global,Aceleracao_global))
        M_giro_global = np.vstack((M_giro_global,Giroscopio_global))
        
        bateria = robot.batterySensorGetValue() 
        
        
        #print('Bateria: ', bateria)
        #print('Aceleração: ', Aceleracao_global) #@ significa que é uma matriz
      #  print ('Matriz Aceleração', M_acel_global)
      #  print('Rotação: ', Giroscopio_global)
      #  print('GPS: ', gps_robo)
       # print('Magnetometro: ', compass_robo)
               
        #print('Posição: ', posicao)
        #print('Posição direção X: ', x)
        #print('Posição direção Y: ', y)
        #print('Ângulo teta: ', teta)
        #print('Ângulo Teta giroscópio: ', Teta)
      #  print('Tempo: ', t)    
    
      #  with open('Leitura.csv', 'a+',newline='') as csv_file:
       #      writer = csv.writer(csv_file)
          #    writer.writerow([x])
        #     writer.writerow([t,gps_robo[0],gps_robo[1],gps_robo[2],Aceleracao_global[0],Aceleracao_global[1],Aceleracao_global[2],Giroscopio_global[0],Giroscopio_global[1],Giroscopio_global[2]] )
       #with open('Leitura.csv', 'a+',newline='') as csv_file:
       #      writer = csv.writer(csv_file)
      #       writer.writerow([t,x,y,M_acel_global] )  
       #      writer.writerow([t,x,y,z,Aceleracao_global[0],Aceleracao_global[1],Aceleracao_global[2] ])   

        
                 
         
        
if __name__ == "__main__":

        my_robot = Robot()
        run_robot(my_robot)      
        