import numpy as np
from numpy import arctan2, sqrt
import cv2
import matplotlib.pyplot as plt
from TauLidarCommon.frame import FrameType, Frame
from TauLidarCamera.camera import Camera
from TauLidarCommon.d3 import ImageColorizer
import math
import numexpr as ne

def setup(serialPort=None):
    port = None
    camera = None
    # if no serial port is specified, scan for available Tau Camera devices
    if serialPort is None:
        ports = Camera.scan()                      ## Scan for available Tau Camera devices

        if len(ports) > 0:
            port = ports[0]
    else:
        port = serialPort

    if port is not None:
        Camera.setRange(0, 4500)                   ## points in the distance range to be colored

        camera = Camera.open(port)             ## Open the first available Tau Camera
        camera.setModulationChannel(0)             ## autoChannelEnabled: 0, channel: 0
        camera.setIntegrationTime3d(0, 5000)       ## set integration time 0: 1000
        camera.setMinimalAmplitude(0, 80)          ## set minimal amplitude 0: 80

        cameraInfo = camera.info()

        print("\nToF camera opened successfully:")
        print("    model:      %s" % cameraInfo.model)
        print("    firmware:   %s" % cameraInfo.firmware)
        print("    uid:        %s" % cameraInfo.uid)
        print("    resolution: %s" % cameraInfo.resolution)
        print("    port:       %s" % cameraInfo.port)

        print("\nPress Esc key over GUI or Ctrl-c in terminal to shutdown ...")

        # cv2.namedWindow('R Channel')
        # cv2.namedWindow('B Channel')
        # cv2.namedWindow('G Channel')
        cv2.namedWindow('RGB Channel')
        cv2.namedWindow('Amplitude')
        cv2.namedWindow('Scale')


        # cv2.moveWindow('R Channel', 20, 20)
        cv2.moveWindow('RGB Channel', 1000, 20)
        # cv2.moveWindow('B Channel', 20, 360)
        # cv2.moveWindow('G Channel', 1000, 360)
        cv2.moveWindow('Scale', 1000, 500)
        cv2.moveWindow('Amplitude', 1000, 100)


    return camera

def getDistanceMap(length):
    minCM = 0
    maxCM = 4500
    colorRange = []
    imageColorizer = ImageColorizer()
    imageColorizer.setRange(minCM,maxCM)
    dl = (maxCM-minCM) / length
    max = imageColorizer.getColor(maxCM)
    for i in range (0,length):
        color = imageColorizer.getColor(i*dl)
        R,G,B = color.r, color.g, color.b
        colorRange.append([[R,B,G]])
    
    return colorRange



def run(camera):
    
    Scale = np.array(getDistanceMap(100)).astype(np.uint8)
    upscale = (2,750)
    S =  cv2.resize(Scale, (Scale.shape[0]*upscale[0], Scale.shape[1]*upscale[1]))
    cv2.imshow('Scale', S)
    fig = plt.figure()
    #Scatter con todos los puntos
    #ax = fig.add_subplot(2,2,1, projection='3d')
    # ax.set_xlabel('X Label')
    # ax.set_ylabel('Y Label')
    # ax.set_zlabel('Z Label')
    lay = fig.add_subplot(2,2,2)
    lay3d = fig.add_subplot(2,2,3, projection='3d')
    #para activar el modo interactivo del plot
    plt.ion()
    

    camera.setRange(0,4500)
    x = []
    y = []
    z = []

    # scat = ax.scatter(x, y, z, marker='o')
    scat_lay = lay.scatter(x, y, marker='o')
    scat_lay3d = lay.scatter(x, y,z, marker='o')
    plt.draw()
    plt.show()


    while True:
        frame = camera.readFrame(FrameType.DISTANCE_AMPLITUDE)
        
        if frame:
            space = frame.points_3d
            x = []
            y = []
            z = []   
            rgb = []
            xyz = []
            minDistance=100000
            for i,arr in enumerate(space):
                if (i%1)!=0:
                    continue
                x.append(arr[0])
                y.append(arr[1])
                z.append(arr[2])
                r = arr[3]/255
                g = arr[4]/255
                b = arr[5]/255
                distance = math.dist([0,0,0],[arr[0],arr[1],arr[2]])
                if distance<minDistance:
                    minDistance = distance
                rgb.append((r,g,b))
                xyz.append((arr[0],arr[1],arr[2]))
            print (f"Nearest point at {minDistance} m")
            #scat.set_offsets(np.c_[x,y])
            # scat = ax.scatter(x, y, z,c=rgb)
            layer = get_z_layer([x,y,z],1.7,0.1)
            scat_lay = lay.scatter(layer[0], layer[1])
            
            height = get_height_layer([x,y,z],1.7,0.1)
            #analyze_height_layer(height)
            scat_lay3d = lay3d.scatter(height[0], height[1],height[2], marker='o')
            #scat.set(x,y,z)
            
            plt.draw()
            plt.pause(0.02)
            # ax.cla()
            lay.cla()
            lay3d.cla()
            
            #plt.pause(0.1) 
            mat_depth_rgb = np.frombuffer(frame.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame.height, frame.width, 3)

            mat_depth_rgb = mat_depth_rgb.astype(np.uint8)
            mat_amplitude = np.frombuffer(frame.data_amplitude, dtype=np.float32, count=-1, offset=0).reshape(frame.height, frame.width)
            mat_amplitude = mat_amplitude.astype(np.uint8)
            #print (mat_depth_rgb[:,:,0].shape)

            #Sin escalar, este mapa de profundidad sirve para dibujar cada voxel


            # Upscalling the image
            upscale = 4
            #img1 =  cv2.resize(mat_depth_rgb[:,:,0], (frame.width*upscale, frame.height*upscale))
            #img2 =  cv2.resize(mat_depth_rgb[:,:,1], (frame.width*upscale, frame.height*upscale))
            #img3 =  cv2.resize(mat_depth_rgb[:,:,2], (frame.width*upscale, frame.height*upscale))
            img4 =  cv2.resize(mat_depth_rgb[:,:,:], (frame.width*upscale, frame.height*upscale))
            amplitude_img =  cv2.resize(mat_amplitude, (frame.width*upscale, frame.height*upscale))

            #cv2.imshow('R Channel', img1)
            #cv2.imshow('G Channel', img2)
            #cv2.imshow('B Channel', img3)
            cv2.imshow('RGB Channel', img4)
            cv2.imshow('Amplitude', amplitude_img)

            if cv2.waitKey(1) == 27: break

#Esta funcion devuelve todos los puntos en la coordenada z indicada (coordenada espacial)
def get_z_layer(coordinates: list, z_height: float, layer_width: float):
    #Previene errores en los extremos
    if z_height<layer_width/2:
        z_height = layer_width/2
    #intervalo en el que se buscan puntos
    z_interval = (z_height-layer_width/2, z_height+layer_width/2)
    # [(X,Y,Z), (X1,Y1,Z1) ....]
    # [[x][y][y]]
    layer_points = [[0],[0],[0]]
    #los datos de coordenadas se deben proporcionar en la forma  [[Array de x][Array de y][Array de z]]
    #asi es mas facil leerlos
    for i, z in enumerate(coordinates[2]):
        if z > z_interval[0] and z< z_interval[1]:
            layer_points[0].append(coordinates[0][i])
            layer_points[1].append(coordinates[1][i])
            #Notese que el valor de z no es el real, sino que esta simplificado a solo la capa
            #De hecho, este metodo deberia devolver una funcion 2d
            layer_points[2].append(z_height)
            #layer_points.append((coordinates[0][i],coordinates[1][i],z))
    return layer_points

#Esta funcion devuelve todos los puntos a distancia z del lidar (tomando en cuenta el angulo de vision)
def get_height_layer(coordinates: list, z_distance_from_source: float, layer_width: float):
    z_height = z_distance_from_source

    #Previene errores en los extremos
    if z_height<layer_width/2:
        z_height = layer_width/2
    #intervalo en el que se buscan puntos
    z_interval = (z_height-layer_width/2, z_height+layer_width/2)
    # [(X,Y,Z), (X1,Y1,Z1) ....]
    # [[x][y][y]]
    layer_points = [[0],[0],[10]]
    #los datos de coordenadas se deben proporcionar en la forma  [[Array de x][Array de y][Array de z]]
    #asi es mas facil leerlos
    
    for i, z in enumerate(coordinates[2]):
        #Hay que expresar cada punto en coordenadas polares y obtener la magnitud del vector
        x = coordinates[0][i]
        y = coordinates[1][i]
        xyz = np.array([x,y,z])
        rad2deg = 180/math.pi
        #Cualquiera de estas dos opciones sirve
        #magnitude = np.linalg.norm(xyz)
        magnitude = np.sqrt(xyz.dot(xyz))
        if magnitude > z_interval[0] and magnitude< z_interval[1]:
            # A diferencia de analyze_z_layer, esta funcion si devuelve un espacio 3d
            # Por que depende de la distancia, no solo la coordenada z
            # Es decir, devuelve puntos que son parte de la seccion de una esfera del radio especificado
            # De todas formas, se puede hacer una proyeccion aproximada, pero, como la proyeccion del globo terraqueo, pierde precision
            layer_points[0].append(x)
            layer_points[1].append(y)
            layer_points[2].append(z)
            azimuth, elevation, r = cart2sph(x,y,z)
            azimuth *= rad2deg
            elevation *= rad2deg
            #if (azimuth >)
            print(f"azimuth: {azimuth}, elevation: {elevation}")

            #layer_points.append((coordinates[0][i],coordinates[1][i],z))
    
    return layer_points

#Esta funcion intenta encontrar los objetos presentes en la capa
def analyze_height_layer(coordinates: list):
    spherical_coordinates = []
    pixels_x = []
    pixels_y = []
    pixel_coords = []
    deg2Rad = 2*math.pi/360
    FOV = [deg2Rad* 81,deg2Rad* 31]
    ratio_x, ratio_y = 160/FOV[0], 30/FOV[1]

    for i,x in enumerate (coordinates[0]):
        y = coordinates[1][i]
        z = coordinates[2][i]
        azimuth, elevation, r = cart2sph(x,y,z)
        spherical_coordinates.append((azimuth,elevation,r))
        # La idea es convertir esta lista de coordenadas en una imagen 2d que corresponda a la imagen del sensor
        #El FOV de la camara es 81°x31°, y la resolucion de 160x60 px
        #Asumiendo una relacion lineak se puede formular la sig ecuacion 

        # azimuth = (81/160)*pixel_x
        # elevation = 90° - (31/30)*pixel_y
        # Despejando se obtienen los pixeles x,y con centro en el centro de la imagen

        #Asumiendo que el pixel 0,0 esta en el centro de la imagen 
        px = moduloV2(azimuth/FOV[0],FOV[0])
        py = moduloV2((math.pi/2 - elevation)/FOV[1],FOV[1])
        pixel_x = px * 160
        pixel_y = py * 30
        #Falta desplazarlos para que coincidan con el estandar (0,0 en la esquina superior izquierda)
        pixel_x += 80
        pixel_y += 30
        # pixels_x.append(int(pixel_x))
        # pixels_y.append(int(pixel_y))
        pixel_coords.append((int(pixel_x),int(pixel_y)))
    #np.frombuffer(, dtype=np.uint16, count=-1, offset=0).reshape(frame.height, frame.width, 3)
    blank_image = np.zeros((60,160,3), np.uint8)
    for x,y in pixel_coords:
        try:
            blank_image[y][x] = 255
        except:
            print("Error")



#Operacion modular, (modulo aritmetico, un poco mas riguroso que el operador %)
def moduloV2(a,b):
    amodb = a%b
    if amodb>0:
        return amodb
    else:
        return a-amodb



def cart2sph(x,y,z, ceval=ne.evaluate):
    """ x, y, z :  ndarray coordinates
        ceval: backend to use: 
              - eval :  pure Numpy
              - numexpr.evaluate:  Numexpr """
    azimuth = ceval('arctan2(y,x)')
    xy2 = ceval('x**2 + y**2')
    elevation = ceval('arctan2(z, sqrt(xy2))')
    r = eval('sqrt(xy2 + z**2)')
    azimuth = moduloV2(azimuth,2*math.pi)
    elevation = moduloV2(elevation,1*math.pi)
    return float(azimuth), float(elevation), float(r)

def cleanup(camera):
    print('\nShutting down ...')
    cv2.destroyAllWindows()
    camera.close()


if __name__ == "__main__":
    port = "COM5"

    camera = setup(port)

    if camera:
        try:
            run(camera)
        except Exception as e:
            print(e)

        cleanup(camera)