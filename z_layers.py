import numpy as np
from numpy import arctan2, sqrt
import cv2
import matplotlib.pyplot as plt
from TauLidarCommon.frame import FrameType, Frame
from TauLidarCamera.camera import Camera
from TauLidarCommon.d3 import ImageColorizer
import math
import numexpr as ne
import lidar_to_file as lf
import utils
from enum import Enum

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
        camera.setIntegrationTime3d(0, 1000)       ## set integration time 0: 1000
        camera.setMinimalAmplitude(0, 40)          ## set minimal amplitude 0: 80
        cameraInfo = camera.info()
        camera.setDefaultParameters()

        print("\nToF camera opened successfully:")
        print("    model:      %s" % cameraInfo.model)
        print("    firmware:   %s" % cameraInfo.firmware)
        print("    uid:        %s" % cameraInfo.uid)
        print("    resolution: %s" % cameraInfo.resolution)
        print("    port:       %s" % cameraInfo.port)

        print("\nPress Esc key over GUI or Ctrl-c in terminal to shutdown ...")

        # cv2.namedWindow('R Channel')
        cv2.namedWindow('B Channel')
        cv2.namedWindow('G Channel')
        cv2.namedWindow('RGB Channel')
        cv2.namedWindow('Amplitude')
        cv2.namedWindow('Scale')


        cv2.moveWindow('B Channel', 20, 20)
        cv2.moveWindow('RGB Channel', 1000, 20)
        cv2.moveWindow('B Channel', 20, 360)
        cv2.moveWindow('G Channel', 1000, 360)
        cv2.moveWindow('Scale', 700, 50)
        cv2.moveWindow('Amplitude', 20, 700)


    return camera


def run(camera):
    
    Scale = np.array(utils.getDistanceMap(100)).astype(np.uint8)
    upscale = (2,750)
    S =  cv2.resize(Scale, (Scale.shape[0]*upscale[0], Scale.shape[1]*upscale[1]))
    cv2.imshow('Scale', S)
    # fig = plt.figure()
    #Scatter con todos los puntos
    #ax = fig.add_subplot(2,2,1, projection='3d')
    # ax.set_xlabel('X Label')
    # ax.set_ylabel('Y Label')
    # ax.set_zlabel('Z Label')
    # lay = fig.add_subplot(2,2,2)
    # lay3d = fig.add_subplot(2,2,3, projection='3d')
    # #para activar el modo interactivo del plot
    # plt.ion()
    

    # camera.setRange(0,4500)
    x = []
    y = []
    z = []

    # # scat = ax.scatter(x, y, z, marker='o')
    # scat_lay = lay.scatter(x, y, marker='o')
    # scat_lay3d = lay.scatter(x, y,z, marker='o')
    # plt.draw()
    # plt.show()
    distance_mask = 1.5
    layer_width = 0.2


    while True:
        frame = camera.readFrame(FrameType.DISTANCE_AMPLITUDE)
        raw = utils.distance_map(camera)
        if frame:
            space = frame.points_3d

            #height = get_height_layer([x,y,z],distance_mask,layer_width)


            mat_depth_rgb = np.frombuffer(frame.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame.height, frame.width, 3)
            mat_depth_rgb = mat_depth_rgb.astype(np.uint8)
            mat_amplitude = np.frombuffer(frame.data_amplitude, dtype=np.float32, count=-1, offset=0).reshape(frame.height, frame.width)
            mat_amplitude = mat_amplitude.astype(np.uint8)
            mat_depth = get_layer(raw,0,2)
            #Aplicar un filtrado 

            mat_depth = mat_depth.astype(np.uint8)

            mat_filtered = analyze_height_layer(mat_depth)
            mat_filtered = mat_filtered.astype(np.uint8)
            count, mat_contours = get_contours(mat_filtered)
            print(f"Se detectaron {count} objetos en la imagen")
            #print (mat_depth_rgb[:,:,0].shape)

            #Sin escalar, este mapa de profundidad sirve para dibujar cada voxel


            # Upscalling the image
            upscale = 4
            #img1 =  cv2.resize(mat_depth_rgb[:,:,0], (frame.width*upscale, frame.height*upscale))
            #img2 =  cv2.resize(mat_depth_rgb[:,:,1], (frame.width*upscale, frame.height*upscale))
            #img3 =  cv2.resize(mat_depth_rgb[:,:,2], (frame.width*upscale, frame.height*upscale))
            img4 =  cv2.resize(mat_depth_rgb[:,:,:], (frame.width*upscale, frame.height*upscale))
            amplitude_img =  cv2.resize(mat_amplitude, (frame.width*upscale, frame.height*upscale))
            img3 =  cv2.resize(mat_depth, (frame.width*upscale, frame.height*upscale))
            filtered =  cv2.resize(mat_filtered, (frame.width*upscale, frame.height*upscale))
            contours = cv2.resize(mat_contours, (frame.width*upscale, frame.height*upscale))

            cv2.imshow('R Channel', contours)
            cv2.imshow('G Channel', filtered)
            cv2.imshow('B Channel', img3)
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

def get_layer(data_raw:list, distanceA: float, distanceB: float):
    #Punto medio
    layer_width = distanceB-distanceA
    z = distanceB - layer_width/2
    return get_height_layer_low_level(data_raw,z_distance_from_source=z,layer_width=layer_width)

#Esta funcion devuelve todos los puntos a distancia z del lidar (tomando en cuenta el angulo de vision)
def get_height_layer(coordinates: list, z_distance_from_source: float, layer_width: float):
    z_height = z_distance_from_source
    rad2deg = 180/math.pi
    #Previene errores en los extremos
    if z_height<layer_width/2:
        z_height = layer_width/2
    #intervalo en el que se buscan puntos
    z_interval = (z_height-layer_width/2, z_height+layer_width/2)
    # [(X,Y,Z), (X1,Y1,Z1) ....]
    # [[x][y][y]]
    
    layer_points = [[0],[0],[5]]
    #los datos de coordenadas se deben proporcionar en la forma  [[Array de x][Array de y][Array de z]]
    #asi es mas facil leerlos
    
    for i, z in enumerate(coordinates[2]):
        #Hay que expresar cada punto en coordenadas polares y obtener la magnitud del vector
        x = coordinates[0][i]
        y = coordinates[1][i]
        xyz = np.array([x,y,z])
        
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
    
    return layer_points


def get_height_layer_low_level(data_raw: list, z_distance_from_source: float, layer_width: float):
    height = 60
    width = 160
    z_height = z_distance_from_source
    rad2deg = 180/math.pi
    #Previene errores en los extremos
    if z_height<layer_width/2:
        z_height = layer_width/2
    #intervalo en el que se buscan puntos
    z_interval = (z_height-layer_width/2, z_height+layer_width/2)
    #en este caso la entrada es un array 1D que se debe separar manualmente
    mat_depth = np.frombuffer(data_raw, dtype=np.float32, count=-1, offset=0).reshape(height, width, 1)
    layer_points = np.zeros((60,160),dtype = int)
    for j,y in enumerate(mat_depth):
        for i,x in enumerate(y):
            dist = mat_depth[j][i]
            if (dist> z_interval[0] and dist<z_interval[1]):
                layer_points[j][i] = 255
    return layer_points

class Filter(Enum):
    Median = 1
    Average = 2
    Gaussian = 3
    Bilateral = 4


#Esta funcion intenta encontrar los objetos presentes en la capa
def analyze_height_layer(img: list, filter:Filter = Filter.Median, params:tuple = (5)):
    #TODO:todo
    ret = None
    #Mas info : https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html
    match (filter):
        case Filter.Median:
            #Params : 5
            ret = cv2.medianBlur(img,params)
        case Filter.Average:
            #params : (5,5)
            ret = cv2.blur(img,params)
        case Filter.Gaussian:
            #params : (5,5)
            ret = cv2.GaussianBlur(img,params,0)
        case Filter.Bilateral:
            #Info: https://people.csail.mit.edu/sparis/bf_course/
            #Params (9,75,75)
            ret = cv2.bilateralFilter(img,params[0],params[1],params[2])

    
    return ret

#dibuja los contornos de la imagen, es decir, cuantos objectos (aproximadamente) se encuentran en ella
def get_contours(img: list):
    
    #Poner un borde a la imagen
    bordered = cv2.copyMakeBorder(img, 10, 10, 10, 10, cv2.BORDER_CONSTANT, None, value = 0)
    
    #Escalar la imagen 
    upscale = 4
    upscaled =  cv2.resize(bordered, (img.shape[1]*upscale, img.shape[0]*upscale))

    #edges
    edges =  cv2.Canny(upscaled, 100, 200)
    # threshold
    #Primer retorno es la matriz que representa el filtro y el segundo es la imagen filtrada
    #Este paso no es tan necesario, por que la imagen ya deberia estar en B/N
    thresh = cv2.threshold(edges, 0, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.bitwise_not(thresh,mask=None)
    
    # get contours
    contours = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    hierarchy = contours[1] if len(contours) == 2 else contours[2]
    contours = contours[0] if len(contours) == 2 else contours[1]
    
    # get the actual inner list of hierarchy descriptions
    try:
        hierarchy = hierarchy[0]
    except:
        pass

    # count inner contours
    count = 0
    result =  upscaled.copy()

    result = cv2.merge([result,result,result])
    try:
        for component in zip(contours, hierarchy):
            cntr = component[0]
            hier = component[1]
            # discard outermost no parent contours and keep innermost no child contours
            # hier = indices for next, previous, child, parent
            # no parent or no child indicated by negative values
            if (hier[3] > -1) & (hier[2] < 0):
                count = count + 1
                cv2.drawContours(result, [cntr], 0, (0,0,255), thickness=1)
    except:
        pass
    #result = cv2.resize(result, (result.shape[1], result.shape[0]))
    return count , result

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
    #azimuth = ceval('arctan2(y,x)')
    xyz = np.array([x,y,z])
    azimuth = np.arctan2(y,x)
    xy2 = x**2 + y**2
    #elevation = ceval('arctan2(z, sqrt(xy2))')
    elevation = np.arctan2(z,np.sqrt(xy2))
    r = np.sqrt(xyz.dot(xyz))
    # azimuth = moduloV2(azimuth,2*math.pi)
    # elevation = moduloV2(elevation,1*math.pi)
    return float(azimuth), float(elevation), float(r)

def cleanup(camera):
    print('\nShutting down ...')
    cv2.destroyAllWindows()
    camera.close()


def analyze_frame():
    
    fig = plt.figure()
    #Scatter con todos los puntos
    #ax = fig.add_subplot(2,2,1, projection='3d')
    # ax.set_xlabel('X Label')
    # ax.set_ylabel('Y Label')
    # ax.set_zlabel('Z Label')
    x = []
    y = []
    z = []

    lay = fig.add_subplot(2,2,2)
    lay3d = fig.add_subplot(2,2,3, projection='3d')
    # scat = ax.scatter(x, y, z, marker='o')
    scat_lay = lay.scatter(x, y, marker='o')
    scat_lay3d = lay.scatter(x, y,z, marker='o')

    space = lf.get_coords("snapshot_2023-01-11.csv")
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
    
    plt.show()
    # plt.pause(0.02)
    # # ax.cla()
    # lay.cla()
    # lay3d.cla()



if __name__ == "__main__":
    port = "COM5"

    camera = setup(port)

    if camera:
        try:
            run(camera)
        except Exception as e:
            print(e)

        cleanup(camera)
    #analyze_frame()