import array
import numpy as np
from TauLidarCommon.frame import FrameType, Frame
from TauLidarCamera.camera import Camera
from TauLidarCommon.d3 import ImageColorizer, _PointDistanceMasked
from time import time
  
#USO: @timer_func
def timer_func(func):
    # This function shows the execution time of 
    # the function object passed
    def wrap_func(*args, **kwargs):
        t1 = time()
        result = func(*args, **kwargs)
        t2 = time()
        print(f'Function {func.__name__!r} executed in {(t2-t1):.4f}s')
        return result
    return wrap_func
  
  





#Constantes:
#Esta mascara binaria remueve los bits de mayor valor (del orden de 2^15 mm), que parecen ser una incertidumbre razonable
MASK_OUT_CONFIDENCE        = 0x3FFF  
VALUE_LIMIT_VALID_PIXEL    = 16000
VALUE_LOW_AMPLITUDE        = 16001
VALUE_ADC_OVERFLOW         = 16002
VALUE_SATURATION           = 16003


def distance_map(camera):
    #El stream de datos binarios, que se van a procesar
    # Solo aplica para FrameType.DISTANCE
    rawDistanceArray = camera.readFrameRawData(FrameType.DISTANCE)
    #Por lo que se nota en el modulo TauLidarCommon.d3, los datos en bruto son numeros uint16 
    # que representan la distancia a la camara (Distancia absoluta esferica, no cartesiana) en milimetros
    data_depth_raw   = array.array('f', [])
    saturated_mask   = array.array('h', [])
    
    len_bytes = len(rawDistanceArray)
    #19200: 160x60 * 2 bytes
    if len_bytes < 19200:
        print("Bad frame ignored, bytes length: %d" % len_bytes)
        return []
    # class _PointDistanceMasked(Structure):
    #     _pack_ = 1
    #     _fields_ = [("distance", c_uint16)]
    #160x60 datos de tipo uint16 (no esta multiplicando, esta creando un array de 9600)
    _PointMasked9600 = 9600 * _PointDistanceMasked
    _points = _PointMasked9600.from_buffer(rawDistanceArray)
    
    for _p in _points:
        #Quitar los bits de incertidumbre
        distance = _p.distance  & MASK_OUT_CONFIDENCE
        dist = float("NaN")
        saturated_mask_v = 0
        if(distance < VALUE_LIMIT_VALID_PIXEL or distance == VALUE_ADC_OVERFLOW or distance == VALUE_SATURATION):
            if (distance == VALUE_ADC_OVERFLOW or distance == VALUE_SATURATION):
                saturated_mask_v = 255
            else:
                #Convertir a metros
                dist = 0.001*distance
        saturated_mask.append(saturated_mask_v)
        data_depth_raw.append(dist)
    return data_depth_raw
    #En este punto ya se tendran 2 arrays de float: el de la distancia al origen y el de los puntos de saturacion

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

def debug(msg):
    print(f"{msg}")