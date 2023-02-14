import numpy as np
import csv
import os
from TauLidarCamera.camera import Camera
from TauLidarCommon.frame import Frame
import laspy
import datetime
import gps_time
# save numpy array as npy file
import numpy


def save_coords(coordinates: list, file: str):
    #coordinates must be in type [(x1,y1,z1,r1,g1,2),(x2,y2,z2,r2,g2,b2)]
    #RGB is optional
    if (not os.path.exists(f"./{file}.csv")):
        with open(f"{file}.csv", 'w', newline='\n') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(['X','Y','Z','R','G','B'])
    with open(f"{file}.csv", 'a', newline='\n') as csvfile:
        writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for coords in coordinates:
            x = coords[0]
            y = coords[1]
            z = coords[2]
            r = 0
            g = 0
            b = 0
            try:
                r = coords[3]
                g = coords[4] 
                b = coords[5] 
            except:
                pass
            writer.writerow([x,y,z,r,g,b])
                
def get_coords(file: str):
    coords = []
    if (not os.path.exists(f"./{file}")):
        return []
    with open(f"{file}","r") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for i,row in enumerate(csv_reader):
            #OMITIR HEADERS
            if (i==0): continue
        coords.append(
            (float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]))
            )
    return coords


#Guarda los datos del lidar en bruto en un archivo binario
def save_raw(data: list,file: str, metadata:dict = {}):
    if (not os.path.exists(f"{file}")):
        numpydata = numpy.asarray(data)
        numpy.save(file,numpydata)
    if (metadata):
        numpydata=numpy.asarray([metadata])
        numpy.save(f"{file}.meta",numpydata,allow_pickle=True)
#Lee los datos binarios
def open_raw(file: str):
    raw = bytearray(list(numpy.load(file=file,allow_pickle=False)))
    meta = numpy.load(file=f"{file}.meta.npy",allow_pickle=True)
    metadata = dict(list(meta)[0])
    return raw, metadata

def saveToLaz(frame: Frame,camera: Camera, filename: str,integrationTime3d = 0, integrationTimeGrayscale = 0, minimalAmplitude = 0):
    info = camera.info()
    space = frame.points_3d
    # #Fecha de creacion
    # #header.creation_date = date.today()
    # FileSignature = "LASF"
    # FileSourceId = 0
    # #int32
    # X = 0
    # Y = 0
    # Z = 0
    # #uint16
    # intensity = 0
    # #3 bits, 3 bits, 1 bit, 1 bit (byte)
    # #return number = 000 (number of return of the light pulse)
    # #number of returns = 000 (total number of returns of the laser)
    # #scan direction = 1 (the lidar was moving from left to right)
    # #edge of flight (depends on the point, if it was at the end of the scan, the border of the image)
    # info = 0
    # #unsigned char
    # classification = 0
    # #signed char
    # #angle at which ray was sent (including angle of the lidar itself), [-90,90], nadir being zero
    # angle = 0
    # #char
    # user_data = 0
    # #uint16
    # point_data = 0



    #header
    header = laspy.header.LasHeader(version="1.4", point_format=6)
    header.file_source_id = 0
    #header.uuid = 0
    #Nombre del hardware
    header.system_identifier = "Onion Tau"
    #Nombre del programa
    header.generating_software = "Tau Point Cloud To LAS"
    #Los valores del lidar se devuelven en milimetros, 0.001 m
    header.scales = np.array([0.001, 0.001, 0.001])
    #Los desplazamientos del lidar, (el origen de coordenadas)
    header.offsets = np.array([0,0,0])
    # La demas informacion deberia generarse automaticamente o son innecesarios, 
    #Especificacion del formato : 
    # http://www.asprs.org/wp-content/uploads/2019/07/LAS_1_4_r15.pdf
    # Info util: 
    # https://laspy.readthedocs.io/en/latest/intro.html    
    # #Variable Length Records (VLR), en estos se podra incluir informacion extra, en este particular caso
    # Utilizare los VLR para colocar las caracteristicas de la camara al momento de la captura
    #Maximo: 65536 bytes
    camera_data = f"Modelo:{info.model}.Firmware:{info.firmware}.UID:{info.uid}.Resolucion:{info.resolution}."
    camera_data += f"Tiempo de Integracion 3D:{integrationTime3d}.Tiempo de Integracion Escala de Grises:{integrationTimeGrayscale}."
    camera_data += f"Amplitud Minima:{minimalAmplitude}."
    bytes_camera_data = bytearray(camera_data,"utf8")
    new_vlr = laspy.VLR("Creator",1,
        description = "Configuracion del lidar",
        record_data=bytes_camera_data)
    #Construir el archivo
    LAZ = laspy.LasData(header=header)
    LAZ.vlrs.append(new_vlr)
    #iterar los puntos del lidar
    #TODO: utilizar los datos en bruto
    count = len(space)
    point_record = laspy.ScaleAwarePointRecord.zeros(point_count=count,header=LAZ.header)
    with laspy.open(f"{filename}.laz",mode = "w",header=LAZ.header) as writer:    
        for point in space:
            x,y,z = point[0],point[1],point[2]
            point_format = LAZ.point_format

            point_record.x = [int(x*1000)]
            point_record.y = [int(y*1000)]
            point_record.z = [int(z*1000)]
            #TODO: con los datos en bruto retornar la intensidad de la luz
            point_record.intensity =[0]
            point_record.return_number =[0]
            point_record.number_of_returns = [0]
            point_record.synthetic = [False]
            point_record.key_point = [False]
            point_record.withheld = [False]
            point_record.overlap = [False]
            point_record.scanner_channel = [0]
            point_record.scan_direction_flag = [True]
            #The Edge of Flight Line Flag has a value of 1 only when the point is at the end of a scan. It is the
            #last point on a given scan line before it changes direction or the mirror facet changes
            #TODO: set this parameter correctly
            point_record.edge_of_flight_line = [False]
            point_record.classification = [0]
            #Angulo con respecto a la vertical
            #TODO: agregar este campo
            #point_record.scan_angle
            #Informacion libre para cada punto
            point_record.user_data=[0]
            #No puede ser 0, indica el tipo de fuente del que se origino
            point_record.point_source_id = [1]
            point_record.gps_time = [gps_time.GPSTime.from_datetime(datetime.datetime.now()).time_of_week]
            writer.write_points(point_record)

