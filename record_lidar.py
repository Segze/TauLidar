import lidar_to_file as lf
from TauLidarCommon.frame import FrameType, Frame
from TauLidarCamera.camera import Camera, FrameBuilder
import utils
import time
import os
import datetime

#'2023 02 12 21 44 49'
DEFAULT_DIR = str(datetime.datetime.now()).replace(":"," ").replace("-"," ").split('.')[0]

colorRange = (0,0)
frameBuilder = FrameBuilder()

#TODO:no inicializar las listas
def setup(serialPort:str=None,camera_range:list = [0,4500],mod_channel:int = 0,integration_time:list=[0,1000],minimal_amplitude:list=[0,80]) -> Camera:
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
        camera = Camera.open(port)             ## Open the first available Tau Camera
        camera.setModulationChannel(mod_channel)             ## autoChannelEnabled: 0, channel: 0
        camera.setIntegrationTime3d(integration_time[0], integration_time[1])       ## set integration time 0: 1000
        camera.setMinimalAmplitude(minimal_amplitude[0],minimal_amplitude[1])          ## set minimal amplitude 0: 80
        cameraInfo = camera.info()
        camera.setDefaultParameters()

        print("\nToF camera opened successfully:")
        print("    model:      %s" % cameraInfo.model)
        print("    firmware:   %s" % cameraInfo.firmware)
        print("    uid:        %s" % cameraInfo.uid)
        print("    resolution: %s" % cameraInfo.resolution)
        print("    port:       %s" % cameraInfo.port)

        print("\nPress Esc key over GUI or Ctrl-c in terminal to shutdown ...")
    return camera

def record(camera:Camera,time_seconds,directory:str,parent_dir:str = "."):
    #Nota: NO GUARDAR NADA MAS EN LA CARPETA
    i = 0
    d = ""
    metadata = {"time":0.0,"frame":0}
    try:
        complete_path = os.path.join(parent_dir,directory)
        os.mkdir(complete_path)
        d = directory
    except:
        utils.debug(f"Fallo el crear el directorio, utilizando otro nombre: {DEFAULT_DIR}")
        complete_path = os.path.join(parent_dir,DEFAULT_DIR)
        os.mkdir(complete_path)
        d = DEFAULT_DIR
    
    start = time.time()
    now = time.time()
    
    #Grabar por el tiempo requerido
    while ((now-start)<=time_seconds):
        rawDistanceArray = camera.readFrameRawData(FrameType.DISTANCE_AMPLITUDE)
        try:
            len_bytes = len(rawDistanceArray)
            #38400: 160x60 * 2 bytes * 2 (un dato de distancia seguido de otro de amplitud)
            if len_bytes < 38400:
                utils.debug(f"Frame erroneo, tamaño en bytes: {len_bytes}")
                pass
        except:
            pass
        #Actualizar los metadatos
        metadata["time"] = now-start
        metadata["frame"] = i
        #Guardar los datos en bruto
        filename = f"arr_{i}.npy"
        path = os.path.join(parent_dir,d,filename)
        lf.save_raw(rawDistanceArray,path,metadata)
        now = time.time()
        i+=1

    print(f"Grabados {now-start} segundos de video, ({i} frames), en la carpeta {os.path.join(parent_dir,d)}")
    print(f"Nota: NO GUARDAR NADA MAS EN LA CARPETA")

def countFrames(camera:Camera,directory:str,parent_dir:str)->int:
    #Contar el numero de frames
    fullpath = os.path.join(parent_dir,directory)
    files = os.listdir(fullpath)
    #Como cada frame tiene un archivo de metadatos, hay que contar los archivos y dividirlo entre 2
    total_frames = int(len(files)/2)
    return total_frames

def readFrame(camera:Camera,directory:str,parent_dir:str, index:int = 0,color_range = (0,4500)) -> tuple[Frame,dict]: 
    global colorRange, frameBuilder
    if (color_range[0] != colorRange[0] or color_range[1] != colorRange[1] ):
        colorRange = color_range
        #Actualizar, si no, no hay necesiadad
        frameBuilder.setRange(color_range[0],color_range[1])

    f = os.path.join(parent_dir,directory,f"arr_{index}.npy")
    rawData, metadata = lf.open_raw(file = f)
    
    return frameBuilder.composeFrame(rawData,FrameType.DISTANCE_AMPLITUDE), metadata

def composeFrameFast(data,frameType):
    #Crea un objeto frame haciendo los calculos necesarios, pero enviandolo a un calculador externo mas rapido
    pass

if __name__ == "__main__":
    port = "COM5"
    camera = setup(port,mod_channel=0,integration_time=[0,1000],minimal_amplitude=[0,10])
    record(camera,30,parent_dir=".\\grabaciones",directory=DEFAULT_DIR)

