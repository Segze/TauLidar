from TauLidarCommon.frame import FrameType, Frame
from TauLidarCamera.camera import Camera
from TauLidarCommon.d3 import ImageColorizer
import numpy as np
import laspy

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
        camera.setMinimalAmplitude(0, 10)          ## set minimal amplitude 0: 80

        cameraInfo = camera.info()

        print("\nToF camera opened successfully:")
        print("    model:      %s" % cameraInfo.model)
        print("    firmware:   %s" % cameraInfo.firmware)
        print("    uid:        %s" % cameraInfo.uid)
        print("    resolution: %s" % cameraInfo.resolution)
        print("    port:       %s" % cameraInfo.port)

        print("\nPress Esc key over GUI or Ctrl-c in terminal to shutdown ...")
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
    camera.setRange(0,4500)
    x = []
    y = []
    z = []
    for i in range (0,1):
        frame = camera.readFrame(FrameType.DISTANCE_AMPLITUDE)
        if frame:
            # #one dimensional array of float32
            # frame.data_depth
            # #one dimensional array of uint16
            # frame.data_amplitude
            # #three dimensional array fo uint8
            # frame.data_depth_rgb
            # #list of points in format [X,Y,Z,R,G,B]
            # #RGB: list of rgb values based on the bounds set by Camera.setRange(Z1,Z2)
            # frame.points_3d
            #header
            header = laspy.header.LasHeader(version="1.4", point_format=6)
            header.file_source_id = 0
            header.uuid = None
            #Nombre del hardware
            header.system_identifier = "Onion Tau"
            #Nombre del programa
            header.generating_software = "Tau Point Cloud To LAS"
            #Fecha de creacion
            #header.creation_date = date.today()
            #TODO: end this conversion

            FileSignature = "LASF"
            FileSourceId = 0


            #int32
            X = 0
            Y = 0
            Z = 0
            #uint16
            intensity = 0
            #3 bits, 3 bits, 1 bit, 1 bit (byte)
            #return number = 000 (number of return of the light pulse)
            #number of returns = 000 (total number of returns of the laser)
            #scan direction = 1 (the lidar was moving from left to right)
            #edge of flight (depends on the point, if it was at the end of the scan, the border of the image)
            info = 0
            #unsigned char
            classification = 0
            #signed char
            #angle at which ray was sent (including angle of the lidar itself), [-90,90], nadir being zero
            angle = 0
            #char
            user_data = 0
            #uint16
            point_data = 0


            mat_depth_rgb = np.frombuffer(frame.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame.height, frame.width, 3)
            mat_depth_rgb = mat_depth_rgb.astype(np.uint8)

            mat_amplitude = np.frombuffer(frame.data_amplitude, dtype=np.float32, count=-1, offset=0).reshape(frame.height, frame.width)
            mat_amplitude = mat_amplitude.astype(np.uint8)
            space = frame.points_3d
            x = []
            y = []
            z = []   
            rgb = []
            for i,arr in enumerate(space):
                if (i%20)!=0:
                    continue
                x.append(arr[0])
                y.append(arr[1])
                z.append(arr[2])
                r = arr[3]/255
                g = arr[4]/255
                b = arr[5]/255
                rgb.append((r,g,b))