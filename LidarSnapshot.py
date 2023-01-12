import numpy as np
from TauLidarCommon.frame import FrameType, Frame
from TauLidarCamera.camera import Camera
from TauLidarCommon.d3 import ImageColorizer
import math
import cv2

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
        camera.setIntegrationTime3d(0, 50)       ## set integration time 0: 1000
        camera.setMinimalAmplitude(0, 80)          ## set minimal amplitude 0: 80

        cameraInfo = camera.info()

        print("\nToF camera opened successfully:")
        print("    model:      %s" % cameraInfo.model)
        print("    firmware:   %s" % cameraInfo.firmware)
        print("    uid:        %s" % cameraInfo.uid)
        print("    resolution: %s" % cameraInfo.resolution)
        print("    port:       %s" % cameraInfo.port)
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


def nextFrame(camera):
    camera.setRange(0,4500)
    x = []
    y = []
    z = []
    frame = camera.readFrame(FrameType.DISTANCE_AMPLITUDE)
    while (not frame):
        frame = camera.readFrame(FrameType.DISTANCE_AMPLITUDE)
        print ("waiting frame")
    if frame:
        space = frame.points_3d
        x = []
        y = []
        z = []   
        rgb = []
        for i,arr in enumerate(space):
            x.append(arr[0])
            y.append(arr[1])
            z.append(arr[2])
            r = arr[3]/255
            g = arr[4]/255
            b = arr[5]/255
            rgb.append((r,g,b))
        #show Image
        depth = frame.data_depth_rgb
        height, width =  frame.height,  frame.width
        mat_depth_rgb = np.frombuffer(depth, dtype=np.uint16, count=-1, offset=0).reshape(height, width, 3)

        mat_depth_rgb = mat_depth_rgb.astype(np.uint8)

        upscale = 4

        img4 =  cv2.resize(mat_depth_rgb[:,:,:], (width*upscale, height*upscale))
        cv2.namedWindow('R Channel')
        cv2.moveWindow('R Channel', 20, 20)
        cv2.imshow("R Channel",img4)
        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
        return [x,y,z,rgb,frame.data_depth_rgb,frame.height,frame.width]
        

def run(camera):
    camera.setRange(0,4500)
    x = []
    y = []
    z = []
    while True:
        frame = camera.readFrame(FrameType.DISTANCE)
        if frame:
            space = frame.points_3d
            x = []
            y = []
            z = []   
            rgb = []
            minDistance=100000
            for i,arr in enumerate(space):
                if (i%2)!=0:
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
            print (f"Nearest point at {minDistance} m")

def cleanup(camera):
    print('\nShutting down ...')
    camera.close()
