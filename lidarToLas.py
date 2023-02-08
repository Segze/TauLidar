from TauLidarCommon.frame import FrameType, Frame
from TauLidarCamera.camera import Camera
from TauLidarCommon.d3 import ImageColorizer
import numpy as np
import lidar_to_file as lf

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
    info  = camera.info()
    integrationTime3d = 1000
    integrationTimeGrayscale = 0
    minimalAmplitude = 10

    x = []
    y = []
    z = []
    for i in range (0,1):
        frame = camera.readFrame(FrameType.DISTANCE_AMPLITUDE)
        if frame:
            space = frame.points_3d

            lf.saveToLaz(frame,camera,"test_file",integrationTime3d,integrationTimeGrayscale,minimalAmplitude)
            mat_depth_rgb = np.frombuffer(frame.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame.height, frame.width, 3)
            mat_depth_rgb = mat_depth_rgb.astype(np.uint8)

            mat_amplitude = np.frombuffer(frame.data_amplitude, dtype=np.float32, count=-1, offset=0).reshape(frame.height, frame.width)
            mat_amplitude = mat_amplitude.astype(np.uint8)
            
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

if __name__ == "__main__":
    port = "COM5"

    camera = setup(port)

    if camera:
        try:
            run(camera)
        except Exception as e:
            print(e)