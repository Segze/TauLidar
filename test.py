import numpy as np
import cv2
import matplotlib.pyplot as plt
from TauLidarCommon.frame import FrameType, Frame
from TauLidarCamera.camera import Camera
from TauLidarCommon.d3 import ImageColorizer, _PointDistanceMasked
import math
import array

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
        camera.setIntegrationTime3d(0, 2000)       ## set integration time 0: 1000
        camera.setMinimalAmplitude(0, 80)          ## set minimal amplitude 0: 80

        cameraInfo = camera.info()

        print("\nToF camera opened successfully:")
        print("    model:      %s" % cameraInfo.model)
        print("    firmware:   %s" % cameraInfo.firmware)
        print("    uid:        %s" % cameraInfo.uid)
        print("    resolution: %s" % cameraInfo.resolution)
        print("    port:       %s" % cameraInfo.port)

        print("\nPress Esc key over GUI or Ctrl-c in terminal to shutdown ...")

        cv2.namedWindow('R Channel')
        cv2.namedWindow('B Channel')
        cv2.namedWindow('G Channel')
        cv2.namedWindow('RGB Channel')
        cv2.namedWindow('Amplitude')
        cv2.namedWindow('Scale')


        cv2.moveWindow('R Channel', 20, 20)
        cv2.moveWindow('RGB Channel', 1000, 20)
        cv2.moveWindow('B Channel', 20, 360)
        cv2.moveWindow('G Channel', 1000, 360)
        cv2.moveWindow('Scale', 500, 500)


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
    ax = fig.add_subplot(projection='3d')
    #para activar el modo interactivo del plot
    plt.ion()
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    camera.setRange(0,4500)
    x = []
    y = []
    z = []

    scat = ax.scatter(x, y, z, marker='o')
    plt.draw()
    mat_depth = None
    img4 = None

        #define the events for the
    # mouse_click.
    def mouse_click(event, x, y,flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:

            cv2.putText(img4,f"{mat_depth[int(y/4)][int(x/4)]}",(x,y),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,0.5,(255,255,255))
            cv2.imshow('R Channel',img4)
    cv2.setMouseCallback('RGB Channel', mouse_click)
    while True:
        frame = camera.readFrame(FrameType.DISTANCE_AMPLITUDE)
        
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
            #scat.set_offsets(np.c_[x,y])
            scat = ax.scatter(x, y, z,c=rgb)
            #scat.set(x,y,z)
            
            plt.draw()
            plt.pause(0.02)
            ax.cla()
            
            #plt.pause(0.1)  

            mat_depth = np.frombuffer(frame.data_depth, dtype=np.float32, count=-1, offset=0).reshape(frame.height, frame.width, 1)
            #mat_depth = mat_depth.astype(np.uint8)
            mat_depth_rgb = np.frombuffer(frame.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame.height, frame.width, 3)
            
            mat_depth_rgb = mat_depth_rgb.astype(np.uint8)
            mat_amplitude = np.frombuffer(frame.data_amplitude, dtype=np.float32, count=-1, offset=0).reshape(frame.height, frame.width)
            mat_amplitude = mat_amplitude.astype(np.uint8)



            #print (mat_depth_rgb[:,:,0].shape)

            #Sin escalar, este mapa de profundidad sirve para dibujar cada voxel


            # Upscalling the image
            upscale = 4
            img1 =  cv2.resize(mat_depth, (frame.width*upscale, frame.height*upscale))
            img2 =  cv2.resize(mat_depth_rgb[:,:,1], (frame.width*upscale, frame.height*upscale))
            img3 =  cv2.resize(mat_depth_rgb[:,:,2], (frame.width*upscale, frame.height*upscale))
            img4 =  cv2.resize(mat_depth_rgb[:,:,:], (frame.width*upscale, frame.height*upscale))
            amplitude_img =  cv2.resize(mat_amplitude, (frame.width*upscale, frame.height*upscale))

            #cv2.imshow('R Channel', img1)
            #cv2.imshow('G Channel', img2)
            #cv2.imshow('B Channel', img3)
            cv2.imshow('RGB Channel', img4)
            cv2.imshow('Amplitude', amplitude_img)

            if cv2.waitKey(1) == 27: break

def Voxel(X,Y,Z,size,material):
    #TODO: blender
    pass



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