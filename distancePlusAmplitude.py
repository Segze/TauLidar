import argparse
import numpy as np
import cv2
from TauLidarCommon.frame import FrameType
from TauLidarCamera.camera import Camera
import record_lidar 
import time
import threading
def setup():

    cv2.namedWindow('Depth Map')
    cv2.namedWindow('Amplitude')

    cv2.moveWindow('Depth Map', 20, 20)
    cv2.moveWindow('Amplitude', 20, 360)

    return None



def run(d,load_all_in_mem = False):
    if load_all_in_mem:
        count = record_lidar.countFrames(None,d,".\\grabaciones")
        frames = [None]*count
        start= time.time()
        print("Cargando frames")
        def _loadFrame(x,y):
            try:frames[x] = (record_lidar.readFrame(None,d,".\\grabaciones",x))
            except:pass
        for I in range(0,count,5):

            thread = threading.Thread(
                target= _loadFrame,
                args=(I,"")
                )
            thread.start()
            thread2 = threading.Thread(
                target= _loadFrame,
                args=(I+1,"")
                )
            thread2.start()
            thread3 = threading.Thread(
                target= _loadFrame,
                args=(I+2,"")
                )
            thread3.start()
            thread4 = threading.Thread(
                target= _loadFrame,
                args=(I+3,"")
                )
            thread4.start()
            thread5 = threading.Thread(
                target= _loadFrame,
                args=(I+4,"")
                )
            thread5.start()

            
            thread.join()
            thread2.join()
            thread3.join()
            thread4.join()
            thread5.join()
            #frames[I+1] = (record_lidar.readFrame(None,d,".\\grabaciones",I+1))
            
            

            print(f"{I}/{count} frames cargados ({(100*I/count).__trunc__()}%), tiempo transcurrido: {int(time.time()-start)} s")
        for i,packd in enumerate(frames):
            frame = packd[0]
            meta = packd[1]
            if frame:
                delay = 0
                try:delay = frames[i+1][1]['time'] - meta['time']
                except: delay=2
                mat_depth_rgb = np.frombuffer(frame.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame.height, frame.width, 3)
                mat_depth_rgb = mat_depth_rgb.astype(np.uint8)
  
                mat_amplitude = np.frombuffer(frame.data_amplitude, dtype=np.float32, count=-1, offset=0).reshape(frame.height, frame.width)
                mat_amplitude = mat_amplitude.astype(np.uint8)
  
                  # Upscalling the image
                upscale = 4
                depth_img =  cv2.resize(mat_depth_rgb, (frame.width*upscale, frame.height*upscale))
                amplitude_img =  cv2.resize(mat_amplitude, (frame.width*upscale, frame.height*upscale))
  
                cv2.imshow('Depth Map', depth_img)
                cv2.imshow('Amplitude', amplitude_img)
                time.sleep(delay)
                if cv2.waitKey(1) == 27: break  
        
    else:
        for I in range(0,record_lidar.countFrames(None,d,".\\grabaciones")):
            frame, meta = record_lidar.readFrame(None,d,".\\grabaciones",I)

            if frame:
                mat_depth_rgb = np.frombuffer(frame.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame.height, frame.width, 3)
                mat_depth_rgb = mat_depth_rgb.astype(np.uint8)

                mat_amplitude = np.frombuffer(frame.data_amplitude, dtype=np.float32, count=-1, offset=0).reshape(frame.height, frame.width)
                mat_amplitude = mat_amplitude.astype(np.uint8)

                # Upscalling the image
                upscale = 4
                depth_img =  cv2.resize(mat_depth_rgb, (frame.width*upscale, frame.height*upscale))
                amplitude_img =  cv2.resize(mat_amplitude, (frame.width*upscale, frame.height*upscale))

                cv2.imshow('Depth Map', depth_img)
                cv2.imshow('Amplitude', amplitude_img)

                if cv2.waitKey(1) == 27: break


def cleanup(camera):
    print('\nShutting down ...')
    cv2.destroyAllWindows()


if __name__ == "__main__":
    setup()

    try:
        run("2023 02 12 23 09 55",load_all_in_mem=True)
    except Exception as e:
        print(e)
