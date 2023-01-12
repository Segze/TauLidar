import bpy
import time
from TauLidarCommon.frame import FrameType, Frame
from TauLidarCamera.camera import Camera
from TauLidarCommon.d3 import ImageColorizer

def remove_all(type=None):
    # Possible type:
    # "MESH", "CURVE", "SURFACE", "META", "FONT", "ARMATURE",
    # "LATTICE", "EMPTY", "CAMERA", "LIGHT"
    if type:
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.select_by_type(type=type)
        bpy.ops.object.delete()
    else:
        # Remove all elements in scene
        bpy.ops.object.select_all(action="SELECT")
        bpy.ops.object.delete(use_global=False)

def remove_object(obj):
    if obj.type == 'MESH':
        if obj.data.name in bpy.data.meshes:
            bpy.data.meshes.remove(obj.data)
        if obj.name in bpy.context.scene.objects:
            bpy.context.scene.objects.unlink(obj)
        bpy.data.objects.remove(obj)
    else:
        raise NotImplementedError('Other types not implemented yet besides \'MESH\'')


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
        remove_all()
        frame = camera.readFrame(FrameType.DISTANCE)
        if frame:
            space_color = frame.points_3d
            x = []
            y = []
            z = []   
            rgb = []
            space = []
            for i,arr in enumerate(space_color):
                if (i%1)!=0:
                    continue
                x.append(arr[0])
                y.append(arr[1])
                z.append(arr[2])
                r = arr[3]/255
                g = arr[4]/255
                b = arr[5]/255
                #Voxel(arr[0],arr[1],arr[2],0.01)
                space.append((arr[0],arr[1],arr[2]))
                rgb.append((r,g,b))

        mesh(space,"Snapshot")
        
def Voxel(X,Y,Z,size=1.0):
    #TODO: blender
    return bpy.ops.mesh.primitive_cube_add(location = [X,Y,Z],size=size)
def mesh(vertices, name):
    edges = []
    faces = []
    new_mesh = bpy.data.meshes.new(name+'_mesh')
    new_mesh.from_pydata(vertices, edges, faces)
    new_mesh.update()
    new_object = bpy.data.objects.new(name+'_object', new_mesh)
    new_collection = bpy.data.collections.new(name+'_collection')
    bpy.context.scene.collection.children.link(new_collection)
    new_collection.objects.link(new_object)

def cleanup(camera):
    print('\nShutting down ...')
    camera.close()


port = "COM5"
camera = setup(port)


if camera:

    try:
        run(camera)
    except Exception as e:
        print(e)
        Voxel(0,0,0,5)
        exit(0)
    cleanup(camera)