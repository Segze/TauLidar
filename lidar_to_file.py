import numpy as np
import csv
import os

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

def save_raw(data: list,file: str):
     if (not os.path.exists(f"./{file}.csv")):
        with open(f"{file}", 'wb') as f:
            for byte in data:
                f.write(byte)
def open_raw(file: str, type: str):
    raw = []
    if type == "DISTANCE":
        with open(f"{file}", 'rb') as f:
            p = f.read(2)
            i = int.from_bytes(p,"big")
            raw.append(i)
    return raw

