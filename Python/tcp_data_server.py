import socket
import numpy
import os
import io
from PIL import Image, ImageFile
import numpy
from rawkit import raw

def recvUntilEndChar(sock, end_char):
    rec_json = {}
    #buf = b''
    buf = ""
    newbuf = ""
    i = 0
    last_key = ""
    while not newbuf == end_char and i < 500:
        newbuf = sock.recv(1).decode("utf-8") 
        if not newbuf: return None
        print(newbuf)
        if newbuf == '=':
            last_key = buf
            buf = ""
        elif newbuf == '&':
            rec_json[last_key] = buf
            buf = ""
        else:
            buf += newbuf
        i = i + 1
        #count -= len(newbuf)
    return rec_json

def convert_cr2_to_jpg(raw_image):
    raw_image_process = raw.Raw(raw_image)
    buffered_image = numpy.array(raw_image_process.to_buffer())
    if raw_image_process.metadata.orientation == 0:
        jpg_image_height = raw_image_process.metadata.height
        jpg_image_width = raw_image_process.metadata.width
    else:
        jpg_image_height = raw_image_process.metadata.width
        jpg_image_width = raw_image_process.metadata.height
    jpg_image = Image.frombytes('RGB', (jpg_image_width, jpg_image_height), buffered_image)
    return jpg_image

def create_image_from_bytes(image_bytes) -> Image.Image:
    stream = io.BytesIO(image_bytes)
    return Image.open(stream)
#byteImg = convert_cr2_to_jpg("RAW_CANON_G10.CR2")

TCP_IP = '0.0.0.0'
TCP_PORT = 8060

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((TCP_IP, TCP_PORT))
s.listen(True)
conn, addr = s.accept()
print(addr)


my_json = recvUntilEndChar(conn, '#')

print("printing dict")
for k, v in my_json.items():
    print(k, v)
#
#image_data = recvall(conn, int(length_int))

#conn.close()
#sock.send(int(size).to_bytes(16,'big'));

#image = Image.open(io.BytesIO(image_data)).convert('RGB')


conn.sendall(b'I Got Your Message')
#byteImg = convert_cr2_to_jpg(image_data)
#dataBytesIO = io.BytesIO(byteImg)
#image = Image.open(dataBytesIO)
#img.save("boat_3.jpg")



#data = numpy.fromstring(stringData, dtype='uint8')
#print("len ", int.from_bytes(length, byteorder='big'))
s.close()