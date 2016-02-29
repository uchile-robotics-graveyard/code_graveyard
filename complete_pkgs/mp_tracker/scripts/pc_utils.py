import numpy as np
import struct
import roslib
#roslib.load_manifest("pcl")
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def read_pc2(pc2_msg):
    """ 
    Reads a PointCloud2 data and output a NumPy array
    """
    num_fields = len(pc2_msg.fields)
    fields_name = [x.name for x in pc2_msg.fields]
    if pc2_msg.height * pc2_msg.width == 0:
        return (fields_name, None)
    data = np.zeros(shape=(pc2_msg.height, pc2_msg.width, num_fields))
    for i in range(0, pc2_msg.height):
        for j in range(0, pc2_msg.width):
            for k in range(num_fields):
                start = i*pc2_msg.row_step + j*pc2_msg.point_step + pc2_msg.fields[k].offset
                end = -1
                field_format = ''
                if pc2_msg.fields[k].datatype == PointField.INT8:
                    field_format = 'b'
                    end = start + 1
                elif  pc2_msg.fields[k].datatype == PointField.UINT8:
                    field_format = 'B'
                    end = start + 1
                elif  pc2_msg.fields[k].datatype == PointField.INT16:
                    field_format = 'h'
                    end = start + 2
                elif  pc2_msg.fields[k].datatype == PointField.UINT16:
                    field_format = 'H'
                    end = start + 2
                elif  pc2_msg.fields[k].datatype == PointField.INT32:
                    field_format = 'i'
                    end = start + 4
                elif  pc2_msg.fields[k].datatype == PointField.UINT32:
                    field_format = 'I'
                    end = start + 4
                elif  pc2_msg.fields[k].datatype == PointField.FLOAT32:
                    field_format = 'f'
                    end = start + 4
                elif  pc2_msg.fields[k].datatype == PointField.FLOAT64:
                    field_format = 'd'
                    end = start + 8
                if start > len(pc2_msg.data) or end > len(pc2_msg.data):
                    raise IndexError
                raw = struct.unpack(field_format, pc2_msg.data[start:end])
                data[i,j,k] = float(raw[0])
    # Reshape if needed
    if pc2_msg.width == 1 or pc2_msg.height == 1:
        data = data.reshape(pc2_msg.width*pc2_msg.height, num_fields)
    return (fields_name, data)

# For now only floats
default_datatype = PointField.FLOAT32
default_datatype_length = 4
def write_pc2(fields, timestamp, frame_id, data):
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.header.seq = 0
    msg.header.stamp = timestamp
    msg.fields = []
    for i in range(len(fields)):
        field = PointField()
        field.name = fields[i]
        field.datatype = PointField.FLOAT32
        field.offset = default_datatype_length * i
        field.count = 1
        msg.fields.append(field)
    if data.ndim == 3:
        msg.height = data.shape[0]
        msg.width = data.shape[1]
    else:
        msg.height = 1
        msg.width = data.shape[0]
    num_points = msg.height * msg.width 
    msg.point_step = len(fields) * default_datatype_length
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = msg.height == 1
    msg.data = [0 for i in range(msg.row_step * msg.height)]
    msg.is_bigendian = True
    indx = 0
    for i in range(num_points):
        for j in range(len(msg.fields)):
            value = 0.0
            if data.ndim == 2:
                value = float(data[i,j])
            else:
                pass
            values = struct.pack('f', value)
            msg.data[indx] = ord(values[0]); indx+=1
            msg.data[indx] = ord(values[1]); indx+=1
            msg.data[indx] = ord(values[2]); indx+=1
            msg.data[indx] = ord(values[3]); indx+=1
    print msg
    return msg