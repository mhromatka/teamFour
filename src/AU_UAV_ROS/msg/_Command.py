"""autogenerated by genpy from AU_UAV_ROS/Command.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Command(genpy.Message):
  _md5sum = "c53c94d81a0a5526e6ff4317b73721aa"
  _type = "AU_UAV_ROS/Command"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Header commandHeader
int16 planeID
float64 latitude
float64 longitude
float64 altitude

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['commandHeader','planeID','latitude','longitude','altitude']
  _slot_types = ['std_msgs/Header','int16','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       commandHeader,planeID,latitude,longitude,altitude

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Command, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.commandHeader is None:
        self.commandHeader = std_msgs.msg.Header()
      if self.planeID is None:
        self.planeID = 0
      if self.latitude is None:
        self.latitude = 0.
      if self.longitude is None:
        self.longitude = 0.
      if self.altitude is None:
        self.altitude = 0.
    else:
      self.commandHeader = std_msgs.msg.Header()
      self.planeID = 0
      self.latitude = 0.
      self.longitude = 0.
      self.altitude = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.commandHeader.seq, _x.commandHeader.stamp.secs, _x.commandHeader.stamp.nsecs))
      _x = self.commandHeader.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_h3d.pack(_x.planeID, _x.latitude, _x.longitude, _x.altitude))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.commandHeader is None:
        self.commandHeader = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.commandHeader.seq, _x.commandHeader.stamp.secs, _x.commandHeader.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.commandHeader.frame_id = str[start:end].decode('utf-8')
      else:
        self.commandHeader.frame_id = str[start:end]
      _x = self
      start = end
      end += 26
      (_x.planeID, _x.latitude, _x.longitude, _x.altitude,) = _struct_h3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.commandHeader.seq, _x.commandHeader.stamp.secs, _x.commandHeader.stamp.nsecs))
      _x = self.commandHeader.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_h3d.pack(_x.planeID, _x.latitude, _x.longitude, _x.altitude))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.commandHeader is None:
        self.commandHeader = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.commandHeader.seq, _x.commandHeader.stamp.secs, _x.commandHeader.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.commandHeader.frame_id = str[start:end].decode('utf-8')
      else:
        self.commandHeader.frame_id = str[start:end]
      _x = self
      start = end
      end += 26
      (_x.planeID, _x.latitude, _x.longitude, _x.altitude,) = _struct_h3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_h3d = struct.Struct("<h3d")
