"""autogenerated by genmsg_py from MissionStatus.msg. Do not edit."""
import roslib.message
import struct

import std_msgs.msg

class MissionStatus(roslib.message.Message):
  _md5sum = "8c0c8ca16cc19a05ea30e3d92308937f"
  _type = "safety_g500/MissionStatus"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

int32 current_wp
int32 total_wp

bool altitude_mode

float64 current_north
float64 current_east
float32 current_depth
float32 current_altitude

float64 wp_north
float64 wp_east
float32 wp_depth_altitude
int32 wp_remaining_time


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
  __slots__ = ['header','current_wp','total_wp','altitude_mode','current_north','current_east','current_depth','current_altitude','wp_north','wp_east','wp_depth_altitude','wp_remaining_time']
  _slot_types = ['Header','int32','int32','bool','float64','float64','float32','float32','float64','float64','float32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,current_wp,total_wp,altitude_mode,current_north,current_east,current_depth,current_altitude,wp_north,wp_east,wp_depth_altitude,wp_remaining_time
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(MissionStatus, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.current_wp is None:
        self.current_wp = 0
      if self.total_wp is None:
        self.total_wp = 0
      if self.altitude_mode is None:
        self.altitude_mode = False
      if self.current_north is None:
        self.current_north = 0.
      if self.current_east is None:
        self.current_east = 0.
      if self.current_depth is None:
        self.current_depth = 0.
      if self.current_altitude is None:
        self.current_altitude = 0.
      if self.wp_north is None:
        self.wp_north = 0.
      if self.wp_east is None:
        self.wp_east = 0.
      if self.wp_depth_altitude is None:
        self.wp_depth_altitude = 0.
      if self.wp_remaining_time is None:
        self.wp_remaining_time = 0
    else:
      self.header = std_msgs.msg._Header.Header()
      self.current_wp = 0
      self.total_wp = 0
      self.altitude_mode = False
      self.current_north = 0.
      self.current_east = 0.
      self.current_depth = 0.
      self.current_altitude = 0.
      self.wp_north = 0.
      self.wp_east = 0.
      self.wp_depth_altitude = 0.
      self.wp_remaining_time = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2iB2d2f2dfi.pack(_x.current_wp, _x.total_wp, _x.altitude_mode, _x.current_north, _x.current_east, _x.current_depth, _x.current_altitude, _x.wp_north, _x.wp_east, _x.wp_depth_altitude, _x.wp_remaining_time))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 57
      (_x.current_wp, _x.total_wp, _x.altitude_mode, _x.current_north, _x.current_east, _x.current_depth, _x.current_altitude, _x.wp_north, _x.wp_east, _x.wp_depth_altitude, _x.wp_remaining_time,) = _struct_2iB2d2f2dfi.unpack(str[start:end])
      self.altitude_mode = bool(self.altitude_mode)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2iB2d2f2dfi.pack(_x.current_wp, _x.total_wp, _x.altitude_mode, _x.current_north, _x.current_east, _x.current_depth, _x.current_altitude, _x.wp_north, _x.wp_east, _x.wp_depth_altitude, _x.wp_remaining_time))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 57
      (_x.current_wp, _x.total_wp, _x.altitude_mode, _x.current_north, _x.current_east, _x.current_depth, _x.current_altitude, _x.wp_north, _x.wp_east, _x.wp_depth_altitude, _x.wp_remaining_time,) = _struct_2iB2d2f2dfi.unpack(str[start:end])
      self.altitude_mode = bool(self.altitude_mode)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3I = struct.Struct("<3I")
_struct_2iB2d2f2dfi = struct.Struct("<2iB2d2f2dfi")
