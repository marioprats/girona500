"""autogenerated by genmsg_py from VehicleStatus.msg. Do not edit."""
import roslib.message
import struct

import std_msgs.msg

class VehicleStatus(roslib.message.Message):
  _md5sum = "bcf01ceb8c5405a89ea107ddd319eef5"
  _type = "safety_g500/VehicleStatus"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

bool dvl_sts
bool svs_sts
bool fog_sts
bool nav_sts
bool bat_sts
bool t_sts
bool h_sts
bool p_sts
bool water_sts


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
  __slots__ = ['header','dvl_sts','svs_sts','fog_sts','nav_sts','bat_sts','t_sts','h_sts','p_sts','water_sts']
  _slot_types = ['Header','bool','bool','bool','bool','bool','bool','bool','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,dvl_sts,svs_sts,fog_sts,nav_sts,bat_sts,t_sts,h_sts,p_sts,water_sts
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(VehicleStatus, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.dvl_sts is None:
        self.dvl_sts = False
      if self.svs_sts is None:
        self.svs_sts = False
      if self.fog_sts is None:
        self.fog_sts = False
      if self.nav_sts is None:
        self.nav_sts = False
      if self.bat_sts is None:
        self.bat_sts = False
      if self.t_sts is None:
        self.t_sts = False
      if self.h_sts is None:
        self.h_sts = False
      if self.p_sts is None:
        self.p_sts = False
      if self.water_sts is None:
        self.water_sts = False
    else:
      self.header = std_msgs.msg._Header.Header()
      self.dvl_sts = False
      self.svs_sts = False
      self.fog_sts = False
      self.nav_sts = False
      self.bat_sts = False
      self.t_sts = False
      self.h_sts = False
      self.p_sts = False
      self.water_sts = False

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
      buff.write(_struct_9B.pack(_x.dvl_sts, _x.svs_sts, _x.fog_sts, _x.nav_sts, _x.bat_sts, _x.t_sts, _x.h_sts, _x.p_sts, _x.water_sts))
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
      end += 9
      (_x.dvl_sts, _x.svs_sts, _x.fog_sts, _x.nav_sts, _x.bat_sts, _x.t_sts, _x.h_sts, _x.p_sts, _x.water_sts,) = _struct_9B.unpack(str[start:end])
      self.dvl_sts = bool(self.dvl_sts)
      self.svs_sts = bool(self.svs_sts)
      self.fog_sts = bool(self.fog_sts)
      self.nav_sts = bool(self.nav_sts)
      self.bat_sts = bool(self.bat_sts)
      self.t_sts = bool(self.t_sts)
      self.h_sts = bool(self.h_sts)
      self.p_sts = bool(self.p_sts)
      self.water_sts = bool(self.water_sts)
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
      buff.write(_struct_9B.pack(_x.dvl_sts, _x.svs_sts, _x.fog_sts, _x.nav_sts, _x.bat_sts, _x.t_sts, _x.h_sts, _x.p_sts, _x.water_sts))
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
      end += 9
      (_x.dvl_sts, _x.svs_sts, _x.fog_sts, _x.nav_sts, _x.bat_sts, _x.t_sts, _x.h_sts, _x.p_sts, _x.water_sts,) = _struct_9B.unpack(str[start:end])
      self.dvl_sts = bool(self.dvl_sts)
      self.svs_sts = bool(self.svs_sts)
      self.fog_sts = bool(self.fog_sts)
      self.nav_sts = bool(self.nav_sts)
      self.bat_sts = bool(self.bat_sts)
      self.t_sts = bool(self.t_sts)
      self.h_sts = bool(self.h_sts)
      self.p_sts = bool(self.p_sts)
      self.water_sts = bool(self.water_sts)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3I = struct.Struct("<3I")
_struct_9B = struct.Struct("<9B")
