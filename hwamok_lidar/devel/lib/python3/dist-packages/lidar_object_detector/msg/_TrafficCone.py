# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from lidar_object_detector/TrafficCone.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class TrafficCone(genpy.Message):
  _md5sum = "fcb703df87d24291d755127aee75bb7e"
  _type = "lidar_object_detector/TrafficCone"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 objectCounts
float64[100] centerX
float64[100] centerY
float64[100] centerZ
float64[100] lengthX
float64[100] lengthY
float64[100] lengthZ"""
  __slots__ = ['objectCounts','centerX','centerY','centerZ','lengthX','lengthY','lengthZ']
  _slot_types = ['int32','float64[100]','float64[100]','float64[100]','float64[100]','float64[100]','float64[100]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       objectCounts,centerX,centerY,centerZ,lengthX,lengthY,lengthZ

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TrafficCone, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.objectCounts is None:
        self.objectCounts = 0
      if self.centerX is None:
        self.centerX = [0.] * 100
      if self.centerY is None:
        self.centerY = [0.] * 100
      if self.centerZ is None:
        self.centerZ = [0.] * 100
      if self.lengthX is None:
        self.lengthX = [0.] * 100
      if self.lengthY is None:
        self.lengthY = [0.] * 100
      if self.lengthZ is None:
        self.lengthZ = [0.] * 100
    else:
      self.objectCounts = 0
      self.centerX = [0.] * 100
      self.centerY = [0.] * 100
      self.centerZ = [0.] * 100
      self.lengthX = [0.] * 100
      self.lengthY = [0.] * 100
      self.lengthZ = [0.] * 100

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
      _x = self.objectCounts
      buff.write(_get_struct_i().pack(_x))
      buff.write(_get_struct_100d().pack(*self.centerX))
      buff.write(_get_struct_100d().pack(*self.centerY))
      buff.write(_get_struct_100d().pack(*self.centerZ))
      buff.write(_get_struct_100d().pack(*self.lengthX))
      buff.write(_get_struct_100d().pack(*self.lengthY))
      buff.write(_get_struct_100d().pack(*self.lengthZ))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (self.objectCounts,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 800
      self.centerX = _get_struct_100d().unpack(str[start:end])
      start = end
      end += 800
      self.centerY = _get_struct_100d().unpack(str[start:end])
      start = end
      end += 800
      self.centerZ = _get_struct_100d().unpack(str[start:end])
      start = end
      end += 800
      self.lengthX = _get_struct_100d().unpack(str[start:end])
      start = end
      end += 800
      self.lengthY = _get_struct_100d().unpack(str[start:end])
      start = end
      end += 800
      self.lengthZ = _get_struct_100d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.objectCounts
      buff.write(_get_struct_i().pack(_x))
      buff.write(self.centerX.tostring())
      buff.write(self.centerY.tostring())
      buff.write(self.centerZ.tostring())
      buff.write(self.lengthX.tostring())
      buff.write(self.lengthY.tostring())
      buff.write(self.lengthZ.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (self.objectCounts,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 800
      self.centerX = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=100)
      start = end
      end += 800
      self.centerY = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=100)
      start = end
      end += 800
      self.centerZ = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=100)
      start = end
      end += 800
      self.lengthX = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=100)
      start = end
      end += 800
      self.lengthY = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=100)
      start = end
      end += 800
      self.lengthZ = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=100)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_100d = None
def _get_struct_100d():
    global _struct_100d
    if _struct_100d is None:
        _struct_100d = struct.Struct("<100d")
    return _struct_100d
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
