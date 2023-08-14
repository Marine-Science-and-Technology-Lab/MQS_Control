# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from xbox/op.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class op(genpy.Message):
  _md5sum = "a88ed851050f435447fefa00180357ec"
  _type = "xbox/op"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# Message files for all of the switch commands for the MQS

uint8 esc  #turn on/off the esc's
uint8 bp   #turn on/off the bilge pump
uint8 daq  #turn on/off the DAQ
uint8 wrt  #raise and lower the wheel retraction
uint8 cp   #turn on/off the cooling pumps for the ESC's
uint8 rvm  #hold to engage reverse mode for marine
uint8 abort #abort joystick operation. Change over to transmitter on arduino
uint8 start #start manuever switch to begin a keyed up manuever
"""
  __slots__ = ['esc','bp','daq','wrt','cp','rvm','abort','start']
  _slot_types = ['uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       esc,bp,daq,wrt,cp,rvm,abort,start

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(op, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.esc is None:
        self.esc = 0
      if self.bp is None:
        self.bp = 0
      if self.daq is None:
        self.daq = 0
      if self.wrt is None:
        self.wrt = 0
      if self.cp is None:
        self.cp = 0
      if self.rvm is None:
        self.rvm = 0
      if self.abort is None:
        self.abort = 0
      if self.start is None:
        self.start = 0
    else:
      self.esc = 0
      self.bp = 0
      self.daq = 0
      self.wrt = 0
      self.cp = 0
      self.rvm = 0
      self.abort = 0
      self.start = 0

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
      buff.write(_get_struct_8B().pack(_x.esc, _x.bp, _x.daq, _x.wrt, _x.cp, _x.rvm, _x.abort, _x.start))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.esc, _x.bp, _x.daq, _x.wrt, _x.cp, _x.rvm, _x.abort, _x.start,) = _get_struct_8B().unpack(str[start:end])
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
      _x = self
      buff.write(_get_struct_8B().pack(_x.esc, _x.bp, _x.daq, _x.wrt, _x.cp, _x.rvm, _x.abort, _x.start))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.esc, _x.bp, _x.daq, _x.wrt, _x.cp, _x.rvm, _x.abort, _x.start,) = _get_struct_8B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_8B = None
def _get_struct_8B():
    global _struct_8B
    if _struct_8B is None:
        _struct_8B = struct.Struct("<8B")
    return _struct_8B
