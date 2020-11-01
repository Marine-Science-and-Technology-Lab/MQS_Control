// Auto-generated. Do not edit!

// (in-package xbox.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class op {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.esc = null;
      this.bp = null;
      this.daq = null;
      this.wrt = null;
      this.cp = null;
      this.rvm = null;
      this.abort = null;
    }
    else {
      if (initObj.hasOwnProperty('esc')) {
        this.esc = initObj.esc
      }
      else {
        this.esc = 0;
      }
      if (initObj.hasOwnProperty('bp')) {
        this.bp = initObj.bp
      }
      else {
        this.bp = 0;
      }
      if (initObj.hasOwnProperty('daq')) {
        this.daq = initObj.daq
      }
      else {
        this.daq = 0;
      }
      if (initObj.hasOwnProperty('wrt')) {
        this.wrt = initObj.wrt
      }
      else {
        this.wrt = 0;
      }
      if (initObj.hasOwnProperty('cp')) {
        this.cp = initObj.cp
      }
      else {
        this.cp = 0;
      }
      if (initObj.hasOwnProperty('rvm')) {
        this.rvm = initObj.rvm
      }
      else {
        this.rvm = 0;
      }
      if (initObj.hasOwnProperty('abort')) {
        this.abort = initObj.abort
      }
      else {
        this.abort = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type op
    // Serialize message field [esc]
    bufferOffset = _serializer.uint8(obj.esc, buffer, bufferOffset);
    // Serialize message field [bp]
    bufferOffset = _serializer.uint8(obj.bp, buffer, bufferOffset);
    // Serialize message field [daq]
    bufferOffset = _serializer.uint8(obj.daq, buffer, bufferOffset);
    // Serialize message field [wrt]
    bufferOffset = _serializer.uint8(obj.wrt, buffer, bufferOffset);
    // Serialize message field [cp]
    bufferOffset = _serializer.uint8(obj.cp, buffer, bufferOffset);
    // Serialize message field [rvm]
    bufferOffset = _serializer.uint8(obj.rvm, buffer, bufferOffset);
    // Serialize message field [abort]
    bufferOffset = _serializer.uint8(obj.abort, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type op
    let len;
    let data = new op(null);
    // Deserialize message field [esc]
    data.esc = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [bp]
    data.bp = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [daq]
    data.daq = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [wrt]
    data.wrt = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cp]
    data.cp = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rvm]
    data.rvm = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abort]
    data.abort = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xbox/op';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ce1000fdad5bdc47b754e81eac323f68';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message files for all of the switch commands for the MQS
    
    uint8 esc  #turn on/off the esc's
    uint8 bp   #turn on/off the bilge pump
    uint8 daq  #turn on/off the DAQ
    uint8 wrt  #raise and lower the wheel retraction
    uint8 cp   #turn on/off the cooling pumps for the ESC's
    uint8 rvm  #hold to engage reverse mode for marine
    uint8 abort #abort joystick operation. Change over to transmitter on arduino
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new op(null);
    if (msg.esc !== undefined) {
      resolved.esc = msg.esc;
    }
    else {
      resolved.esc = 0
    }

    if (msg.bp !== undefined) {
      resolved.bp = msg.bp;
    }
    else {
      resolved.bp = 0
    }

    if (msg.daq !== undefined) {
      resolved.daq = msg.daq;
    }
    else {
      resolved.daq = 0
    }

    if (msg.wrt !== undefined) {
      resolved.wrt = msg.wrt;
    }
    else {
      resolved.wrt = 0
    }

    if (msg.cp !== undefined) {
      resolved.cp = msg.cp;
    }
    else {
      resolved.cp = 0
    }

    if (msg.rvm !== undefined) {
      resolved.rvm = msg.rvm;
    }
    else {
      resolved.rvm = 0
    }

    if (msg.abort !== undefined) {
      resolved.abort = msg.abort;
    }
    else {
      resolved.abort = 0
    }

    return resolved;
    }
};

module.exports = op;
