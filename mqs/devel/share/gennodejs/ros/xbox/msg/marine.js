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

class marine {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.thl = null;
      this.strm = null;
    }
    else {
      if (initObj.hasOwnProperty('thl')) {
        this.thl = initObj.thl
      }
      else {
        this.thl = 0.0;
      }
      if (initObj.hasOwnProperty('strm')) {
        this.strm = initObj.strm
      }
      else {
        this.strm = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type marine
    // Serialize message field [thl]
    bufferOffset = _serializer.float32(obj.thl, buffer, bufferOffset);
    // Serialize message field [strm]
    bufferOffset = _serializer.float32(obj.strm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type marine
    let len;
    let data = new marine(null);
    // Deserialize message field [thl]
    data.thl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [strm]
    data.strm = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xbox/marine';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e8ddfec22a5ef4b813c4ad7dee7890ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for marine commands
    
    float32 thl	#controls the throttle
    float32 strm	#controls the waterjet nozzle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new marine(null);
    if (msg.thl !== undefined) {
      resolved.thl = msg.thl;
    }
    else {
      resolved.thl = 0.0
    }

    if (msg.strm !== undefined) {
      resolved.strm = msg.strm;
    }
    else {
      resolved.strm = 0.0
    }

    return resolved;
    }
};

module.exports = marine;
