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

class land {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fwd = null;
      this.rev = null;
      this.strl = null;
    }
    else {
      if (initObj.hasOwnProperty('fwd')) {
        this.fwd = initObj.fwd
      }
      else {
        this.fwd = 0.0;
      }
      if (initObj.hasOwnProperty('rev')) {
        this.rev = initObj.rev
      }
      else {
        this.rev = 0.0;
      }
      if (initObj.hasOwnProperty('strl')) {
        this.strl = initObj.strl
      }
      else {
        this.strl = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type land
    // Serialize message field [fwd]
    bufferOffset = _serializer.float32(obj.fwd, buffer, bufferOffset);
    // Serialize message field [rev]
    bufferOffset = _serializer.float32(obj.rev, buffer, bufferOffset);
    // Serialize message field [strl]
    bufferOffset = _serializer.float32(obj.strl, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type land
    let len;
    let data = new land(null);
    // Deserialize message field [fwd]
    data.fwd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rev]
    data.rev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [strl]
    data.strl = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xbox/land';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'edaf8b5219d5128d3faa0e4ddabb09c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message file for land control commands
    
    float32 fwd
    float32 rev
    float32 strl
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new land(null);
    if (msg.fwd !== undefined) {
      resolved.fwd = msg.fwd;
    }
    else {
      resolved.fwd = 0.0
    }

    if (msg.rev !== undefined) {
      resolved.rev = msg.rev;
    }
    else {
      resolved.rev = 0.0
    }

    if (msg.strl !== undefined) {
      resolved.strl = msg.strl;
    }
    else {
      resolved.strl = 0.0
    }

    return resolved;
    }
};

module.exports = land;
