// Auto-generated. Do not edit!

// (in-package xbee.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class trigger {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.go_mqs = null;
    }
    else {
      if (initObj.hasOwnProperty('go_mqs')) {
        this.go_mqs = initObj.go_mqs
      }
      else {
        this.go_mqs = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type trigger
    // Serialize message field [go_mqs]
    bufferOffset = _serializer.bool(obj.go_mqs, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type trigger
    let len;
    let data = new trigger(null);
    // Deserialize message field [go_mqs]
    data.go_mqs = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xbee/trigger';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a233c8f8bcc02b3d95334687e9718eb4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Trigger message to tell mqs_handshake that auto_ctrls is available
    bool go_mqs
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new trigger(null);
    if (msg.go_mqs !== undefined) {
      resolved.go_mqs = msg.go_mqs;
    }
    else {
      resolved.go_mqs = false
    }

    return resolved;
    }
};

module.exports = trigger;
