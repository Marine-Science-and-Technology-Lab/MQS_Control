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

class mqs_ctrl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmds = null;
    }
    else {
      if (initObj.hasOwnProperty('cmds')) {
        this.cmds = initObj.cmds
      }
      else {
        this.cmds = new Array(16).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mqs_ctrl
    // Check that the constant length array field [cmds] has the right length
    if (obj.cmds.length !== 16) {
      throw new Error('Unable to serialize array field cmds - length must be 16')
    }
    // Serialize message field [cmds]
    bufferOffset = _arraySerializer.uint8(obj.cmds, buffer, bufferOffset, 16);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mqs_ctrl
    let len;
    let data = new mqs_ctrl(null);
    // Deserialize message field [cmds]
    data.cmds = _arrayDeserializer.uint8(buffer, bufferOffset, 16)
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xbee/mqs_ctrl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '65c47160f0c57a1a6dca334e2e8b2754';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Message where final commands for the MQS are concatenated to
    
    uint8[16] cmds
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mqs_ctrl(null);
    if (msg.cmds !== undefined) {
      resolved.cmds = msg.cmds;
    }
    else {
      resolved.cmds = new Array(16).fill(0)
    }

    return resolved;
    }
};

module.exports = mqs_ctrl;
