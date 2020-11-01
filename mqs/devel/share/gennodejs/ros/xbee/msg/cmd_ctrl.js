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

class cmd_ctrl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd_ctrls = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd_ctrls')) {
        this.cmd_ctrls = initObj.cmd_ctrls
      }
      else {
        this.cmd_ctrls = new Array(16).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmd_ctrl
    // Check that the constant length array field [cmd_ctrls] has the right length
    if (obj.cmd_ctrls.length !== 16) {
      throw new Error('Unable to serialize array field cmd_ctrls - length must be 16')
    }
    // Serialize message field [cmd_ctrls]
    bufferOffset = _arraySerializer.uint8(obj.cmd_ctrls, buffer, bufferOffset, 16);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmd_ctrl
    let len;
    let data = new cmd_ctrl(null);
    // Deserialize message field [cmd_ctrls]
    data.cmd_ctrls = _arrayDeserializer.uint8(buffer, bufferOffset, 16)
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xbee/cmd_ctrl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0180a78c48bb4a1e71184e050bf8b39c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Message where each sub command from joy or keyboard message for the MQS is published into
    
    uint8[16] cmd_ctrls
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cmd_ctrl(null);
    if (msg.cmd_ctrls !== undefined) {
      resolved.cmd_ctrls = msg.cmd_ctrls;
    }
    else {
      resolved.cmd_ctrls = new Array(16).fill(0)
    }

    return resolved;
    }
};

module.exports = cmd_ctrl;
