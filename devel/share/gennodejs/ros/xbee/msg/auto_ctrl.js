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

class auto_ctrl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.auto_ctrls = null;
    }
    else {
      if (initObj.hasOwnProperty('auto_ctrls')) {
        this.auto_ctrls = initObj.auto_ctrls
      }
      else {
        this.auto_ctrls = new Array(16).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type auto_ctrl
    // Check that the constant length array field [auto_ctrls] has the right length
    if (obj.auto_ctrls.length !== 16) {
      throw new Error('Unable to serialize array field auto_ctrls - length must be 16')
    }
    // Serialize message field [auto_ctrls]
    bufferOffset = _arraySerializer.uint8(obj.auto_ctrls, buffer, bufferOffset, 16);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type auto_ctrl
    let len;
    let data = new auto_ctrl(null);
    // Deserialize message field [auto_ctrls]
    data.auto_ctrls = _arrayDeserializer.uint8(buffer, bufferOffset, 16)
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xbee/auto_ctrl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9d019e319a4cd95831863d035d7d77d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Message where each sub command from simulink or trigger script for the MQS is published into
    
    uint8[16] auto_ctrls
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new auto_ctrl(null);
    if (msg.auto_ctrls !== undefined) {
      resolved.auto_ctrls = msg.auto_ctrls;
    }
    else {
      resolved.auto_ctrls = new Array(16).fill(0)
    }

    return resolved;
    }
};

module.exports = auto_ctrl;
