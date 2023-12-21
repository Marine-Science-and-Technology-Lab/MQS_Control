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

class script_ctrl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.script_ctrls = null;
    }
    else {
      if (initObj.hasOwnProperty('script_ctrls')) {
        this.script_ctrls = initObj.script_ctrls
      }
      else {
        this.script_ctrls = new Array(5).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type script_ctrl
    // Check that the constant length array field [script_ctrls] has the right length
    if (obj.script_ctrls.length !== 5) {
      throw new Error('Unable to serialize array field script_ctrls - length must be 5')
    }
    // Serialize message field [script_ctrls]
    bufferOffset = _arraySerializer.uint8(obj.script_ctrls, buffer, bufferOffset, 5);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type script_ctrl
    let len;
    let data = new script_ctrl(null);
    // Deserialize message field [script_ctrls]
    data.script_ctrls = _arrayDeserializer.uint8(buffer, bufferOffset, 5)
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xbee/script_ctrl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f4ceca9c837a67ee537847b65bec8a91';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Message where each sub command from csv script is published into
    
    uint8[5] script_ctrls
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new script_ctrl(null);
    if (msg.script_ctrls !== undefined) {
      resolved.script_ctrls = msg.script_ctrls;
    }
    else {
      resolved.script_ctrls = new Array(5).fill(0)
    }

    return resolved;
    }
};

module.exports = script_ctrl;
