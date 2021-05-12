// Auto-generated. Do not edit!

// (in-package lab7.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class test {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.A = null;
      this.B = null;
      this.Sum = null;
    }
    else {
      if (initObj.hasOwnProperty('A')) {
        this.A = initObj.A
      }
      else {
        this.A = 0;
      }
      if (initObj.hasOwnProperty('B')) {
        this.B = initObj.B
      }
      else {
        this.B = 0;
      }
      if (initObj.hasOwnProperty('Sum')) {
        this.Sum = initObj.Sum
      }
      else {
        this.Sum = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type test
    // Serialize message field [A]
    bufferOffset = _serializer.int64(obj.A, buffer, bufferOffset);
    // Serialize message field [B]
    bufferOffset = _serializer.int64(obj.B, buffer, bufferOffset);
    // Serialize message field [Sum]
    bufferOffset = _serializer.int64(obj.Sum, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type test
    let len;
    let data = new test(null);
    // Deserialize message field [A]
    data.A = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [B]
    data.B = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [Sum]
    data.Sum = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lab7/test';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c1459700de7bba184f907aed36761b8d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 A
    int64 B
    
    int64 Sum
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new test(null);
    if (msg.A !== undefined) {
      resolved.A = msg.A;
    }
    else {
      resolved.A = 0
    }

    if (msg.B !== undefined) {
      resolved.B = msg.B;
    }
    else {
      resolved.B = 0
    }

    if (msg.Sum !== undefined) {
      resolved.Sum = msg.Sum;
    }
    else {
      resolved.Sum = 0
    }

    return resolved;
    }
};

module.exports = test;
