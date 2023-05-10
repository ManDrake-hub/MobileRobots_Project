// Auto-generated. Do not edit!

// (in-package navigation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CalibrationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalibrationRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalibrationRequest
    let len;
    let data = new CalibrationRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'navigation/CalibrationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalibrationRequest(null);
    return resolved;
    }
};

class CalibrationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.answer = null;
    }
    else {
      if (initObj.hasOwnProperty('answer')) {
        this.answer = initObj.answer
      }
      else {
        this.answer = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalibrationResponse
    // Serialize message field [answer]
    bufferOffset = std_msgs.msg.String.serialize(obj.answer, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalibrationResponse
    let len;
    let data = new CalibrationResponse(null);
    // Deserialize message field [answer]
    data.answer = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.answer);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'navigation/CalibrationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a255970bffa18c4d56f14a3df609a6be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String answer
    
    
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalibrationResponse(null);
    if (msg.answer !== undefined) {
      resolved.answer = std_msgs.msg.String.Resolve(msg.answer)
    }
    else {
      resolved.answer = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = {
  Request: CalibrationRequest,
  Response: CalibrationResponse,
  md5sum() { return 'a255970bffa18c4d56f14a3df609a6be'; },
  datatype() { return 'navigation/Calibration'; }
};
