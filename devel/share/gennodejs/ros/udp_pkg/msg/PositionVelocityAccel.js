// Auto-generated. Do not edit!

// (in-package udp_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PositionVelocityAccel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frame_id = null;
      this.stamp = null;
      this.x_pos = null;
      this.y_pos = null;
      this.z_pos = null;
      this.x_ori = null;
      this.y_ori = null;
      this.z_ori = null;
      this.w_ori = null;
      this.x_vel = null;
      this.y_vel = null;
      this.z_vel = null;
      this.x_acc = null;
      this.y_acc = null;
      this.z_acc = null;
    }
    else {
      if (initObj.hasOwnProperty('frame_id')) {
        this.frame_id = initObj.frame_id
      }
      else {
        this.frame_id = '';
      }
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = 0.0;
      }
      if (initObj.hasOwnProperty('x_pos')) {
        this.x_pos = initObj.x_pos
      }
      else {
        this.x_pos = 0.0;
      }
      if (initObj.hasOwnProperty('y_pos')) {
        this.y_pos = initObj.y_pos
      }
      else {
        this.y_pos = 0.0;
      }
      if (initObj.hasOwnProperty('z_pos')) {
        this.z_pos = initObj.z_pos
      }
      else {
        this.z_pos = 0.0;
      }
      if (initObj.hasOwnProperty('x_ori')) {
        this.x_ori = initObj.x_ori
      }
      else {
        this.x_ori = 0.0;
      }
      if (initObj.hasOwnProperty('y_ori')) {
        this.y_ori = initObj.y_ori
      }
      else {
        this.y_ori = 0.0;
      }
      if (initObj.hasOwnProperty('z_ori')) {
        this.z_ori = initObj.z_ori
      }
      else {
        this.z_ori = 0.0;
      }
      if (initObj.hasOwnProperty('w_ori')) {
        this.w_ori = initObj.w_ori
      }
      else {
        this.w_ori = 0.0;
      }
      if (initObj.hasOwnProperty('x_vel')) {
        this.x_vel = initObj.x_vel
      }
      else {
        this.x_vel = 0.0;
      }
      if (initObj.hasOwnProperty('y_vel')) {
        this.y_vel = initObj.y_vel
      }
      else {
        this.y_vel = 0.0;
      }
      if (initObj.hasOwnProperty('z_vel')) {
        this.z_vel = initObj.z_vel
      }
      else {
        this.z_vel = 0.0;
      }
      if (initObj.hasOwnProperty('x_acc')) {
        this.x_acc = initObj.x_acc
      }
      else {
        this.x_acc = 0.0;
      }
      if (initObj.hasOwnProperty('y_acc')) {
        this.y_acc = initObj.y_acc
      }
      else {
        this.y_acc = 0.0;
      }
      if (initObj.hasOwnProperty('z_acc')) {
        this.z_acc = initObj.z_acc
      }
      else {
        this.z_acc = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PositionVelocityAccel
    // Serialize message field [frame_id]
    bufferOffset = _serializer.string(obj.frame_id, buffer, bufferOffset);
    // Serialize message field [stamp]
    bufferOffset = _serializer.float64(obj.stamp, buffer, bufferOffset);
    // Serialize message field [x_pos]
    bufferOffset = _serializer.float64(obj.x_pos, buffer, bufferOffset);
    // Serialize message field [y_pos]
    bufferOffset = _serializer.float64(obj.y_pos, buffer, bufferOffset);
    // Serialize message field [z_pos]
    bufferOffset = _serializer.float64(obj.z_pos, buffer, bufferOffset);
    // Serialize message field [x_ori]
    bufferOffset = _serializer.float64(obj.x_ori, buffer, bufferOffset);
    // Serialize message field [y_ori]
    bufferOffset = _serializer.float64(obj.y_ori, buffer, bufferOffset);
    // Serialize message field [z_ori]
    bufferOffset = _serializer.float64(obj.z_ori, buffer, bufferOffset);
    // Serialize message field [w_ori]
    bufferOffset = _serializer.float64(obj.w_ori, buffer, bufferOffset);
    // Serialize message field [x_vel]
    bufferOffset = _serializer.float64(obj.x_vel, buffer, bufferOffset);
    // Serialize message field [y_vel]
    bufferOffset = _serializer.float64(obj.y_vel, buffer, bufferOffset);
    // Serialize message field [z_vel]
    bufferOffset = _serializer.float64(obj.z_vel, buffer, bufferOffset);
    // Serialize message field [x_acc]
    bufferOffset = _serializer.float64(obj.x_acc, buffer, bufferOffset);
    // Serialize message field [y_acc]
    bufferOffset = _serializer.float64(obj.y_acc, buffer, bufferOffset);
    // Serialize message field [z_acc]
    bufferOffset = _serializer.float64(obj.z_acc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PositionVelocityAccel
    let len;
    let data = new PositionVelocityAccel(null);
    // Deserialize message field [frame_id]
    data.frame_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_pos]
    data.x_pos = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_pos]
    data.y_pos = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_pos]
    data.z_pos = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_ori]
    data.x_ori = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_ori]
    data.y_ori = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_ori]
    data.z_ori = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [w_ori]
    data.w_ori = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_vel]
    data.x_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_vel]
    data.y_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_vel]
    data.z_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_acc]
    data.x_acc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_acc]
    data.y_acc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_acc]
    data.z_acc = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.frame_id);
    return length + 116;
  }

  static datatype() {
    // Returns string type for a message object
    return 'udp_pkg/PositionVelocityAccel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b1ec75ae739361310f2b2bc1600571ae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string frame_id
    float64 stamp
    float64 x_pos
    float64 y_pos
    float64 z_pos
    float64 x_ori
    float64 y_ori
    float64 z_ori
    float64 w_ori
    float64 x_vel
    float64 y_vel
    float64 z_vel
    float64 x_acc
    float64 y_acc
    float64 z_acc
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PositionVelocityAccel(null);
    if (msg.frame_id !== undefined) {
      resolved.frame_id = msg.frame_id;
    }
    else {
      resolved.frame_id = ''
    }

    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = 0.0
    }

    if (msg.x_pos !== undefined) {
      resolved.x_pos = msg.x_pos;
    }
    else {
      resolved.x_pos = 0.0
    }

    if (msg.y_pos !== undefined) {
      resolved.y_pos = msg.y_pos;
    }
    else {
      resolved.y_pos = 0.0
    }

    if (msg.z_pos !== undefined) {
      resolved.z_pos = msg.z_pos;
    }
    else {
      resolved.z_pos = 0.0
    }

    if (msg.x_ori !== undefined) {
      resolved.x_ori = msg.x_ori;
    }
    else {
      resolved.x_ori = 0.0
    }

    if (msg.y_ori !== undefined) {
      resolved.y_ori = msg.y_ori;
    }
    else {
      resolved.y_ori = 0.0
    }

    if (msg.z_ori !== undefined) {
      resolved.z_ori = msg.z_ori;
    }
    else {
      resolved.z_ori = 0.0
    }

    if (msg.w_ori !== undefined) {
      resolved.w_ori = msg.w_ori;
    }
    else {
      resolved.w_ori = 0.0
    }

    if (msg.x_vel !== undefined) {
      resolved.x_vel = msg.x_vel;
    }
    else {
      resolved.x_vel = 0.0
    }

    if (msg.y_vel !== undefined) {
      resolved.y_vel = msg.y_vel;
    }
    else {
      resolved.y_vel = 0.0
    }

    if (msg.z_vel !== undefined) {
      resolved.z_vel = msg.z_vel;
    }
    else {
      resolved.z_vel = 0.0
    }

    if (msg.x_acc !== undefined) {
      resolved.x_acc = msg.x_acc;
    }
    else {
      resolved.x_acc = 0.0
    }

    if (msg.y_acc !== undefined) {
      resolved.y_acc = msg.y_acc;
    }
    else {
      resolved.y_acc = 0.0
    }

    if (msg.z_acc !== undefined) {
      resolved.z_acc = msg.z_acc;
    }
    else {
      resolved.z_acc = 0.0
    }

    return resolved;
    }
};

module.exports = PositionVelocityAccel;
