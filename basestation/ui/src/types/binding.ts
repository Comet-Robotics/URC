export interface Message {
  imu?: IMUData;
  gps?: GPSData;
  twist?: Twist;
}

export function encodeMessage(message: Message): Uint8Array {
  let bb = popByteBuffer();
  _encodeMessage(message, bb);
  return toUint8Array(bb);
}

function _encodeMessage(message: Message, bb: ByteBuffer): void {
  // optional IMUData imu = 1;
  let $imu = message.imu;
  if ($imu !== undefined) {
    writeVarint32(bb, 10);
    let nested = popByteBuffer();
    _encodeIMUData($imu, nested);
    writeVarint32(bb, nested.limit);
    writeByteBuffer(bb, nested);
    pushByteBuffer(nested);
  }

  // optional GPSData gps = 2;
  let $gps = message.gps;
  if ($gps !== undefined) {
    writeVarint32(bb, 18);
    let nested = popByteBuffer();
    _encodeGPSData($gps, nested);
    writeVarint32(bb, nested.limit);
    writeByteBuffer(bb, nested);
    pushByteBuffer(nested);
  }

  // optional Twist twist = 3;
  let $twist = message.twist;
  if ($twist !== undefined) {
    writeVarint32(bb, 26);
    let nested = popByteBuffer();
    _encodeTwist($twist, nested);
    writeVarint32(bb, nested.limit);
    writeByteBuffer(bb, nested);
    pushByteBuffer(nested);
  }
}

export function decodeMessage(binary: Uint8Array): Message {
  return _decodeMessage(wrapByteBuffer(binary));
}

function _decodeMessage(bb: ByteBuffer): Message {
  let message: Message = {} as any;

  end_of_message: while (!isAtEnd(bb)) {
    let tag = readVarint32(bb);

    switch (tag >>> 3) {
      case 0:
        break end_of_message;

      // optional IMUData imu = 1;
      case 1: {
        let limit = pushTemporaryLength(bb);
        message.imu = _decodeIMUData(bb);
        bb.limit = limit;
        break;
      }

      // optional GPSData gps = 2;
      case 2: {
        let limit = pushTemporaryLength(bb);
        message.gps = _decodeGPSData(bb);
        bb.limit = limit;
        break;
      }

      // optional Twist twist = 3;
      case 3: {
        let limit = pushTemporaryLength(bb);
        message.twist = _decodeTwist(bb);
        bb.limit = limit;
        break;
      }

      default:
        skipUnknownField(bb, tag & 7);
    }
  }

  return message;
}

export interface Twist {
  linear?: Vector3;
  angular?: Vector3;
}

export function encodeTwist(message: Twist): Uint8Array {
  let bb = popByteBuffer();
  _encodeTwist(message, bb);
  return toUint8Array(bb);
}

function _encodeTwist(message: Twist, bb: ByteBuffer): void {
  // optional Vector3 linear = 1;
  let $linear = message.linear;
  if ($linear !== undefined) {
    writeVarint32(bb, 10);
    let nested = popByteBuffer();
    _encodeVector3($linear, nested);
    writeVarint32(bb, nested.limit);
    writeByteBuffer(bb, nested);
    pushByteBuffer(nested);
  }

  // optional Vector3 angular = 2;
  let $angular = message.angular;
  if ($angular !== undefined) {
    writeVarint32(bb, 18);
    let nested = popByteBuffer();
    _encodeVector3($angular, nested);
    writeVarint32(bb, nested.limit);
    writeByteBuffer(bb, nested);
    pushByteBuffer(nested);
  }
}

export function decodeTwist(binary: Uint8Array): Twist {
  return _decodeTwist(wrapByteBuffer(binary));
}

function _decodeTwist(bb: ByteBuffer): Twist {
  let message: Twist = {} as any;

  end_of_message: while (!isAtEnd(bb)) {
    let tag = readVarint32(bb);

    switch (tag >>> 3) {
      case 0:
        break end_of_message;

      // optional Vector3 linear = 1;
      case 1: {
        let limit = pushTemporaryLength(bb);
        message.linear = _decodeVector3(bb);
        bb.limit = limit;
        break;
      }

      // optional Vector3 angular = 2;
      case 2: {
        let limit = pushTemporaryLength(bb);
        message.angular = _decodeVector3(bb);
        bb.limit = limit;
        break;
      }

      default:
        skipUnknownField(bb, tag & 7);
    }
  }

  return message;
}

export interface GPSData {
  longitude?: number;
  latitude?: number;
  altitude?: number;
  ground_speed?: number;
  satellites?: number;
  mode_indicator?: number;
  separation?: number;
  true_course?: number;
  true_course_magnetic?: number;
  dilution?: number;
  utc_time?: number;
}

export function encodeGPSData(message: GPSData): Uint8Array {
  let bb = popByteBuffer();
  _encodeGPSData(message, bb);
  return toUint8Array(bb);
}

function _encodeGPSData(message: GPSData, bb: ByteBuffer): void {
  // optional double longitude = 1;
  let $longitude = message.longitude;
  if ($longitude !== undefined) {
    writeVarint32(bb, 9);
    writeDouble(bb, $longitude);
  }

  // optional double latitude = 2;
  let $latitude = message.latitude;
  if ($latitude !== undefined) {
    writeVarint32(bb, 17);
    writeDouble(bb, $latitude);
  }

  // optional double altitude = 3;
  let $altitude = message.altitude;
  if ($altitude !== undefined) {
    writeVarint32(bb, 25);
    writeDouble(bb, $altitude);
  }

  // optional float ground_speed = 4;
  let $ground_speed = message.ground_speed;
  if ($ground_speed !== undefined) {
    writeVarint32(bb, 37);
    writeFloat(bb, $ground_speed);
  }

  // optional uint32 satellites = 5;
  let $satellites = message.satellites;
  if ($satellites !== undefined) {
    writeVarint32(bb, 40);
    writeVarint32(bb, $satellites);
  }

  // optional uint32 mode_indicator = 6;
  let $mode_indicator = message.mode_indicator;
  if ($mode_indicator !== undefined) {
    writeVarint32(bb, 48);
    writeVarint32(bb, $mode_indicator);
  }

  // optional float separation = 7;
  let $separation = message.separation;
  if ($separation !== undefined) {
    writeVarint32(bb, 61);
    writeFloat(bb, $separation);
  }

  // optional float true_course = 8;
  let $true_course = message.true_course;
  if ($true_course !== undefined) {
    writeVarint32(bb, 69);
    writeFloat(bb, $true_course);
  }

  // optional float true_course_magnetic = 9;
  let $true_course_magnetic = message.true_course_magnetic;
  if ($true_course_magnetic !== undefined) {
    writeVarint32(bb, 77);
    writeFloat(bb, $true_course_magnetic);
  }

  // optional float dilution = 10;
  let $dilution = message.dilution;
  if ($dilution !== undefined) {
    writeVarint32(bb, 85);
    writeFloat(bb, $dilution);
  }

  // optional uint32 utc_time = 11;
  let $utc_time = message.utc_time;
  if ($utc_time !== undefined) {
    writeVarint32(bb, 88);
    writeVarint32(bb, $utc_time);
  }
}

export function decodeGPSData(binary: Uint8Array): GPSData {
  return _decodeGPSData(wrapByteBuffer(binary));
}

function _decodeGPSData(bb: ByteBuffer): GPSData {
  let message: GPSData = {} as any;

  end_of_message: while (!isAtEnd(bb)) {
    let tag = readVarint32(bb);

    switch (tag >>> 3) {
      case 0:
        break end_of_message;

      // optional double longitude = 1;
      case 1: {
        message.longitude = readDouble(bb);
        break;
      }

      // optional double latitude = 2;
      case 2: {
        message.latitude = readDouble(bb);
        break;
      }

      // optional double altitude = 3;
      case 3: {
        message.altitude = readDouble(bb);
        break;
      }

      // optional float ground_speed = 4;
      case 4: {
        message.ground_speed = readFloat(bb);
        break;
      }

      // optional uint32 satellites = 5;
      case 5: {
        message.satellites = readVarint32(bb) >>> 0;
        break;
      }

      // optional uint32 mode_indicator = 6;
      case 6: {
        message.mode_indicator = readVarint32(bb) >>> 0;
        break;
      }

      // optional float separation = 7;
      case 7: {
        message.separation = readFloat(bb);
        break;
      }

      // optional float true_course = 8;
      case 8: {
        message.true_course = readFloat(bb);
        break;
      }

      // optional float true_course_magnetic = 9;
      case 9: {
        message.true_course_magnetic = readFloat(bb);
        break;
      }

      // optional float dilution = 10;
      case 10: {
        message.dilution = readFloat(bb);
        break;
      }

      // optional uint32 utc_time = 11;
      case 11: {
        message.utc_time = readVarint32(bb) >>> 0;
        break;
      }

      default:
        skipUnknownField(bb, tag & 7);
    }
  }

  return message;
}

export interface IMUData {
  header?: Header;
  orientation?: Quaternion;
  orientation_covariance?: number[];
  angular_velocity?: Vector3;
  angular_velocity_covariance?: number[];
  linear_acceleration?: Vector3;
  linear_acceleration_covariance?: number[];
}

export function encodeIMUData(message: IMUData): Uint8Array {
  let bb = popByteBuffer();
  _encodeIMUData(message, bb);
  return toUint8Array(bb);
}

function _encodeIMUData(message: IMUData, bb: ByteBuffer): void {
  // optional Header header = 1;
  let $header = message.header;
  if ($header !== undefined) {
    writeVarint32(bb, 10);
    let nested = popByteBuffer();
    _encodeHeader($header, nested);
    writeVarint32(bb, nested.limit);
    writeByteBuffer(bb, nested);
    pushByteBuffer(nested);
  }

  // optional Quaternion orientation = 2;
  let $orientation = message.orientation;
  if ($orientation !== undefined) {
    writeVarint32(bb, 18);
    let nested = popByteBuffer();
    _encodeQuaternion($orientation, nested);
    writeVarint32(bb, nested.limit);
    writeByteBuffer(bb, nested);
    pushByteBuffer(nested);
  }

  // repeated double orientation_covariance = 3;
  let array$orientation_covariance = message.orientation_covariance;
  if (array$orientation_covariance !== undefined) {
    let packed = popByteBuffer();
    for (let value of array$orientation_covariance) {
      writeDouble(packed, value);
    }
    writeVarint32(bb, 26);
    writeVarint32(bb, packed.offset);
    writeByteBuffer(bb, packed);
    pushByteBuffer(packed);
  }

  // optional Vector3 angular_velocity = 4;
  let $angular_velocity = message.angular_velocity;
  if ($angular_velocity !== undefined) {
    writeVarint32(bb, 34);
    let nested = popByteBuffer();
    _encodeVector3($angular_velocity, nested);
    writeVarint32(bb, nested.limit);
    writeByteBuffer(bb, nested);
    pushByteBuffer(nested);
  }

  // repeated double angular_velocity_covariance = 5;
  let array$angular_velocity_covariance = message.angular_velocity_covariance;
  if (array$angular_velocity_covariance !== undefined) {
    let packed = popByteBuffer();
    for (let value of array$angular_velocity_covariance) {
      writeDouble(packed, value);
    }
    writeVarint32(bb, 42);
    writeVarint32(bb, packed.offset);
    writeByteBuffer(bb, packed);
    pushByteBuffer(packed);
  }

  // optional Vector3 linear_acceleration = 6;
  let $linear_acceleration = message.linear_acceleration;
  if ($linear_acceleration !== undefined) {
    writeVarint32(bb, 50);
    let nested = popByteBuffer();
    _encodeVector3($linear_acceleration, nested);
    writeVarint32(bb, nested.limit);
    writeByteBuffer(bb, nested);
    pushByteBuffer(nested);
  }

  // repeated double linear_acceleration_covariance = 7;
  let array$linear_acceleration_covariance = message.linear_acceleration_covariance;
  if (array$linear_acceleration_covariance !== undefined) {
    let packed = popByteBuffer();
    for (let value of array$linear_acceleration_covariance) {
      writeDouble(packed, value);
    }
    writeVarint32(bb, 58);
    writeVarint32(bb, packed.offset);
    writeByteBuffer(bb, packed);
    pushByteBuffer(packed);
  }
}

export function decodeIMUData(binary: Uint8Array): IMUData {
  return _decodeIMUData(wrapByteBuffer(binary));
}

function _decodeIMUData(bb: ByteBuffer): IMUData {
  let message: IMUData = {} as any;

  end_of_message: while (!isAtEnd(bb)) {
    let tag = readVarint32(bb);

    switch (tag >>> 3) {
      case 0:
        break end_of_message;

      // optional Header header = 1;
      case 1: {
        let limit = pushTemporaryLength(bb);
        message.header = _decodeHeader(bb);
        bb.limit = limit;
        break;
      }

      // optional Quaternion orientation = 2;
      case 2: {
        let limit = pushTemporaryLength(bb);
        message.orientation = _decodeQuaternion(bb);
        bb.limit = limit;
        break;
      }

      // repeated double orientation_covariance = 3;
      case 3: {
        let values = message.orientation_covariance || (message.orientation_covariance = []);
        if ((tag & 7) === 2) {
          let outerLimit = pushTemporaryLength(bb);
          while (!isAtEnd(bb)) {
            values.push(readDouble(bb));
          }
          bb.limit = outerLimit;
        } else {
          values.push(readDouble(bb));
        }
        break;
      }

      // optional Vector3 angular_velocity = 4;
      case 4: {
        let limit = pushTemporaryLength(bb);
        message.angular_velocity = _decodeVector3(bb);
        bb.limit = limit;
        break;
      }

      // repeated double angular_velocity_covariance = 5;
      case 5: {
        let values = message.angular_velocity_covariance || (message.angular_velocity_covariance = []);
        if ((tag & 7) === 2) {
          let outerLimit = pushTemporaryLength(bb);
          while (!isAtEnd(bb)) {
            values.push(readDouble(bb));
          }
          bb.limit = outerLimit;
        } else {
          values.push(readDouble(bb));
        }
        break;
      }

      // optional Vector3 linear_acceleration = 6;
      case 6: {
        let limit = pushTemporaryLength(bb);
        message.linear_acceleration = _decodeVector3(bb);
        bb.limit = limit;
        break;
      }

      // repeated double linear_acceleration_covariance = 7;
      case 7: {
        let values = message.linear_acceleration_covariance || (message.linear_acceleration_covariance = []);
        if ((tag & 7) === 2) {
          let outerLimit = pushTemporaryLength(bb);
          while (!isAtEnd(bb)) {
            values.push(readDouble(bb));
          }
          bb.limit = outerLimit;
        } else {
          values.push(readDouble(bb));
        }
        break;
      }

      default:
        skipUnknownField(bb, tag & 7);
    }
  }

  return message;
}

export interface Quaternion {
  x?: number;
  y?: number;
  z?: number;
  w?: number;
}

export function encodeQuaternion(message: Quaternion): Uint8Array {
  let bb = popByteBuffer();
  _encodeQuaternion(message, bb);
  return toUint8Array(bb);
}

function _encodeQuaternion(message: Quaternion, bb: ByteBuffer): void {
  // optional double x = 1;
  let $x = message.x;
  if ($x !== undefined) {
    writeVarint32(bb, 9);
    writeDouble(bb, $x);
  }

  // optional double y = 2;
  let $y = message.y;
  if ($y !== undefined) {
    writeVarint32(bb, 17);
    writeDouble(bb, $y);
  }

  // optional double z = 3;
  let $z = message.z;
  if ($z !== undefined) {
    writeVarint32(bb, 25);
    writeDouble(bb, $z);
  }

  // optional double w = 4;
  let $w = message.w;
  if ($w !== undefined) {
    writeVarint32(bb, 33);
    writeDouble(bb, $w);
  }
}

export function decodeQuaternion(binary: Uint8Array): Quaternion {
  return _decodeQuaternion(wrapByteBuffer(binary));
}

function _decodeQuaternion(bb: ByteBuffer): Quaternion {
  let message: Quaternion = {} as any;

  end_of_message: while (!isAtEnd(bb)) {
    let tag = readVarint32(bb);

    switch (tag >>> 3) {
      case 0:
        break end_of_message;

      // optional double x = 1;
      case 1: {
        message.x = readDouble(bb);
        break;
      }

      // optional double y = 2;
      case 2: {
        message.y = readDouble(bb);
        break;
      }

      // optional double z = 3;
      case 3: {
        message.z = readDouble(bb);
        break;
      }

      // optional double w = 4;
      case 4: {
        message.w = readDouble(bb);
        break;
      }

      default:
        skipUnknownField(bb, tag & 7);
    }
  }

  return message;
}

export interface Vector3 {
  x?: number;
  y?: number;
  z?: number;
}

export function encodeVector3(message: Vector3): Uint8Array {
  let bb = popByteBuffer();
  _encodeVector3(message, bb);
  return toUint8Array(bb);
}

function _encodeVector3(message: Vector3, bb: ByteBuffer): void {
  // optional double x = 1;
  let $x = message.x;
  if ($x !== undefined) {
    writeVarint32(bb, 9);
    writeDouble(bb, $x);
  }

  // optional double y = 2;
  let $y = message.y;
  if ($y !== undefined) {
    writeVarint32(bb, 17);
    writeDouble(bb, $y);
  }

  // optional double z = 3;
  let $z = message.z;
  if ($z !== undefined) {
    writeVarint32(bb, 25);
    writeDouble(bb, $z);
  }
}

export function decodeVector3(binary: Uint8Array): Vector3 {
  return _decodeVector3(wrapByteBuffer(binary));
}

function _decodeVector3(bb: ByteBuffer): Vector3 {
  let message: Vector3 = {} as any;

  end_of_message: while (!isAtEnd(bb)) {
    let tag = readVarint32(bb);

    switch (tag >>> 3) {
      case 0:
        break end_of_message;

      // optional double x = 1;
      case 1: {
        message.x = readDouble(bb);
        break;
      }

      // optional double y = 2;
      case 2: {
        message.y = readDouble(bb);
        break;
      }

      // optional double z = 3;
      case 3: {
        message.z = readDouble(bb);
        break;
      }

      default:
        skipUnknownField(bb, tag & 7);
    }
  }

  return message;
}

export interface Header {
  seq?: number;
  stamp?: string;
  frame_id?: string;
}

export function encodeHeader(message: Header): Uint8Array {
  let bb = popByteBuffer();
  _encodeHeader(message, bb);
  return toUint8Array(bb);
}

function _encodeHeader(message: Header, bb: ByteBuffer): void {
  // optional uint32 seq = 1;
  let $seq = message.seq;
  if ($seq !== undefined) {
    writeVarint32(bb, 8);
    writeVarint32(bb, $seq);
  }

  // optional string stamp = 2;
  let $stamp = message.stamp;
  if ($stamp !== undefined) {
    writeVarint32(bb, 18);
    writeString(bb, $stamp);
  }

  // optional string frame_id = 3;
  let $frame_id = message.frame_id;
  if ($frame_id !== undefined) {
    writeVarint32(bb, 26);
    writeString(bb, $frame_id);
  }
}

export function decodeHeader(binary: Uint8Array): Header {
  return _decodeHeader(wrapByteBuffer(binary));
}

function _decodeHeader(bb: ByteBuffer): Header {
  let message: Header = {} as any;

  end_of_message: while (!isAtEnd(bb)) {
    let tag = readVarint32(bb);

    switch (tag >>> 3) {
      case 0:
        break end_of_message;

      // optional uint32 seq = 1;
      case 1: {
        message.seq = readVarint32(bb) >>> 0;
        break;
      }

      // optional string stamp = 2;
      case 2: {
        message.stamp = readString(bb, readVarint32(bb));
        break;
      }

      // optional string frame_id = 3;
      case 3: {
        message.frame_id = readString(bb, readVarint32(bb));
        break;
      }

      default:
        skipUnknownField(bb, tag & 7);
    }
  }

  return message;
}

export interface Long {
  low: number;
  high: number;
  unsigned: boolean;
}

interface ByteBuffer {
  bytes: Uint8Array;
  offset: number;
  limit: number;
}

function pushTemporaryLength(bb: ByteBuffer): number {
  let length = readVarint32(bb);
  let limit = bb.limit;
  bb.limit = bb.offset + length;
  return limit;
}

function skipUnknownField(bb: ByteBuffer, type: number): void {
  switch (type) {
    case 0: while (readByte(bb) & 0x80) { } break;
    case 2: skip(bb, readVarint32(bb)); break;
    case 5: skip(bb, 4); break;
    case 1: skip(bb, 8); break;
    default: throw new Error("Unimplemented type: " + type);
  }
}

function stringToLong(value: string): Long {
  return {
    low: value.charCodeAt(0) | (value.charCodeAt(1) << 16),
    high: value.charCodeAt(2) | (value.charCodeAt(3) << 16),
    unsigned: false,
  };
}

function longToString(value: Long): string {
  let low = value.low;
  let high = value.high;
  return String.fromCharCode(
    low & 0xFFFF,
    low >>> 16,
    high & 0xFFFF,
    high >>> 16);
}

// The code below was modified from https://github.com/protobufjs/bytebuffer.js
// which is under the Apache License 2.0.

let f32 = new Float32Array(1);
let f32_u8 = new Uint8Array(f32.buffer);

let f64 = new Float64Array(1);
let f64_u8 = new Uint8Array(f64.buffer);

function intToLong(value: number): Long {
  value |= 0;
  return {
    low: value,
    high: value >> 31,
    unsigned: value >= 0,
  };
}

let bbStack: ByteBuffer[] = [];

function popByteBuffer(): ByteBuffer {
  const bb = bbStack.pop();
  if (!bb) return { bytes: new Uint8Array(64), offset: 0, limit: 0 };
  bb.offset = bb.limit = 0;
  return bb;
}

function pushByteBuffer(bb: ByteBuffer): void {
  bbStack.push(bb);
}

function wrapByteBuffer(bytes: Uint8Array): ByteBuffer {
  return { bytes, offset: 0, limit: bytes.length };
}

function toUint8Array(bb: ByteBuffer): Uint8Array {
  let bytes = bb.bytes;
  let limit = bb.limit;
  return bytes.length === limit ? bytes : bytes.subarray(0, limit);
}

function skip(bb: ByteBuffer, offset: number): void {
  if (bb.offset + offset > bb.limit) {
    throw new Error('Skip past limit');
  }
  bb.offset += offset;
}

function isAtEnd(bb: ByteBuffer): boolean {
  return bb.offset >= bb.limit;
}

function grow(bb: ByteBuffer, count: number): number {
  let bytes = bb.bytes;
  let offset = bb.offset;
  let limit = bb.limit;
  let finalOffset = offset + count;
  if (finalOffset > bytes.length) {
    let newBytes = new Uint8Array(finalOffset * 2);
    newBytes.set(bytes);
    bb.bytes = newBytes;
  }
  bb.offset = finalOffset;
  if (finalOffset > limit) {
    bb.limit = finalOffset;
  }
  return offset;
}

function advance(bb: ByteBuffer, count: number): number {
  let offset = bb.offset;
  if (offset + count > bb.limit) {
    throw new Error('Read past limit');
  }
  bb.offset += count;
  return offset;
}

function readBytes(bb: ByteBuffer, count: number): Uint8Array {
  let offset = advance(bb, count);
  return bb.bytes.subarray(offset, offset + count);
}

function writeBytes(bb: ByteBuffer, buffer: Uint8Array): void {
  let offset = grow(bb, buffer.length);
  bb.bytes.set(buffer, offset);
}

function readString(bb: ByteBuffer, count: number): string {
  // Sadly a hand-coded UTF8 decoder is much faster than subarray+TextDecoder in V8
  let offset = advance(bb, count);
  let fromCharCode = String.fromCharCode;
  let bytes = bb.bytes;
  let invalid = '\uFFFD';
  let text = '';

  for (let i = 0; i < count; i++) {
    let c1 = bytes[i + offset], c2: number, c3: number, c4: number, c: number;

    // 1 byte
    if ((c1 & 0x80) === 0) {
      text += fromCharCode(c1);
    }

    // 2 bytes
    else if ((c1 & 0xE0) === 0xC0) {
      if (i + 1 >= count) text += invalid;
      else {
        c2 = bytes[i + offset + 1];
        if ((c2 & 0xC0) !== 0x80) text += invalid;
        else {
          c = ((c1 & 0x1F) << 6) | (c2 & 0x3F);
          if (c < 0x80) text += invalid;
          else {
            text += fromCharCode(c);
            i++;
          }
        }
      }
    }

    // 3 bytes
    else if ((c1 & 0xF0) == 0xE0) {
      if (i + 2 >= count) text += invalid;
      else {
        c2 = bytes[i + offset + 1];
        c3 = bytes[i + offset + 2];
        if (((c2 | (c3 << 8)) & 0xC0C0) !== 0x8080) text += invalid;
        else {
          c = ((c1 & 0x0F) << 12) | ((c2 & 0x3F) << 6) | (c3 & 0x3F);
          if (c < 0x0800 || (c >= 0xD800 && c <= 0xDFFF)) text += invalid;
          else {
            text += fromCharCode(c);
            i += 2;
          }
        }
      }
    }

    // 4 bytes
    else if ((c1 & 0xF8) == 0xF0) {
      if (i + 3 >= count) text += invalid;
      else {
        c2 = bytes[i + offset + 1];
        c3 = bytes[i + offset + 2];
        c4 = bytes[i + offset + 3];
        if (((c2 | (c3 << 8) | (c4 << 16)) & 0xC0C0C0) !== 0x808080) text += invalid;
        else {
          c = ((c1 & 0x07) << 0x12) | ((c2 & 0x3F) << 0x0C) | ((c3 & 0x3F) << 0x06) | (c4 & 0x3F);
          if (c < 0x10000 || c > 0x10FFFF) text += invalid;
          else {
            c -= 0x10000;
            text += fromCharCode((c >> 10) + 0xD800, (c & 0x3FF) + 0xDC00);
            i += 3;
          }
        }
      }
    }

    else text += invalid;
  }

  return text;
}

function writeString(bb: ByteBuffer, text: string): void {
  // Sadly a hand-coded UTF8 encoder is much faster than TextEncoder+set in V8
  let n = text.length;
  let byteCount = 0;

  // Write the byte count first
  for (let i = 0; i < n; i++) {
    let c = text.charCodeAt(i);
    if (c >= 0xD800 && c <= 0xDBFF && i + 1 < n) {
      c = (c << 10) + text.charCodeAt(++i) - 0x35FDC00;
    }
    byteCount += c < 0x80 ? 1 : c < 0x800 ? 2 : c < 0x10000 ? 3 : 4;
  }
  writeVarint32(bb, byteCount);

  let offset = grow(bb, byteCount);
  let bytes = bb.bytes;

  // Then write the bytes
  for (let i = 0; i < n; i++) {
    let c = text.charCodeAt(i);
    if (c >= 0xD800 && c <= 0xDBFF && i + 1 < n) {
      c = (c << 10) + text.charCodeAt(++i) - 0x35FDC00;
    }
    if (c < 0x80) {
      bytes[offset++] = c;
    } else {
      if (c < 0x800) {
        bytes[offset++] = ((c >> 6) & 0x1F) | 0xC0;
      } else {
        if (c < 0x10000) {
          bytes[offset++] = ((c >> 12) & 0x0F) | 0xE0;
        } else {
          bytes[offset++] = ((c >> 18) & 0x07) | 0xF0;
          bytes[offset++] = ((c >> 12) & 0x3F) | 0x80;
        }
        bytes[offset++] = ((c >> 6) & 0x3F) | 0x80;
      }
      bytes[offset++] = (c & 0x3F) | 0x80;
    }
  }
}

function writeByteBuffer(bb: ByteBuffer, buffer: ByteBuffer): void {
  let offset = grow(bb, buffer.limit);
  let from = bb.bytes;
  let to = buffer.bytes;

  // This for loop is much faster than subarray+set on V8
  for (let i = 0, n = buffer.limit; i < n; i++) {
    from[i + offset] = to[i];
  }
}

function readByte(bb: ByteBuffer): number {
  return bb.bytes[advance(bb, 1)];
}

function writeByte(bb: ByteBuffer, value: number): void {
  let offset = grow(bb, 1);
  bb.bytes[offset] = value;
}

function readFloat(bb: ByteBuffer): number {
  let offset = advance(bb, 4);
  let bytes = bb.bytes;

  // Manual copying is much faster than subarray+set in V8
  f32_u8[0] = bytes[offset++];
  f32_u8[1] = bytes[offset++];
  f32_u8[2] = bytes[offset++];
  f32_u8[3] = bytes[offset++];
  return f32[0];
}

function writeFloat(bb: ByteBuffer, value: number): void {
  let offset = grow(bb, 4);
  let bytes = bb.bytes;
  f32[0] = value;

  // Manual copying is much faster than subarray+set in V8
  bytes[offset++] = f32_u8[0];
  bytes[offset++] = f32_u8[1];
  bytes[offset++] = f32_u8[2];
  bytes[offset++] = f32_u8[3];
}

function readDouble(bb: ByteBuffer): number {
  let offset = advance(bb, 8);
  let bytes = bb.bytes;

  // Manual copying is much faster than subarray+set in V8
  f64_u8[0] = bytes[offset++];
  f64_u8[1] = bytes[offset++];
  f64_u8[2] = bytes[offset++];
  f64_u8[3] = bytes[offset++];
  f64_u8[4] = bytes[offset++];
  f64_u8[5] = bytes[offset++];
  f64_u8[6] = bytes[offset++];
  f64_u8[7] = bytes[offset++];
  return f64[0];
}

function writeDouble(bb: ByteBuffer, value: number): void {
  let offset = grow(bb, 8);
  let bytes = bb.bytes;
  f64[0] = value;

  // Manual copying is much faster than subarray+set in V8
  bytes[offset++] = f64_u8[0];
  bytes[offset++] = f64_u8[1];
  bytes[offset++] = f64_u8[2];
  bytes[offset++] = f64_u8[3];
  bytes[offset++] = f64_u8[4];
  bytes[offset++] = f64_u8[5];
  bytes[offset++] = f64_u8[6];
  bytes[offset++] = f64_u8[7];
}

function readInt32(bb: ByteBuffer): number {
  let offset = advance(bb, 4);
  let bytes = bb.bytes;
  return (
    bytes[offset] |
    (bytes[offset + 1] << 8) |
    (bytes[offset + 2] << 16) |
    (bytes[offset + 3] << 24)
  );
}

function writeInt32(bb: ByteBuffer, value: number): void {
  let offset = grow(bb, 4);
  let bytes = bb.bytes;
  bytes[offset] = value;
  bytes[offset + 1] = value >> 8;
  bytes[offset + 2] = value >> 16;
  bytes[offset + 3] = value >> 24;
}

function readInt64(bb: ByteBuffer, unsigned: boolean): Long {
  return {
    low: readInt32(bb),
    high: readInt32(bb),
    unsigned,
  };
}

function writeInt64(bb: ByteBuffer, value: Long): void {
  writeInt32(bb, value.low);
  writeInt32(bb, value.high);
}

function readVarint32(bb: ByteBuffer): number {
  let c = 0;
  let value = 0;
  let b: number;
  do {
    b = readByte(bb);
    if (c < 32) value |= (b & 0x7F) << c;
    c += 7;
  } while (b & 0x80);
  return value;
}

function writeVarint32(bb: ByteBuffer, value: number): void {
  value >>>= 0;
  while (value >= 0x80) {
    writeByte(bb, (value & 0x7f) | 0x80);
    value >>>= 7;
  }
  writeByte(bb, value);
}

function readVarint64(bb: ByteBuffer, unsigned: boolean): Long {
  let part0 = 0;
  let part1 = 0;
  let part2 = 0;
  let b: number;

  b = readByte(bb); part0 = (b & 0x7F); if (b & 0x80) {
    b = readByte(bb); part0 |= (b & 0x7F) << 7; if (b & 0x80) {
      b = readByte(bb); part0 |= (b & 0x7F) << 14; if (b & 0x80) {
        b = readByte(bb); part0 |= (b & 0x7F) << 21; if (b & 0x80) {

          b = readByte(bb); part1 = (b & 0x7F); if (b & 0x80) {
            b = readByte(bb); part1 |= (b & 0x7F) << 7; if (b & 0x80) {
              b = readByte(bb); part1 |= (b & 0x7F) << 14; if (b & 0x80) {
                b = readByte(bb); part1 |= (b & 0x7F) << 21; if (b & 0x80) {

                  b = readByte(bb); part2 = (b & 0x7F); if (b & 0x80) {
                    b = readByte(bb); part2 |= (b & 0x7F) << 7;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  return {
    low: part0 | (part1 << 28),
    high: (part1 >>> 4) | (part2 << 24),
    unsigned,
  };
}

function writeVarint64(bb: ByteBuffer, value: Long): void {
  let part0 = value.low >>> 0;
  let part1 = ((value.low >>> 28) | (value.high << 4)) >>> 0;
  let part2 = value.high >>> 24;

  // ref: src/google/protobuf/io/coded_stream.cc
  let size =
    part2 === 0 ?
      part1 === 0 ?
        part0 < 1 << 14 ?
          part0 < 1 << 7 ? 1 : 2 :
          part0 < 1 << 21 ? 3 : 4 :
        part1 < 1 << 14 ?
          part1 < 1 << 7 ? 5 : 6 :
          part1 < 1 << 21 ? 7 : 8 :
      part2 < 1 << 7 ? 9 : 10;

  let offset = grow(bb, size);
  let bytes = bb.bytes;

  switch (size) {
    case 10: bytes[offset + 9] = (part2 >>> 7) & 0x01;
    case 9: bytes[offset + 8] = size !== 9 ? part2 | 0x80 : part2 & 0x7F;
    case 8: bytes[offset + 7] = size !== 8 ? (part1 >>> 21) | 0x80 : (part1 >>> 21) & 0x7F;
    case 7: bytes[offset + 6] = size !== 7 ? (part1 >>> 14) | 0x80 : (part1 >>> 14) & 0x7F;
    case 6: bytes[offset + 5] = size !== 6 ? (part1 >>> 7) | 0x80 : (part1 >>> 7) & 0x7F;
    case 5: bytes[offset + 4] = size !== 5 ? part1 | 0x80 : part1 & 0x7F;
    case 4: bytes[offset + 3] = size !== 4 ? (part0 >>> 21) | 0x80 : (part0 >>> 21) & 0x7F;
    case 3: bytes[offset + 2] = size !== 3 ? (part0 >>> 14) | 0x80 : (part0 >>> 14) & 0x7F;
    case 2: bytes[offset + 1] = size !== 2 ? (part0 >>> 7) | 0x80 : (part0 >>> 7) & 0x7F;
    case 1: bytes[offset] = size !== 1 ? part0 | 0x80 : part0 & 0x7F;
  }
}

function readVarint32ZigZag(bb: ByteBuffer): number {
  let value = readVarint32(bb);

  // ref: src/google/protobuf/wire_format_lite.h
  return (value >>> 1) ^ -(value & 1);
}

function writeVarint32ZigZag(bb: ByteBuffer, value: number): void {
  // ref: src/google/protobuf/wire_format_lite.h
  writeVarint32(bb, (value << 1) ^ (value >> 31));
}

function readVarint64ZigZag(bb: ByteBuffer): Long {
  let value = readVarint64(bb, /* unsigned */ false);
  let low = value.low;
  let high = value.high;
  let flip = -(low & 1);

  // ref: src/google/protobuf/wire_format_lite.h
  return {
    low: ((low >>> 1) | (high << 31)) ^ flip,
    high: (high >>> 1) ^ flip,
    unsigned: false,
  };
}

function writeVarint64ZigZag(bb: ByteBuffer, value: Long): void {
  let low = value.low;
  let high = value.high;
  let flip = high >> 31;

  // ref: src/google/protobuf/wire_format_lite.h
  writeVarint64(bb, {
    low: (low << 1) ^ flip,
    high: ((high << 1) | (low >>> 31)) ^ flip,
    unsigned: false,
  });
}
