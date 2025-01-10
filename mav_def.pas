{********************************************************}
{                                                        }
{       Common functions and definitions                 }
{                                                        }
{       Copyright (c) 2024-2025    Helmut Elsner         }
{                                                        }
{       Compiler: FPC 3.2.3   /    Lazarus 3.7           }
{                                                        }
{ Pascal programmers tend to plan ahead, they think      }
{ before they type. We type a lot because of Pascal      }
{ verboseness, but usually our code is right from the    }
{ start. We end up typing less because we fix less bugs. }
{           [Jorge Aldo G. de F. Junior]                 }
{********************************************************}

(*
This source is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free
Software Foundation; either version 2 of the License, or (at your option)
any later version.

This code is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details.

A copy of the GNU General Public License is available on the World Wide Web
at <http://www.gnu.org/copyleft/gpl.html>. You can also obtain it by writing
to the Free Software Foundation, Inc., 51 Franklin Street - Fifth Floor,
Boston, MA 02110-1335, USA.

*******************************************************************************)



(* Helpful information:

https://mavlink.io/en/guide/serialization.html
https://github.com/mavlink/c_library_v2/tree/master/common

MAVLINK_LITTLE_ENDIAN
*)

unit mav_def;                                 {MAVlink definitions and variables}

{$mode objfpc}{$H+}

interface

uses
  sysutils, math;

const
  MAVsatCount=19;                                        {20 sats means 0..19}
  LengthFixPartBC=6;
  LengthFixPartFD=10;
  LengthFixPartFE=8;
  MagicBC=$BC;  {Flight controller <-> GUI, Sensor files, nested in CGO3+ messages}
  MagicFD=$FD;  {PX4 based Yuneec drones (MAVlink V2)}
  MagicFE=$FE;  {Flight controller <-> Gimbal <-> camera CGO3+}
  YGCsysID=10;
  MilliSecondsPerDay=86400000;
  max8=255;
  max16=65535;
  maxLenMAVmsg=280;

// Parameter names
  pGeoFence='FENCE_RADIUS';
  pHeightLimit='FENCE_ALT_MAX';


{The ISO 8601 standard provides an extensive set of practical well-designed
 formats for expressing date-time values as text. These formats are easy to
 parse by machine as well as easy to read by humans across cultures.

These include:
    Date-only: 2019-01-23
    Moment in UTC: 2019-01-23T12:34:56.123456Z
    Moment with offset-from-UTC: 2019-01-23T18:04:56.123456+05:30
    Week of week-based-year: 2019-W23
    Ordinal date (1st to 366th day of year): 2019-234   }

  timefull='yyyy-mm-dd hh:nn:ss';
  timezzz='nn:ss.zzz';
  floatformat8='0.00000000';
  floatformat3='0.000';
  floatformat2='0.00';
  floatformat1='0.0';

  rsNotSpecified='Not specified';
  rsNotProvided='Not provided';
  rsInvalid='Invalid';
  rsUnknown='Unkown';

  H480_HWflags=$00A0FC2F;
  H480RS_HWflags=$02A0FC6F;

type
  TMAVmessage = record
    msglength, msgid: uint16;
    msgbytes: array[0..maxLenMAVmsg] of byte;
    sysid, targetid, msgformat: byte;
    valid: boolean;
  end;

type

// Add here some data records for different analysis cases

  TGPSdata = record
// GPS data from GPS_raw_int (24)
    boottime, timeUTC, yuneectime: TDateTime;
    lat, lon: int32;               {[degE7] WGS84, EGM96 ellipsoid}
    altMSL, alt_rel: int32;        {[mm] Altitude. Positive for up}
    eph, epv, vel, cog, hdg: uint16;
    sats_inuse, fix_type: byte;
    alt_ellipsoid: int32;          {[mm] Altitude (above WGS84, EGM96 ellipsoid)}
    h_acc, v_acc, vel_acc: uint32; {[mm] uncertainty}
    hdg_acc: uint32;               {[degE5] Heading / track uncertainty}
    yaw: uint32; {[cdeg] Yaw in earth frame from north. Use 36000 for north.}
    vx, vy, vz: int16;
    load, voltage: uint16;
    gps_present, sensors_OK: boolean;

// Satellites data from GPS_status (25)
    sats_visible: byte;
    sat_prn:       array[0..MAVsatCount] of byte;        {Global satellite ID}
    sat_used:      array[0..MAVsatCount] of byte;        {0 not, 1 used}
    sat_elevation: array[0..MAVsatCount] of byte;        {[deg] Elevation (0: right on top of receiver, 90: on the horizon) of satellite}
    sat_azimuth:   array[0..MAVsatCount] of byte;        {[deg] Direction of satellite}
    sat_snr:       array[0..MAVsatCount] of byte;        {[dB] Signal to noise ratio of satellite}
  end;

type
  TAttitudeData = record
    boottime: TDateTime;
    altMSL, alt_rel: single;                             {[m] Altitude. Positive for up}
    posx, posy, posz: single;                            {m}
    roll, pitch, yaw, yaw_abs: single;                   {deg}
    rollspeed, pitchspeed, yawspeed: single;             {rad/s}
    vx, vy, vz, airspeed, groundspeed, climbrate: single; {m/s}
    heading, throttle: int16;                            {°, %}
    velocity_variance, pos_horiz_variance, pos_vert_variance: single;
    compass_variance, terrain_alt_variance: single;
    EKFstatus: uint16;
  end;

type
  THWstatusData = record
    boottime: TDateTime;
    rssi: byte;
    load, droprate, batt: UInt16;                        {% or unitless}
    baro_temp, IMU_temp: Int16;                          {cdeg}
    pressure_abs, pressure_diff: float;                  {hPa}
    pressure_raw, baro_rawtemp: int32;
    AccX, AccY, AccZ: int16;
    AccCaliX, AccCaliY, AccCaliZ: single;
    GyroX, GyroY, GyroZ: int16;
    GyroCaliX, GyroCaliY, GyroCaliZ: single;
    MagX, MagY, MagZ: int16;
    MagOfsX, MagOfsY, MagOfsZ: int16;
    MagDecl: float;
    voltage, vccboard, current: uint16;                  {mV, mA}
    RangeFinderDist, RangeFinderRawVoltage: single;
  end;

type
  THWFlags = record
    sensor_present, sensor_enabled, sensor_healthy: UInt32;

  end;

type
  TCommandLong = record
    params: array[0..6] of single;
    commandID: uint16;
  end;


type
  TFourBytes = packed array[0..3] of Byte;

{Public functions and procedures}
function CRC16X25(const msg: TMAVmessage; LengthFixPart: byte; startpos: byte=1): uint16;  {for $BC}
function CheckCRC16X25(const msg: TMAVmessage; LengthFixPart: byte): boolean;
procedure CRC_accumulate(const b: byte; var crcAccum: uint16);
function CRC16MAV(const msg: TMAVmessage; LengthFixPart: byte; startpos: byte=1): uint16;  {for $FE}
function CheckCRC16MAV(const msg: TMAVmessage; LengthFixPart: byte; startpos: byte=1): boolean;

procedure ClearMAVmessage(var msg: TMAVmessage);
procedure ClearAttitudeData(var data: TAttitudeData);
procedure GPSdata_SetDefaultValues(var GPSvalues: TGPSdata);

function MavGetUInt64(const msg: TMAVmessage; pos: integer): uint64;
function MAVGetUINT64Reverse(const msg: TMAVmessage; pos: integer): uint64;  {Big endian}
function MavGetInt16(const msg: TMAVmessage; pos: integer): int16;
function MavGetUInt16(const msg: TMAVmessage; pos: integer): uint16;
function MavGetUInt32(const msg: TMAVmessage; pos: integer): uint32;
function MavGetInt32(const msg: TMAVmessage; pos: integer): int32;
function MavGetFloat(const msg: TMAVmessage; pos: integer): single;
procedure MavFloatToBytes(var msg: TMAVmessage; const pos: integer; value: single);
procedure Read3UInt16(const msg: TMAVmessage; pos: integer; var x, y, z: uint16);
procedure Read3Int16(const msg: TMAVmessage; pos: integer; var x, y, z: int16);

function SensorTypeToStr(id: byte): string;  {Message BC}
function MAV_PARAM_TYPEtoStr(const id: byte): string;        {Specifies the datatype of a MAVLink parameter}

function MicroSecondsToDateTime(const t: uint64): TDateTime;
function MilliSecondsToDateTime(const t: uint64): TDateTime;
function SatAzimuthToDeg(const azi: byte): single;       {[deg] Direction of satellite, 0: 0 deg, 255: 360 deg}
function SatElevationToDeg(const ele: byte): single;     {[deg] Elevation of satellite, 0: 0 deg, 255: 90 deg}

function FormatCoordinates(const coord: int32): shortstring;
function FormatAltitude(const alt: int32): shortstring;       {[mm]}
function FormatDOP(const dop: uint32): shortstring;
function FormatSpeed(const vel: uint32): shortstring;
function FormatXYZSpeed(const vel: int32): shortstring;
function FormatHdg(const hdg: uint32): shortstring;
function FormatDeziProcent(const prz: uint16): shortstring;
function FormatMilliVolt(const volt: uint16): shortstring;
function FixTypeToStr(const fixtype: byte): shortstring;      {MAVlink GPS fix type to string}

function MsgFormatTypeToStr(mft: byte): shortstring;

function BatteryTypeToStr(const batttype: byte): shortstring;
function BatteryFunctionToStr(const battfunc: byte): shortstring;
function BatteryChargeStateToStr(const battstate: byte): shortstring;

function FormatAcc(const acc: uint32): shortstring;           {[mm]}
function FormatVelAcc(const acc: uint32): shortstring;        {[mm]}
function FormatHdgAcc(const acc: uint32): shortstring;        {[degE5] Heading / track uncertainty}
function SeverityToStr(const severity: byte): shortstring;
function RadToDegree180(const radangle: single): single;      {rad to ° +/-180}
function RadToDegree360(const radangle: single): single;      {rad to ° 0..360, 0 is north}
function GimbalAngleToDegree(const angle: uint16): single;
function GimbalPanToDegree(const angle: uint16): single;
function Value3D(const x, y, z: single): single;
function IntToHexSpace(const hex: uint64; len: byte=8; space: byte=4; separator: char=' '): string;


implementation

{Tabelle CCITT X25 CRC aus ST16 MavLinkPackage.java
 b ... Array of Byte
 ln... Länge Payload (Byte 1) der Message, Byte 0=$BC wird nicht genutzt
       Schleife über Rest der Message 0...Länge Payload+Länge Fixpart-3}

function CRC16X25(const msg: TMAVmessage; LengthFixPart: byte; startpos: byte=1): uint16;
const
  Crc16Tab: array[0..255] of Word = (
    $0000, $1189, $2312, $329B, $4624, $57AD, $6536, $74BF,
    $8C48, $9DC1, $AF5A, $BED3, $CA6C, $DBE5, $E97E, $F8F7,
    $1081, $0108, $3393, $221A, $56A5, $472C, $75B7, $643E,
    $9CC9, $8D40, $BFDB, $AE52, $DAED, $CB64, $F9FF, $E876,
    $2102, $308B, $0210, $1399, $6726, $76AF, $4434, $55BD,
    $AD4A, $BCC3, $8E58, $9FD1, $EB6E, $FAE7, $C87C, $D9F5,
    $3183, $200A, $1291, $0318, $77A7, $662E, $54B5, $453C,
    $BDCB, $AC42, $9ED9, $8F50, $FBEF, $EA66, $D8FD, $C974,
    $4204, $538D, $6116, $709F, $0420, $15A9, $2732, $36BB,
    $CE4C, $DFC5, $ED5E, $FCD7, $8868, $99E1, $AB7A, $BAF3,
    $5285, $430C, $7197, $601E, $14A1, $0528, $37B3, $263A,
    $DECD, $CF44, $FDDF, $EC56, $98E9, $8960, $BBFB, $AA72,
    $6306, $728F, $4014, $519D, $2522, $34AB, $0630, $17B9,
    $EF4E, $FEC7, $CC5C, $DDD5, $A96A, $B8E3, $8A78, $9BF1,
    $7387, $620E, $5095, $411C, $35A3, $242A, $16B1, $0738,
    $FFCF, $EE46, $DCDD, $CD54, $B9EB, $A862, $9AF9, $8B70,
    $8408, $9581, $A71A, $B693, $C22C, $D3A5, $E13E, $F0B7,
    $0840, $19C9, $2B52, $3ADB, $4E64, $5FED, $6D76, $7CFF,
    $9489, $8500, $B79B, $A612, $D2AD, $C324, $F1BF, $E036,
    $18C1, $0948, $3BD3, $2A5A, $5EE5, $4F6C, $7DF7, $6C7E,
    $A50A, $B483, $8618, $9791, $E32E, $F2A7, $C03C, $D1B5,
    $2942, $38CB, $0A50, $1BD9, $6F66, $7EEF, $4C74, $5DFD,
    $B58B, $A402, $9699, $8710, $F3AF, $E226, $D0BD, $C134,
    $39C3, $284A, $1AD1, $0B58, $7FE7, $6E6E, $5CF5, $4D7C,
    $C60C, $D785, $E51E, $F497, $8028, $91A1, $A33A, $B2B3,
    $4A44, $5BCD, $6956, $78DF, $0C60, $1DE9, $2F72, $3EFB,
    $D68D, $C704, $F59F, $E416, $90A9, $8120, $B3BB, $A232,
    $5AC5, $4B4C, $79D7, $685E, $1CE1, $0D68, $3FF3, $2E7A,
    $E70E, $F687, $C41C, $D595, $A12A, $B0A3, $8238, $93B1,
    $6B46, $7ACF, $4854, $59DD, $2D62, $3CEB, $0E70, $1FF9,
    $F78F, $E606, $D49D, $C514, $B1AB, $A022, $92B9, $8330,
    $7BC7, $6A4E, $58D5, $495C, $3DE3, $2C6A, $1EF1, $0F78);

var
  i: integer;

begin
  result:=$FFFF;                             {CRC Initializing}
  for i:=startpos to LengthFixPart+msg.msglength-1 do begin
    result:=((result shr 8) and $00FF) xor
            Crc16Tab[((msg.msgbytes[i]) xor result) and $00FF];
  end;
end;

function CheckCRC16X25(const msg: TMAVmessage; LengthFixPart: byte): boolean;
begin
  result:=(CRC16X25(msg, LengthFixPart+2)=0);
end;  {result should be zero over all bytes except Record ID including CRC}

{Translated from checksum.h of MavlinkLib-master}
procedure CRC_accumulate(const b: byte; var crcAccum: uint16);
var
  tmp: byte;

begin
  tmp:=b xor (crcAccum and $00FF);
  tmp:=tmp xor (tmp shl 4);
  crcAccum:=(crcAccum shr 8) xor (tmp shl 8) xor (tmp shl 3) xor (tmp shr 4);
end;

function CRC16MAV(const msg: TMAVmessage; LengthFixPart: byte; startpos: byte=1): uint16;
var
  i: integer;

begin
  result:=$FFFF;
  for i:=startpos to LengthFixPart+msg.msglength-1 do begin
    CRC_accumulate(msg.msgbytes[i], result);
  end;
  CRC_accumulate(0, result);
end;

function CheckCRC16MAV(const msg: TMAVmessage; LengthFixPart: byte; startpos: byte=1): boolean;
begin
  result:=(CRC16MAV(msg, LengthFixPart, startpos)=MavGetUInt16(msg, LengthFixPart+msg.msglength));
end;

procedure ClearMAVmessage(var msg: TMAVmessage);
begin
  msg:=default(TMAVmessage);
  msg.valid:=false;
end;

procedure ClearAttitudeData(var data: TAttitudeData);
begin
  data:=default(TAttitudeData);
end;

procedure GPSdata_SetDefaultValues(var GPSvalues: TGPSdata);
begin
  GPSvalues:=default(TGPSdata);                               {Set all to zero}
  with GPSvalues do begin
    eph:=max16;
    epv:=max16;
    vel:=max16;
    cog:=max16;
    hdg:=max16;
    voltage:=max16;
    sensors_OK:=true;
    gps_present:=true;
  end;
end;

function MavGetUInt64(const msg: TMAVmessage; pos: integer): uint64;
var
  i: integer;

begin
  result:=0;
  for i:=0 to 7 do begin
    result:=result+msg.msgbytes[pos+i]*(256**i);
  end;
end;

function MAVGetUINT64Reverse(const msg: TMAVmessage; pos: integer): uint64;
var
  i: integer;

begin
  result:=0;
  for i:=0 to 7 do begin
    result:=result+msg.msgbytes[pos+i]*(256**(7-i));
  end;
end;

function MavGetInt16(const msg: TMAVmessage; pos: integer): int16;
begin
  result:=msg.msgbytes[pos]+msg.msgbytes[pos+1]*256;
end;

function MavGetUInt16(const msg: TMAVmessage; pos: integer): uint16;
begin
  result:=msg.msgbytes[pos]+msg.msgbytes[pos+1]*256;
end;

function MavGetUInt32(const msg: TMAVmessage; pos: integer): uint32;
var
  i: integer;

begin
  result:=0;
  for i:=0 to 3 do begin
    result:=result+msg.msgbytes[pos+i]*(256**i);
  end;
end;

function MavGetInt32(const msg: TMAVmessage; pos: integer): int32;
var
  i: integer;

begin
  result:=0;
  for i:=0 to 3 do begin
    result:=result+msg.msgbytes[pos+i]*(256**i);
  end;
end;

function MavGetFloat(const msg: TMAVmessage; pos: integer): single;
var i: integer;
    wfl: packed array[0..3] of Byte;
    wx: Single absolute wfl;

begin
  result:=0;
  for i:=0 to 3 do                                       {Endianess prüfen (to/downto)}
    wfl[i]:=msg.msgbytes[i+pos];                         {4 byte aus Buffer ausschneiden}
  result:=wx;                                            {Typecast mittels absolute}
end;

procedure MavFloatToBytes(var msg: TMAVmessage; const pos: integer; value: single);
var
    i: byte;
    by: TFourBytes;

begin
  Move(value, by, 4);
  for i:=0 to 3 do
    msg.msgbytes[i+pos]:=by[i];
end;


procedure Read3UInt16(const msg: TMAVmessage; pos: integer; var x, y, z: uint16);
begin
  x:=MavGetUInt16(msg, pos);
  y:=MavGetUInt16(msg, pos+2);
  z:=MavGetUInt16(msg, pos+4);
end;

procedure Read3Int16(const msg: TMAVmessage; pos: integer; var x, y, z: int16);
begin
  x:=MavGetUInt16(msg, pos);
  y:=MavGetUInt16(msg, pos+2);
  z:=MavGetUInt16(msg, pos+4);
end;


function MicroSecondsToDateTime(const t: uint64): TDateTime;
begin
  result:=t/MilliSecondsPerDay/1000;
end;

function MilliSecondsToDateTime(const t: uint64): TDateTime;
begin
  result:=t/MilliSecondsPerDay;
end;

function SatAzimuthToDeg(const azi: byte): single;       {[deg] Direction of satellite, 0: 0 deg, 255: 360 deg}
begin
  result:=azi*360/255;
end;

function SatElevationToDeg(const ele: byte): single;     {[deg] Elevation of satellite, 0: 0 deg, 255: 90 deg}
begin
  result:=ele*90/255;
end;

function FormatCoordinates(const coord: int32): shortstring;
begin
  result:=FormatFloat(floatformat8, coord/10000000);     {[degE7]}
end;

function FormatAltitude(const alt: int32): shortstring;  {[mm]}
begin
  result:=FormatFloat(floatformat2, alt/1000)+'m';
end;

function FormatDOP(const dop: uint32): shortstring;
begin
  result:='';
  if dop<max16 then
    result:=FormatFloat(floatformat2, dop/100);
end;

function FormatSpeed(const vel: uint32): shortstring;
begin
  result:='';
  if vel<max16 then
    result:=FormatFloat(floatformat2, vel/100)+'m/s';
end;

function FormatXYZSpeed(const vel: int32): shortstring;
begin
  result:=FormatFloat(floatformat2, vel/100)+'m/s';
end;

function FormatHdg(const hdg: uint32): shortstring;
begin
  result:='';
  if hdg<max16 then
    result:=FormatFloat(floatformat2, hdg/100)+'°';
end;

function FormatDeziProcent(const prz: uint16): shortstring;
begin
  result:='';
  if prz<1001 then
    result:=FormatFloat(floatformat1, prz/10)+'%';
end;

function FormatMilliVolt(const volt: uint16): shortstring;
begin
  result:='';
  if (volt>0) and (volt<max16) then
    result:=FormatFloat(floatformat2, volt/1000)+'V';
end;


function FixTypeToStr(const fixtype: byte): shortstring; {MAVlink GPS fix type to string}
begin
  result:='';
  case fixtype of
    0:	Result:='No GPS connected';
    1:	Result:='No position information, GPS is connected';
    2:	Result:='2D position';
    3:	Result:='3D position';
    4:	Result:='DGPS/SBAS aided 3D position';
    5:	Result:='RTK float, 3D position';
    6:	Result:='RTK fixed, 3D position';
    7:	Result:='Static fixed, typically used for base stations';
    8:	Result:='PPP, 3D position';
  end;
end;

function MsgFormatTypeToStr(mft: byte): shortstring;
begin
  result:='';
  case mft of
    0: result:=rsInvalid;
    1: result:='MAVlink V2 common';        {ToDo}
    2: result:='Yuneec H520 TLOG';
    3: result:='Yuneec Mantis LOG';
    4: result:='Yuneec Mantis FlyLog';
    5: result:='Yuneec HPlus Sensor';
    6: result:='Yuneec H480 Sensor';
  end;
end;

function BatteryTypeToStr(const batttype: byte): shortstring;
begin
  result:='';
  case batttype of
    0: result:=rsNotSpecified;
    1: result:='LiPo';
    2: result:='LiFe';
    3: result:='LiIon';
    4: result:='NiMH';
  end;
end;

function BatteryFunctionToStr(const battfunc: byte): shortstring;
begin
  result:='';
  case battfunc of
    0: result:=rsNotSpecified;
    1: result:='All flight systems';
    2: result:='Propulsion system';
    3: result:='Avionics';
    4: result:='Payload';
  end;
end;

function BatteryChargeStateToStr(const battstate: byte): shortstring;
begin
  result:='';
  case battstate of
    0: result:=rsNotProvided;
    1: result:='Normal operation';
    2: result:='Low';
    3: result:='Critical';
    4: result:='Emergency';
    5: result:='Failed';
    6: result:='Unhealthy';
    7: result:='Charging';
  end;
end;

{ENUMs see: https://github.com/mavlink/c_library_v2/blob/master/common/common.h}
function SeverityToStr(const severity: byte): shortstring;
begin
  result:=IntToStr(severity);   {Default if unknown}
  case severity of
    0: result:='EMERGENCY';     {System is unusable. This is a "panic" condition}
    1: result:='ALERT    ';     {Action should be taken immediately. Indicates error
                                in non-critical systems}
    2: result:='CRITICAL ';     {Action must be taken immediately. Indicates failure
                                in a primary system}
    3: result:='ERROR    ';     {Indicates an error in secondary/redundant systems}
    4: result:='WARNING  ';     {Indicates about a possible future error if this
                                is not resolved within a given timeframe. Example
                                would be a low battery warning}
    5: result:='NOTICE   ';     {An unusual event has occurred, though not an error
                                condition. This should be investigated for the root cause.}
    6: result:='INFO     ';     {Normal operational messages. Useful for logging.
                                No action is required for these messages.}
    7: result:='DEBUG    ';     {Useful non-operational messages that can assist in
                                debugging. These should not occur during normal operation}
  end;
end;

function SensorTypeToStr(id: byte): string;  {Message BC}
begin
  result:=rsUnknown+IntToStr(id);
  case id of
    0:   result:='Heartbeat';
    1:   result:='SYS_STATUS';
    2:   result:='SYSTEM_TIME';
    21:  result:='PARAM_REQUEST_LIST';       {Request all parameters of this component.
                                              After this request, all parameters are emitted.
                                              The parameter microservice is documented at
                                              https://mavlink.io/en/services/parameter.html}
    22:  result:='PARAM_VALUE';
    23:  result:='PARAM_SET';
    24:  result:='GPS_RAW_INT';
    25:  result:='GPS_STATUS';
    27:  result:='RAW_IMU';
    29:  result:='SCALED_PRESSURE';
    30:  result:='ATTITUDE';
    32:  result:='LOCAL_POSITION_NED';
    33:  result:='GLOBAL_POSITION';
    35:  result:='RC_CHANNELS_RAW';
    36:  result:='SERVO_OUTPUT_RAW';
    42:  result:='MISSION_CURRENT';
    51:  result:='MISSION_REQUEST_INT';
    52:  result:='System_type';                    {Text: CGO3_Plus / TyphoonH}
    56:  result:='Serial_number';
    57:  result:='License_Cmd';
    58:  result:='License_Ack';
    62:  result:='NAV_CONTROLLER_OUTPUT';
    65:  result:='RC_CHANNELS';
    74:  result:='VRF_HUD';
    76:  result:='COMMAND_LONG';
    77:  result:='COMMAND_ACK';
    150: result:='SENSOR_OFFSETS';
    163: result:='AHRS';                           {Attitude and Heading Reference System}
    165: result:='HW_STATUS';
    171: result:='DATA64';
    172: result:='DATA96';
    173: result:='RANGEFINDER';
    178: result:='AHRS2';
    179: result:='CAMERA_STATUS';
    182: result:='AHRS3';
    183: result:='AUTOPILOT_VERSION_REQUEST';
    193: result:='EKF_STATUS_REPORT';              {Extended Kalman Filter}
    200: result:='GIMBAL_REPORT';
    253: result:='STATUS_TEXT';
  end;
end;

function MAV_PARAM_TYPEtoStr(const id: byte): string;        {Specifies the datatype of a MAVLink parameter}
begin
  result:='PARAM_TYPE '+intToStr(id);
  case id of
    1: result:='UINT8';
    2: result:='INT8';
    3: result:='UINT16';
    4: result:='INT16';
    5: result:='UINT32';
    6: result:='INT32';
    7: result:='UINT64';
    8: result:='INT64';
    9: result:='REAL32';
    10: result:='REAL64';
  end;
end;

{
1   EKF_ATTITUDE=1,             Set if EKF's attitude estimate is good.
0   EKF_VELOCITY_HORIZ=2,       Set if EKF's horizontal velocity estimate is good.
1   EKF_VELOCITY_VERT=4,        Set if EKF's vertical velocity estimate is good.
0   EKF_POS_HORIZ_REL=8,        Set if EKF's horizontal position (relative) estimate is good.

0   EKF_POS_HORIZ_ABS=16,       Set if EKF's horizontal position (absolute) estimate is good.
1   EKF_POS_VERT_ABS=32,        Set if EKF's vertical position (absolute) estimate is good.
0   EKF_POS_VERT_AGL=64,        Set if EKF's vertical position (above ground) estimate is good.
1   EKF_CONST_POS_MODE=128,     EKF is in constant position mode and does not know it's absolute or relative position.

0   EKF_PRED_POS_HORIZ_REL=256, Set if EKF's predicted horizontal position (relative) estimate is good.
0   EKF_PRED_POS_HORIZ_ABS=512, Set if EKF's predicted horizontal position (absolute) estimate is good.
0   EKF_UNINITIALIZED=1024,     Set if EKF has never been healthy.
0
...
0   EKF_GPS_GLITCHING=32768,    /* Set if EKF believes the GPS input data is faulty. | $8000
}
function EKFstatusToStr(const ekf: uint16; separator: char=';'): string;
begin
  result:='';
// todo
end;

function FormatAcc(const acc: uint32): shortstring;           {[mm]}
begin
  result:=FormatFloat(floatformat2, acc/10000)+'cm';
end;

function FormatVelAcc(const acc: uint32): shortstring;        {[mm/s]}
begin
  result:=FormatFloat(floatformat3, acc/100000)+'m/s';
end;

function FormatHdgAcc(const acc: uint32): shortstring;        {[degE5] Heading / track uncertainty}
begin
  result:=FormatFloat(floatformat3, acc/100000)+'°';
end;

function RadToDegree180(const radangle: single): single;      {rad to ° +/-180}
begin
  result:=radangle*180/pi;
end;

function RadToDegree360(const radangle: single): single;      {rad to ° 0..360, 0 is north}
begin
  result:=(RadToDegree180(radangle)+360) mod 360;
end;

function GimbalAngleToDegree(const angle: uint16): single;
begin
  result:=(angle-$3800)/10;
end;

function GimbalPanToDegree(const angle: uint16): single;
begin
  result:=$800-angle;
  if result<0 then
    result:=result+$1000;
  result:=result*360/$1000;
end;

function Value3D(const x, y, z: single): single;
begin
 result:=sqrt(x*x+y*y+z*z);
end;

function IntToHexSpace(const hex: uint64; len: byte=8; space: byte=4; separator: char=' '): string;
var
  i: byte;
  s: string;

begin
  s:=IntToHex(hex, len);
    if length(s)>space then begin
    result:=s[1];
    for i:=2 to length(s)-1 do begin
      result:=result+s[i];
      if (i mod space)=0 then
          result:=result+separator;
    end;
    result:=result+s[length(s)];
  end else
    result:=s;
end;

end.
