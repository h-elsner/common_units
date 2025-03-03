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
  SatPRNinGPS=[1..32];
  SatPRNinGLONASS=[65..96];
  SatPRNinSBAS=[33..64, 120..138];

  LengthFixPartBC=6;
  LengthFixPartFD=10;
  LengthFixPartFE=8;
  MagicBC=$BC;  {Flight controller <-> GUI, Sensor files, nested in CGO3+ messages}
  MagicFD=$FD;  {PX4 based Yuneec drones (MAVlink V2)}
  MagicFE=$FE;  {Flight controller <-> Gimbal <-> camera CGO3+}
  
  CRC_EXTRA_FE=0;
  CRC_EXTRA_heartbeat=50;
  CRC_EXTRA_cmd5000=252;
  CRC_EXTRA_cmd5002=224;

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
  timeHzzz='hh:nn:ss.zzz';
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
  TCRC_EXTRA_array=array [0..1] of array of UINT32;

const                         {MAVLINK_MSG_ID_xxx_CRC}
  CRCextra: TCRC_EXTRA_array=(( 0,  1,  2,  4, 20, 21, 22, 23,24,25, 26, 27,28, 29,30, 31, 32, 33, 34, 35, 36,46,50, 62, 65, 69, 70,74, 76, 77, 85, 87,105,111,124,139,140,141,147,148,230,241,245,253,264,265,283,284,285,4007,5000,5002),
                              (50,124,137,237,214,159,220,168,24,23,170,144,67,115,39,246,185,104,237,244,222,11,78,183,118,243,124,20,152,143,140,150, 93, 34, 87,168,181, 47,154,178,163, 90,130, 83, 49, 26, 74, 99,137,  22, 252,CRC_EXTRA_cmd5002));

type
  TMAVmessage = record
    msglength, msgid: uint16;
    msgid32: uint32;
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
    load, voltage, drop_rate: uint16;
    batt_cap: byte;
    gps_present, sensors_OK: boolean;

// Satellites data from GPS_status (25)
    sats_visible: byte;
    sat_prn:       array[0..MAVsatCount] of byte;        {Global satellite ID}
    sat_used:      array[0..MAVsatCount] of byte;        {0 not, 1 used}
    sat_elevation: array[0..MAVsatCount] of byte;        {[deg] Elevation (0: right on top of receiver, 90: on the horizon) of satellite}
    sat_azimuth:   array[0..MAVsatCount] of byte;        {[deg] Direction of satellite}
    sat_snr:       array[0..MAVsatCount] of byte;        {[dB] Signal to noise ratio of satellite}
    numGPS_visible, numGLONASS_visible, numSBAS_visible, numOther_visible: byte;
    numGPS_used, numGLONASS_used, numSBAS_used, numOther_used: byte;
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
    pos_horiz_accuracy, pos_vert_accuracy: single;
    velocity_variance, pos_horiz_variance, pos_vert_variance: single;
    tas_ratio, compass_variance, terrain_alt_variance: single;
    EKFstatus: uint16;
  end;

type
  THWstatusData = record
    boottime: TDateTime;
    rssi: byte;
    load, droprate, batt: UInt16;                        {% or unitless}
    baro_temp, IMU_temp: Int16;                          {cdeg}
    pressure_abs, pressure_diff, pressure_alt, pressure_temp: float;
    pressure_raw, baro_rawtemp: int32;
    AccX, AccY, AccZ: int16;
    XAcc, YAcc, ZAcc: single;
    AccCaliX, AccCaliY, AccCaliZ: single;
    GyroX, GyroY, GyroZ: int16;
    XGyro, YGyro, ZGyro: single;
    GyroCaliX, GyroCaliY, GyroCaliZ: single;
    MagX, MagY, MagZ: int16;
    XMag, YMag, ZMag: single;
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
  TData96 = record
    value: array[0..23] of single;
    value_type: byte;
  end;

type
  TChannelData = record
    boottime: TDateTime;
    channel_raw: array[0..7] of uint16;
    Servo_output: array[0..7] of uint16;
    port, rssi: byte;
  end;

type
  TFourBytes = packed array[0..3] of Byte;

{Public functions and procedures}
function CRC16X25(const msg: TMAVmessage; LengthFixPart: byte): uint16;  {for $BC}
function CheckCRC16X25(const msg: TMAVmessage; LengthFixPart: byte): boolean;
procedure CRC_accumulate(const b: byte; var crcAccum: uint16);
function CRC16MAV(const msg: TMAVmessage; LengthFixPart, CRC_EXTRA: byte): uint16;
function CheckCRC16MAV(const msg: TMAVmessage; LengthFixPart, CRC_EXTRA: byte): boolean;

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
function MsgIDtoStr(id: integer): string;
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
function GetCRCextra(const msgid: integer): byte;


implementation

{Tabelle CCITT X25 CRC aus ST16 MavLinkPackage.java
 b ... Array of Byte
 ln... Länge Payload (Byte 1) der Message, Byte 0=$BC wird nicht genutzt
       Schleife über Rest der Message 0...Länge Payload+Länge Fixpart-3}

function CRC16X25(const msg: TMAVmessage; LengthFixPart: byte): uint16;
const
  CRC16Tab: array[0..255] of Word = (
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
  for i:=1 to LengthFixPart+msg.msglength-1 do begin
    result:=((result shr 8) and $00FF) xor
            CRC16Tab[((msg.msgbytes[i]) xor result) and $00FF];
  end;
end;

function CheckCRC16X25(const msg: TMAVmessage; LengthFixPart: byte): boolean;
begin
  result:=CRC16X25(msg, LengthFixPart+2)=0;
end;  {result should be zero over all bytes except Record ID including CRC}

{Translated from checksum.h of MavlinkLib-master}
procedure CRC_accumulate(const b: byte; var crcAccum: uint16);
var
  tmp: uint8;

begin
  tmp:=b xor (crcAccum and $00FF);
  tmp:=tmp xor (tmp shl 4);
  crcAccum:=(crcAccum shr 8) xor (tmp shl 8) xor (tmp shl 3) xor (tmp shr 4);
end;

function CRC16MAV(const msg: TMAVmessage; LengthFixPart, CRC_EXTRA: byte): uint16;
var
  i: integer;

begin
  result:=$FFFF;
  for i:=1 to LengthFixPart+msg.msglength-1 do begin
    CRC_accumulate(msg.msgbytes[i], result);
  end;
  CRC_accumulate(CRC_EXTRA, result);
end;

function CheckCRC16MAV(const msg: TMAVmessage; LengthFixPart, CRC_EXTRA: byte): boolean;
begin
  result:=(CRC16MAV(msg, LengthFixPart, CRC_EXTRA)=MavGetUInt16(msg, LengthFixPart+msg.msglength));
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
    31:  result:='ATTITUDE_QUATERNION';
    32:  result:='LOCAL_POSITION_NED';
    33:  result:='GLOBAL_POSITION';
    35:  result:='RC_CHANNELS_RAW';
    36:  result:='SERVO_OUTPUT_RAW';
    42:  result:='MISSION_CURRENT';
    51:  result:='MISSION_REQUEST_INT';
    52:  result:='System_type';                    {Text: CGO3_Plus / TyphoonH}
    55:  result:='POSITION_TARGET_LOCAL_NED';
    56:  result:='Serial_number';
    57:  result:='License_Cmd';
    58:  result:='License_Ack';
    62:  result:='NAV_CONTROLLER_OUTPUT';
    65:  result:='RC_CHANNELS';
    74:  result:='VRF_HUD';
    76:  result:='COMMAND_LONG';
    77:  result:='COMMAND_ACK';
    105: result:='HIRES_IMU';
    111: result:='TIME_SYNC';
    147: result:='BATTERY_STATUS';
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
    230: result:='ESTIMATOR_STATUS';
    241: result:='VIBRATION';
    245: result:='EXTENDED_SYS_STATE';
    253: result:='STATUS_TEXT';
  end;
end;

{Message Struktur:
 https://github.com/mavlink/c_library_v2/tree/master/common
      inconsistent !
 https://github.com/YUNEEC/MavlinkLib/blob/master/message_definitions/common.xml
 https://github.com/YUNEEC/MavlinkLib}

function MsgIDtoStr(id: integer): string;
begin
  result:=rsUnknown+' MAV_CMD'+' $'+IntToHex(id, 2)+
          ' ('+IntToStr(id)+')';                   {default}
  case id of
      0:  result:='heartbeat';                     {Supported Msg Länge 9}
      1:  result:='sys_status';                    {Supported Msg Länge 1F}
      2:  result:='system_time';                   {Länge 0B}
      4:  result:='ping';
      5:  result:='change_operator_control';
      6:  result:='change_operator_control_ack';
      7:  result:='auth_key';
      8:  result:='link_node_status';
     11:  result:='set_mode';
     19:  result:='param_ack_transaction';
     20:  result:='param_request_read';
     21:  result:='param_request_list';
     22:  result:='param_value';
     23:  result:='param_set';
     24:  result:='gps_raw_int';                   {Supported Msg Länge 31/32}
     25:  result:='gps_status';                    {Länge 1}
     26:  result:='scaled_imu';
    $1B:  result:='raw_imu';
    $1C:  result:='raw_pressure';
    $1D:  result:='scaled_pressure';
    $1E:  result:='attitude';                      {Länge 1C}
    $1F:  result:='attitude_quaternion';           {Supported Msg Länge 20}
    $20:  result:='local_position_ned';            {Länge 1C}
    $21:  result:='global_position_int';           {Supported Msg Länge 1C}
    $22:  result:='rc_channels_scaled';
    $23:  result:='rc_channels_raw';
    $24:  result:='servo_output_raw';              {Länge 10 oder 15}
    $25:  result:='mission_request_partial_list';
    $26:  result:='mission_write_partial_list';
    $27:  result:='mission_item';
    $28:  result:='mission_request';
    $29:  result:='mission_set_current';
    $2A:  result:='mission_current';
     43:  result:='mission_request_list';
    $2C:  result:='mission_count';                 {Länge 3 oder 5}
    $2D:  result:='mission_clear_all';
    $2E:  result:='mission_item_reached';
    $2F:  result:='mission_ack';
    $30:  result:='set_gps_global_origin';
    $31:  result:='gps_global_origin';
    $32:  result:='param_map_rc';
     51:  result:='mission_request_int';
     52:  result:='mission_changed';

     54:  result:='safety_set_allowed_area';
     55:  result:='safety_allowed_area';
    $3D:  result:='attitude_quaternion_cov';
    $3E:  result:='nav_controller_output';
    $3F:  result:='global_position_int_cov';
    $40:  result:='local_position_ned_cov';
    $41:  result:='rc_channels';                   {Supported Msg Länge 2A}
    $42:  result:='request_data_stream';
    $43:  result:='data_stream';
    $45:  result:='manual_control';                {Länge 0B}
    $46:  result:='rc_channels_override';          {Länge 11}
    $49:  result:='mission_item_int';
    $4A:  result:='vfr_hud';                       {Länge 11}
    $4B:  result:='command_int';
    $4C:  result:='command_long';                  {Länge 20}
    $4D:  result:='command_ack';
    $4E:  result:='command_cancel';                {78: UTC time stamp, Boot time}
    $4F:  result:='command_long_stamped';          {79: not supported anymore}
    $51:  result:='manual_setpoint';
    $52:  result:='set_attitude_target';
    $53:  result:='attitude_target';               {Länge 24}
    $54:  result:='set_position_target_local_ned';
    $55:  result:='position_target_local_ned';     {Länge 33}
    $56:  result:='set_position_target_global_int';
    $57:  result:='position_target_global_int';    {Länge 3}
    $59:  result:='local_position_ned_system_global_offset';
    $5A:  result:='hil_state';
    $5B:  result:='hil_controls';
    $5C:  result:='hil_rc_inputs_raw';
    $5D:  result:='hil_actuator_controls';

    $64:  result:='optical_flow';
    $65:  result:='global_vision_position_estimate';
    $66:  result:='vision_position_estimate';
    $67:  result:='vision_speed_estimate';
    $68:  result:='vicon_position_estimate';
    $69:  result:='highres_imu';                   {Länge 3E}
    $6A:  result:='optical_flow_rad';
    $6B:  result:='hil_sensor';
    $6C:  result:='sim_state';
    $6D:  result:='radio_status';
    $6E:  result:='file_transfer_protocol';
    $6F:  result:='timesync';                      {Länge 0D}
    $70:  result:='camera_trigger';
    $71:  result:='hil_gps';
    $72:  result:='hil_optical_flow';
    $73:  result:='hil_state_quaternion';
    116:  result:='scaled_imu2';
    $75:  result:='log_request_list';
    $76:  result:='log_entry';
    $77:  result:='log_request_data';
    $78:  result:='log_data';
    $79:  result:='log_erase';
    $7A:  result:='log_request_end';
    $7B:  result:='gps_inject_data';
    $7C:  result:='gps2_raw';
    $7D:  result:='power_status';
    $7E:  result:='serial_control';
    $7F:  result:='gps_rtk';
    $80:  result:='gps2_rtk';
    $81:  result:='scaled_imu3';
    $82:  result:='data_transmission_handshake';
    $83:  result:='encapsulated_data';
    $84:  result:='distance_sensor';
    $85:  result:='terrain_request';
    $86:  result:='terrain_data';
    $87:  result:='terrain_check';
    $88:  result:='terrain_report';
    $89:  result:='scaled_pressure2';
    $8A:  result:='att_pos_mocap';
    $8B:  result:='set_actuator_control_target';
    $8C:  result:='actuator_control_target';       {Länge 14}
    $8D:  result:='altitude';                      {Länge 20}
    $8E:  result:='resource_request';
    $8F:  result:='scaled_pressure3';
    $90:  result:='follow_target';
    $92:  result:='control_system_state';
    $93:  result:='battery_status';
    $94:  result:='autopilot_version';             {Länge 34, 48, 4C}
    149:  result:='landing_target';

    150: result:='SENSOR_OFFSETS';
    162:  result:='fence_status';
    163: result:='AHRS';                           {Attitude and Heading Reference System}
    165: result:='HW_STATUS';
    172: result:='DATA96';                         {Whatever this is...}
    173: result:='RANGEFINDER';
    178: result:='AHRS2';
    192:  result:='mag_cal_report';
    193: result:='EKF_STATUS_REPORT';              {Extended Kalman Filter}

    225:  result:='efi_status';

{MESSAGE IDs 180 - 229: Space for custom messages in
 individual projectname_messages.xml files -->}
(*  201:  result:='sens_power';                    {I do not know if used}
    202:  result:='sens_MPTT';
    203:  result:='aslctrl_data';
    204:  result:='aslctrl_debug';
    205:  result:='asluav_status';
    206:  result:='ekf_ext';                       {Wind speed and such stuff}
    207:  result:='asl_obctrl';
    208:  result:='sens_atmos';                    {Atmospheric sensors}
    209:  result:='sens_batmon';                   {Battery monitor}
    210:  result:='fw_soaring_data';               {fixed wing...}
    211:  result:='sensorpod_status';
    212:  result:='sens_power_board';
    213:  result:='gsm_link_status';               {LTE too}       *)

    230:  result:='estimator_status';              {Länge 2A}
    $E7:  result:='wind_cov';                      {Länge 20}
    $E8:  result:='gps_input';
    $E9:  result:='gps_rtcm_data';
    $EA:  result:='high_latency';
    $EB:  result:='high_latency2';
    241:  result:='vibration';                     {Länge 14}
    $F2:  result:='home_position';                 {Supported Msg Länge 28 oder 3C}
    $F3:  result:='set_home_position';
    $F4:  result:='message_interval';
    $F5:  result:='extended_sys_state';            {Länge 02}
    $F6:  result:='adsb_vehicle';
    $F7:  result:='collision';
    $F8:  result:='v2_extension';
    $F9:  result:='memory_vect';
    $FA:  result:='debug_vect';
    $FB:  result:='named_value_float';
    $FC:  result:='named_value_int';
    $FD:  result:='statustext';                    {Länge variabel}
    $FE:  result:='debug';
    256:  result:='setup_signing';
    $101: result:='button_change';
    $102: result:='play_tune';
    $103: result:='camera_information';
    $104: result:='camera_settings';
    $105: result:='storage_information';
    $106: result:='camera_capture_status';
    $107: result:='camera_image_captured';         {Länge FC}
    $108: result:='flight_information';            {Supported Msg Länge 1B}
    $109: result:='mount_orientation';             {Länge 20}
    $10A: result:='logging_data';
    $10B: result:='logging_data_acked';
    $10C: result:='logging_ack';
    $10D: result:='video_stream_information';
    $10E: result:='video_stream_status';           {270 len 19}
    $10F: result:='camera_fov_status';             {271 len 52}

    275:  result:='camera_tracking_image_status';  {275 len 31}
    276:  result:='camera_tracking_geo_status';    {276 len 49}

    280:  result:='gimbal_manager_information';
    281:  result:='gimbal_manager_status';
    282:  result:='gimbal_manager_set_attitude';
    283:  result:='gimbal_device_information';
    284:  result:='gimbal_device_set_attitude';
    285:  result:='gimbal_device_attitude_status';
    286:  result:='autopilot_state_for_gimbal_device';
    287:  result:='gimbal_manager_set_pitchyaw';
    288:  result:='gimbal_manager_set_manual_control';
    290:  result:='esc_info';
    291:  result:='esc_status';

    299:  result:='wifi_config_ap';
    300:  result:='protocol_version';              {12C'h not supported anymore}

    301:  result:='ais_vessel';

    310:  result:='uavcan_node_status';
    $137: result:='uavcan_node_info';
    $140: result:='param_ext_request_read';
    $141: result:='param_ext_request_list';
    $142: result:='param_ext_value';               {Länge 95}
    $143: result:='param_ext_set';
    $144: result:='param_ext_ack';                 {Länge 91}
    $14A: result:='obstacle_distance';             {Länge 9E}
    $14B: result:='odometry';
    $14C: result:='trajectory_representation_waypoints';
    $14D: result:='trajectory_representation_bezier';

    336:  result:='cellular_config';

    339:  result:='raw_rpm';
    340:  result:='UTM_global_position';           {154'h}
    350:  result:='debug_float_array';
    360:  result:='orbit_execution_status';

    370:  result:='smart_battery_info';
    373:  result:='generator_status';
    375:  result:='actuator_output_status';
    380:  result:='time_estimate_to_target';
    385:  result:='tunnel';
    390:  result:='onboard_computer_status';
    395:  result:='component_information';
    400:  result:='play_tune v2';
    401:  result:='supported_tunes';

    5000: result:='gimbal_control';                {CRC_EXTRA=252}
    5002: result:='gimbal_calibration';

    9000: result:='wheel_distance';                {2328'h}
    9005: result:='winch_status';

   12900: result:='open_drone_id_basic_id';        {3264'h}
   12901: result:='open_drone_id_location';
   12902: result:='open_drone_id_authentication';
   12903: result:='open_drone_id_self_id';
   12904: result:='open_drone_id_system';
   12905: result:='open_drone_id_operator_id';
   12915: result:='open_drone_id_message_pack';
   12918: result:='open_drone_id_arm_status';
   12919: result:='open_drone_id_system_update';
   19920: result:='hygrometer_sensor';
  end;
end;

function MAV_PARAM_TYPEtoStr(const id: byte): string;        {Specifies the datatype of a MAVLink parameter}
begin
  result:='PARAM_TYPE '+intToStr(id);
  case id of
    1:  result:='UINT8';
    2:  result:='INT8';
    3:  result:='UINT16';
    4:  result:='INT16';
    5:  result:='UINT32';
    6:  result:='INT32';
    7:  result:='UINT64';
    8:  result:='INT64';
    9:  result:='REAL32';
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
  if radangle<1000 then
    result:=radangle*180/pi
  else
    result:=radangle;
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

function GetCRCextra(const msgid: integer): byte;
var
  i: integer;

begin
  result:=CRC_EXTRA_FE;
  for i:=0 to High(CRCextra[0]) do begin
    if CRCextra[0, i]=msgid then begin
      exit(CRCextra[1, i]);
    end;
  end;
end;

end.
