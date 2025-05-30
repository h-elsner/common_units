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
https://github.com/mavlink/c_library_v1/tree/master/common
https://github.com/mavlink/c_library_v1/tree/master/ardupilotmega

https://github.com/mavlink/c_library_v2/tree/master/common

This Unit offers procedures and functions for different Yuneec-specific MAVlink
communication used in Typhoon H hardware.

MagicBC=$BC:   Flight controller <-> GUI, Sensor files, nested in CGO3+ messages
MagicFD=$FD:   PX4 based Yuneec drones  (MAVlink V2)
MagicFE=$FE:   Flight controller <-> Gimbal <-> camera CGO3+

*)

unit mav_msg;                                       {Decode MAVlink messages}

{$mode objfpc}{$H+}

interface

uses
  sysutils, mav_def, DateUtils;

{Public functions and procedures}
// MAV link
procedure SYS_STATUS(const msg: TMAVmessage; index: byte; var data: TGPSdata);
procedure SYS_TIME(const msg: TMAVmessage; index: byte; var data: TGPSdata);
function  PING(const msg: TMAVmessage; index: byte; var data: TGPSdata): byte;
function  PARAM_VALUE(const msg: TMAVmessage; index: byte; var value: single): string;
procedure GPS_RAW_INT(const msg: TMAVmessage; index: byte; var data: TGPSdata);
procedure GPS_STATUS(const msg: TMAVmessage; index: byte; var data: TGPSdata);
procedure RAW_IMU(const msg: TMAVmessage; index: byte; var data: THWstatusData);
procedure SCALED_PRESSURE(const msg: TMAVmessage; index: byte; var data: THWstatusData);
procedure ATTITUDE(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
procedure ATTITUDE_QUATERNION(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
procedure LOCAL_POSITION_NED(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
procedure GLOBAL_POSITION_INT(const msg: TMAVmessage; index: byte; var data: TGPSdata);
procedure RC_CHANNELS_RAW(const msg: TMAVmessage; index: byte; var data: TChannelData);
procedure SERVO_OUTPUT_RAW(const msg: TMAVmessage; index: byte; var data: TChannelData);
procedure VRF_HUD(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
procedure ESTIMATOR_STATUS(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
procedure HIGHRES_IMU(const msg: TMAVmessage; index: byte; var data: THWstatusData);
procedure SENSOR_OFFSETS(const msg: TMAVmessage; index: byte; var data: THWstatusData);
procedure DATA96(const msg: TMAVmessage; index: byte; var data: TData96);
procedure RANGEFINDER(const msg: TMAVmessage; index: byte; var data: THWstatusData);
procedure EKF_STATUS_REPORT(const msg: TMAVmessage; index: byte; var data: TAttitudeData);

function COMMAND_LONG(const msg: TMAVmessage; index: byte; var CommandNumber: uint16): byte;
function COMMAND_ACK(const msg: TMAVmessage; index: byte; var CommandNumber: uint16): byte;
function TextOut(const msg: TMAVmessage; index, len: uint16): string;
function STATUSTEXT(const msg: TMAVmessage; index: byte; SeveritySeparator: char='|'): string;
procedure MOUNT_ORIENTATION(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
function YuneecTimeStampInSeconds(const msg: TMAVmessage): uint64;
function GetSERIAL(const msg: TMAVmessage; index: byte): string;
function GetSYSTEM(const msg: TMAVmessage; index: byte; var FW, FWdate: string): string;

// Gimbal UART messages
function GetGIMBAL_FW_VERSION(msg: TMAVmessage): string;
function GetTEXT_MESSAGE(msg: TMAVmessage): string;
procedure SetCRC(var msg: TMAVmessage; const LengthFixPart: byte; CRC_EXTRA: byte=CRC_EXTRA_FE);

// Send YGC (CGO3+) messages
procedure SetUInt16ToMsg(var msg: TMavMessage; const index, value: uint16);
procedure CreateStandardPartMsg(var msg: TMAVmessage; const MsgLength: byte);
procedure CreateStandardGUIMsg(var msg: TMAVmessage; const MsgLength: byte);
procedure CreateFCHeartBeat(var msg: TMavMessage; const SequenceNumber: byte);
procedure CreateYGCcommandMessage(var msg: TMavMessage; const func: byte=$24);
procedure CreateYGCcommandMessageLong(var msg: TMavMessage; const YGCtype: byte);

// Send YGCali (C90) messages
procedure CreateE90StandardPartMsg(var msg: TMAVmessage; const MsgLength: byte);
procedure CreateE90HeartBeat(var msg: TMavMessage; const SequenceNumber: byte);
procedure CreateE90commandMessage(var msg: TMavMessage;
                                  const SequenceNumber: byte; func: byte=$FE);

// Send GUI messages
procedure SetCRC_BC(var msg: TMAVmessage);
procedure CreateGUIheartbeat(var msg: TMAVmessage);
procedure CreateGUIEmptyMsg(var msg: TMAVmessage; const MsgId, MsgLen: byte);
procedure CreateGUI_SYS_STATUS(var msg: TMAVmessage);    {From GUI with default values}
procedure CreateGUI_PARAM_REQUEST_LIST(var msg: TMAVmessage);
procedure CreateGUI_MISSION_REQUEST_INT(var msg: TMAVmessage; const target: byte);
procedure CreateGUI_PARAM_SET(var msg: TMAVmessage;
                              const parameter: shortstring; value: single);
procedure CreateGUI_COMMAND_LONG(var msg: TMAVmessage; const command: TCommandLong);

implementation

procedure SYS_STATUS(const msg: TMAVmessage; index: byte; var data: TGPSdata);
var
  SensorFlags: THWFlags;

begin
  with SensorFlags do begin
    sensor_present:=MavGetUInt32(msg, index);
    sensor_enabled:=MavGetUInt32(msg, index+4);
    sensor_healthy:=MavGetUInt32(msg, index+8);
    if sensor_present>0 then begin
      data.gps_present:=(sensor_present and H480_HWflags)=H480_HWflags;
      data.sensors_OK:=((sensor_enabled and $FFFEFFFF)=
                        (sensor_healthy and $FFFEFFFF));  {Ignore SR24}
    end;
  end;

  data.load:=MavGetUInt16(msg, index+12);
  data.voltage:=MavGetUInt16(msg, index+14);
  data.drop_rate:=MavGetUInt16(msg, index+18);

end;

procedure SYS_TIME(const msg: TMAVmessage; index: byte; var data: TGPSdata);
begin
  data.timeUTC:=UnixToDateTime(MavGetUInt64(msg, index) div 1000000);  {us --> s}
  if msg.msglength=10 then
    data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, index+8) and $FFFF)
  else
    if msg.msglength=11 then
      data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, index+8) and $FFFFFF)
    else
      data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, index+8));
end;

function PING(const msg: TMAVmessage; index: byte; var data: TGPSdata): byte;
begin
  data.boottime:=MavGetUInt64(msg, index)/MilliSecondsPerDay/1000;
  result:=msg.msgbytes[index+8];                        {Ping_sequence_number}
end;

function PARAM_VALUE(const msg: TMAVmessage; index: byte; var value: single): string;
var      {22}
  i: byte;

begin
  result:='';
  for i:=index+8 to index+21 do
    if msg.msgbytes[i] in [32..126] then
      result:=result+chr(msg.msgbytes[i]);
  value:=MavGetFloat(msg, index);
end;

procedure GPS_RAW_INT(const msg: TMAVmessage; index: byte; var data: TGPSdata);
begin     {24}
  data.boottime:=MavGetUInt64(msg, index)/MilliSecondsPerDay/1000;
  data.lat:=MavGetInt32(msg, index+8);
  data.lon:=MavGetInt32(msg, index+12);
  data.altMSL:=MavGetInt32(msg, index+16);
  data.eph:=MavGetUInt16(msg, index+20);
  data.epv:=MavGetUInt16(msg, index+22);
  data.vel:=MavGetUInt16(msg, index+24);
  data.cog:=MavGetUInt16(msg, index+26);
  data.fix_type:=msg.msgbytes[index+28];
  data.sats_inuse:=msg.msgbytes[index+29] and $3F;

  if msg.msglength>30 then begin
    data.alt_ellipsoid:=MavGetInt32(msg, index+30);
    data.h_acc:=MavGetUInt32(msg, index+34);
    data.v_acc:=MavGetUInt32(msg, index+38);
    data.vel_acc:=MavGetUInt32(msg, index+42);

    if msg.msglength=49 then begin                       {Yuneec specific}
      data.hdg_acc:=MavGetUInt16(msg, index+46);
      data.yaw:=msg.msgbytes[index+48];
    end else begin                                       {msg length = 52}
      data.hdg_acc:=MavGetUInt32(msg, index+46);
      data.yaw:=MavGetUInt16(msg, index+50);
    end;
  end;
end;

procedure GPS_STATUS(const msg: TMAVmessage; index: byte; var data: TGPSdata); {Possibly never used by Yuneec or empty}
var       {25}
  i: integer;

begin
  data.sats_visible:=msg.msgbytes[index] and $3F;
  data.sats_inuse:=0;
  for i:=0 to MAVsatCount do begin
    data.sat_prn[i]:=msg.msgbytes[i+index+1];
    if data.sat_prn[i] in SatPRNinGPS then
      inc(data.numGPS_visible)
    else
      if data.sat_prn[i] in SatPRNinSBAS then
        inc(data.numSBAS_visible)
      else
        if data.sat_prn[i] in SatPRNinGLONASS then
          inc(data.numGLONASS_visible)
        else
          inc(data.numOther_visible);

    data.sat_used[i]:=msg.msgbytes[i+index+21];
    if data.sat_used[i]>0 then begin
      inc(data.sats_inuse);
      if data.sat_prn[i] in SatPRNinGPS then
        inc(data.numGPS_used)
      else
        if data.sat_prn[i] in SatPRNinSBAS then
          inc(data.numSBAS_used)
        else
          if data.sat_prn[i] in SatPRNinGLONASS then
            inc(data.numGLONASS_used)
          else
            inc(data.numOther_used);
    end;

    data.sat_elevation[i]:=msg.msgbytes[i+index+41];
    data.sat_azimuth[i]:=msg.msgbytes[i+index+61];
    data.sat_snr[i]:=msg.msgbytes[i+index+81];
  end;
end;

procedure RAW_IMU(const msg: TMAVmessage; index: byte; var data: THWstatusData); {Possibly never used by Yuneec or empty}
begin     {27}
  data.boottime:=MavGetUInt64(msg, index)/MilliSecondsPerDay/1000;
  data.AccX:=MavGetInt16(msg, index+8);
  data.AccY:=MavGetInt16(msg, index+10);
  data.AccZ:=MavGetInt16(msg, index+12);
  data.GyroX:=MavGetInt16(msg, index+14);
  data.GyroY:=MavGetInt16(msg, index+16);
  data.GyroZ:=MavGetInt16(msg, index+18);
  data.MagX:=MavGetInt16(msg, index+20);
  data.MagY:=MavGetInt16(msg, index+22);
  data.MagZ:=MavGetInt16(msg, index+24);
end;

procedure SCALED_PRESSURE(const msg: TMAVmessage; index: byte; var data: THWstatusData);
begin     {29}
  data.boottime:=MavGetUInt32(msg, index)/MilliSecondsPerDay;
  data.pressure_abs:=MavGetFloat(msg, index+4);
  data.pressure_diff:=MavGetFloat(msg, index+8);
  data.baro_temp:=MavGetUInt16(msg, index+12);
end;

procedure ATTITUDE(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
begin     {30}
  data.boottime:=MavGetUInt32(msg, index)/MilliSecondsPerDay;

  data.roll:=RadToDegree180(MavGetFloat(msg, index+4));
  data.pitch:=RadToDegree180(MavGetFloat(msg, index+8));
  data.yaw:=RadToDegree360(MavGetFloat(msg, index+12));

  data.rollspeed:=MavGetFloat(msg, index+16);
  data.pitchspeed:=MavGetFloat(msg, index+20);
  data.yawspeed:=MavGetFloat(msg, index+24);
end;

procedure ATTITUDE_QUATERNION(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
begin     {31}
  data.boottime:=MavGetUInt32(msg, index)/MilliSecondsPerDay;
// Q1 to Q4 float not used at this time

  data.rollspeed:=MavGetFloat(msg, index+20);
  data.pitchspeed:=MavGetFloat(msg, index+24);
  data.yawspeed:=MavGetFloat(msg, index+28);
end;

procedure LOCAL_POSITION_NED(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
begin     {32}
  data.boottime:=MavGetUInt32(msg, index)/MilliSecondsPerDay;

  data.posx:=MavGetFloat(msg, index+4);
  data.posy:=MavGetFloat(msg, index+8);
  data.posz:=MavGetFloat(msg, index+12);

  data.vx:=MavGetFloat(msg, index+16);
  data.vy:=MavGetFloat(msg, index+20);
  data.vz:=MavGetFloat(msg, index+24);
end;

procedure GLOBAL_POSITION_INT(const msg: TMAVmessage; index: byte; var data: TGPSdata);
begin     {33}
  data.boottime:=MavGetUInt32(msg, index)/MilliSecondsPerDay;

  data.lat:=MavGetInt32(msg, index+4);
  data.lon:=MavGetInt32(msg, index+8);
//  data.altMSL:=MavGetInt32(msg, index+12);        {Wrong in FC data to GUI}
  data.alt_rel:=MavGetInt32(msg, index+16);

  data.vx:=MavGetInt16(msg, index+20);
  data.vy:=MavGetInt16(msg, index+22);
  data.vz:=MavGetInt16(msg, index+24);
  data.hdg:=MavGetUInt16(msg, index+26);
end;

procedure RC_CHANNELS_RAW(const msg: TMAVmessage; index: byte; var data: TChannelData);
var
  i: byte;

begin
  data.boottime:=MavGetUInt32(msg, index)/MilliSecondsPerDay;
  for i:=0 to 7 do
    data. channel_raw[i]:=MavGetUInt16(msg, I*2+index+4);
  data.port:=msg.msgbytes[index+20];
  data.rssi:=msg.msgbytes[index+21];
end;

procedure SERVO_OUTPUT_RAW(const msg: TMAVmessage; index: byte; var data: TChannelData);
var
  i: byte;

begin
  data.boottime:=MavGetUInt32(msg, index)/MilliSecondsPerDay/1000;
  for i:=0 to 7 do
    data.Servo_output[i]:=MavGetUInt16(msg, I*2+index+4);
  data.port:=msg.msgbytes[index+20];
end;

procedure VRF_HUD(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
begin     {74}
  data.airspeed:=MavGetFloat(msg, index);
  data.groundspeed:=MavGetFloat(msg, index+4);
  data.altmsl:=MavGetFloat(msg, index+8);               {For Yuneec: altitude relative}
  data.climbrate:=MavGetFloat(msg, index+12);

  if msg.msglength=20 then begin
    data.heading:=MavGetInt16(msg, index+16);
    data.throttle:=MavGetUInt16(msg, index+18);         {Throttle [%]}
  end else
    data.heading:=msg.msgbytes[index+16];               {Yuneec specific}
end;

procedure HIGHRES_IMU(const msg: TMAVmessage; index: byte; var data: THWstatusData);
begin
  data.boottime:=MavGetUInt64(msg, index)/MilliSecondsPerDay/1000;
  data.XAcc:=MavGetFloat(msg, index+8);
  data.YAcc:=MavGetFloat(msg, index+12);
  data.ZAcc:=MavGetFloat(msg, index+16);
  data.XGyro:=MavGetFloat(msg, index+20);
  data.YGyro:=MavGetFloat(msg, index+24);
  data.ZGyro:=MavGetFloat(msg, index+28);
  data.XMag:=MavGetFloat(msg, index+32);
  data.YMag:=MavGetFloat(msg, index+36);
  data.ZMag:=MavGetFloat(msg, index+40);

  data.pressure_abs:=MavGetFloat(msg, index+44);
  data.pressure_diff:=MavGetFloat(msg, index+48);
  data.pressure_alt:=MavGetFloat(msg, index+52);
  data.pressure_temp:=MavGetFloat(msg, index+56);
end;

procedure SENSOR_OFFSETS(const msg: TMAVmessage; index: byte; var data: THWstatusData);
begin     {150}
  data.MagDecl:=MavGetFloat(msg, index);
  data.MagOfsX:=MavGetInt16(msg, index+36);
  data.MagOfsY:=MavGetInt16(msg, index+38);
  data.MagOfsZ:=MavGetInt16(msg, index+40);

  data.pressure_raw:=MavGetInt32(msg, index+4);
  data.baro_rawtemp:=MavGetInt32(msg, index+8);

  data.GyroCaliX:=MavGetFloat(msg, index+12);
  data.GyroCaliY:=MavGetFloat(msg, index+16);
  data.GyroCaliZ:=MavGetFloat(msg, index+20);

  data.AccCaliX:=MavGetFloat(msg, index+24);
  data.AccCaliY:=MavGetFloat(msg, index+28);
  data.AccCaliZ:=MavGetFloat(msg, index+32);
end;

procedure DATA96(const msg: TMAVmessage; index: byte; var data: TData96);
var
  i: byte;

begin     {172  $AC}
  data.value_type:=msg.msgbytes[index];
  for i:=0 to 23 do
    data.value[i]:=MavGetFloat(msg, i*4+index+2);
end;

procedure RANGEFINDER(const msg: TMAVmessage; index: byte; var data: THWstatusData);
begin     {173  $AD}
  data.RangeFinderDist:=MavGetFloat(msg, index);
  data.RangeFinderRawVoltage:=MavGetFloat(msg, index+4);
end;

procedure EKF_STATUS_REPORT(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
begin     {193  $C1}
  data.velocity_variance:=MavGetFloat(msg, index);
  data.pos_horiz_variance:=MavGetFloat(msg, index+4);
  data.pos_vert_variance:=MavGetFloat(msg, index+8);
  data.compass_variance:=MavGetFloat(msg, index+12);
  data.terrain_alt_variance:=MavGetFloat(msg, index+16);
  data.EKFstatus:=MavGetUInt16(msg, index+20);
end;

function COMMAND_LONG(const msg: TMAVmessage; index: byte; var CommandNumber: uint16): byte;
begin    {76   $4C}
  CommandNumber:=MavGetUint16(msg, index+28);
  {7 parameter (float)}
  result:=msg.msgbytes[index+31];
end;

function COMMAND_ACK(const msg: TMAVmessage; index: byte; var CommandNumber: uint16): byte;
begin    {77   $4D}
  CommandNumber:=MavGetUint16(msg, index);
  result:=msg.msgbytes[index+2];
end;

procedure ESTIMATOR_STATUS(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
begin     {230  $E6}
  data.boottime:=MavGetUInt64(msg, index)/MilliSecondsPerDay/1000;

  data.velocity_variance:=MavGetFloat(msg, index+8);
  data.pos_horiz_variance:=MavGetFloat(msg, index+12);
  data.pos_vert_variance:=MavGetFloat(msg, index+16);
  data.compass_variance:=MavGetFloat(msg, index+20);
  data.terrain_alt_variance:=MavGetFloat(msg, index+24);
  data.tas_ratio:=MavGetFloat(msg, index+28);
  data.pos_horiz_accuracy:=MavGetFloat(msg, index+32);
  data.pos_vert_accuracy:=MavGetFloat(msg, index+36);
  if msg.msglength=41 then
    data.EKFstatus:=msg.msgbytes[index+40] and $FF
  else
    data.EKFstatus:=MavGetUint16(msg, index+40);
end;

function TextOut(const msg: TMAVmessage; index, len: uint16): string;
var
  i: uint16;

begin
  result:='';
  for i:=0 to len-1 do begin
    if msg.msgbytes[index+i] in [10, 13, 32..126] then
      result:=result+chr(msg.msgbytes[index+i]);
  end;
end;

function STATUSTEXT(const msg: TMAVmessage; index: byte; SeveritySeparator: char='|'): string;
begin
  result:=SeverityToStr(msg.msgbytes[index])+SeveritySeparator+
  TextOut(msg, index+1, msg.msglength-1);
end;

procedure MOUNT_ORIENTATION(const msg: TMAVmessage; index: byte; var data: TAttitudeData);
begin     {256}
  data.boottime:=MavGetUInt32(msg, index)/MilliSecondsPerDay;

  data.roll:=MavGetFloat(msg, index+4);
  data.pitch:=MavGetFloat(msg, index+8);
  data.yaw:=MavGetFloat(msg, index+12);
  data.yaw_abs:=MavGetFloat(msg, index+16);
end;

function YuneecTimeStampInSeconds(const msg: TMAVmessage): uint64;
begin                                                    {+2 Time starts after! CRC}
  if (msg.msgformat=3) or (msg.msgformat=4) or (msg.msgformat=5) then
    result:=MavGetUInt64(msg, msg.msglength+LengthFixPartFD+2) div 1000
  else
    result:=MavGetUInt64Reverse(msg, msg.msglength+LengthFixPartFD+2) div 1000000;
end;

function GetSERIAL(const msg: TMAVmessage; index: byte): string;
var
  i: uint32;

begin
  i:=MavGetUInt32(msg, index);
  result:=IntToHex(i, 8)+'-';
  i:=MavGetUInt32(msg, index+4);
  result:=result+IntToHex(i, 8)+'-';
  i:=MavGetUInt32(msg, index+8);
  result:=result+IntToHex(i, 8);
end;

function GetSYSTEM(const msg: TMAVmessage; index: byte; var FW, FWdate: string): string;
var
  i: byte;

begin
  result:='';
  FW:=FormatFloat('0.00', msg.msgbytes[index]/100);
  FWdate:=IntToStr(MavGetUInt16(msg, index+4))+'-'+
          Format('%1.2d', [msg.msgbytes[index+23]])+'-'+
          Format('%1.2d', [msg.msgbytes[index+24]]);
  for i:=index+6 to index+21 do
    if msg.msgbytes[i] in [32..126] then
      result:=result+chr(msg.msgbytes[i]);
end;

function GetGIMBAL_FW_VERSION(msg: TMAVmessage): string;
begin
  result:='';
// From heartbeat message sent by Gimbal
  if (msg.sysid=2) and (msg.msgid=0) and (msg.msgbytes[10]>0) then begin
    result:='V'+FormatFloat('0.00', msg.msgbytes[10]/100);
  end;
end;

function GetTEXT_MESSAGE(msg: TMAVmessage): string;
var
  i: integer;

begin
  result:='';
  for i:=9 to msg.msglength+LengthFixPartFE-1 do begin
    if msg.msgbytes[i]>31 then
      result:=result+Chr(msg.msgbytes[i]);
    if msg.msgbytes[i]=$3A then
      result:=result+' ';
  end;
end;

procedure SetUInt16ToMsg(var msg: TMavMessage; const index, value: uint16);
var
  v: uint16;

begin
  v:=value;
  msg.msgbytes[index]:=v and $00FF;                        {value low}
  v:=v shr 8;
  msg.msgbytes[index+1]:=v and $00FF;                      {value high}
end;

procedure CreateStandardPartMsg(var msg: TMAVmessage; const MsgLength: byte);
begin
  ClearMAVmessage(msg);
  msg.msgbytes[0]:=MagicFE;
  msg.msgbytes[1]:=MsgLength;
  msg.msglength:=MsgLength;
  msg.msgbytes[3]:=1;                                    {SysId FC}
end;

procedure CreateStandardGUIMsg(var msg: TMAVmessage; const MsgLength: byte);
begin
  ClearMAVmessage(msg);
  msg.msgbytes[0]:=MagicBC;
  msg.msgbytes[1]:=MsgLength;
  msg.msglength:=MsgLength;
  msg.msgbytes[2]:=1;                                    {SequNo}
  msg.msgbytes[3]:=200;                                  {SysId GUI=$C8}
  msg.msgbytes[4]:=1;                                    {TargetID flight controller}
end;

procedure SetCRC(var msg: TMAVmessage; const LengthFixPart: byte; CRC_EXTRA: byte=CRC_EXTRA_FE);
var
  crc: uint16;

begin
  crc:=CRC16X25MAV(msg, LengthFixPart, 1, true, CRC_EXTRA);
  SetUInt16ToMsg(msg, msg.msglength+LengthFixPart, crc);
  msg.valid:=true;
end;

procedure SetCRC_BC(var msg: TMAVmessage);
var
  crc: uint16;

begin
  crc:=CRC16X25MAV(msg, LengthFixPartBC);
  SetUInt16ToMsg(msg, msg.msglength+LengthFixPartBC, crc);
  msg.valid:=true;
end;

procedure CreateFCHeartBeat(var msg: TMavMessage; const SequenceNumber: byte);
begin
  CreateStandardPartMsg(msg, 5);
  msg.msgbytes[2]:=SequenceNumber;
  msg.msgbytes[12]:=1;
  SetCRC(msg, LengthFixPartFE, CRC_EXTRA_FE);
end;

{ YGC_Type from gimbal
    1: 'GYRO_POWER'
    2: 'EULER_ANGLE'
    3: 'ACC'
    5: 'TEMP_DIFF'
    6: 'M_STATUS'
    $12: 'Serial number'
    $FE: 'Text info'

 YGC_Command: short commands: len=2
      0: Erase all & Button 1                           !!

      1: len=21, default all Bytes 0
      2: len=21, default all Bytes 0
      3: len=21, default all Bytes 0
      4: len=21, default Bytes 11..16 $64, all other 0
      5: len=21, default Bytes 11..16 $64, all other 0

      6: Read PID
      7: Save PID
      8: Reset PID
      9: IMU temp cali
    $0A: IMU temp erase                                 !!
    $0C: Yaw encoder cali
    $0D: Yaw encoder erase
    $0F: Pre front cali
    $10: Zero phase cali
    $11: Zero phase erase
    $12: Acc cali
    $13: Acc erase
    $14: Front cali
    $15: Front erase
    $16: Close motor
    $17: Open motor
    $18: Read software
    $19: Restart GB
    $1A: High frequency IMU
    $1E: Motor test
    $1F: Reson test
    $20: Vibration test
    $24: Reaction to heartbeat gimbal, no button assigned
    $25: High frequency Gyro
    $26: High frequency Acc
    $27: Close high frequency
}

procedure CreateYGCcommandMessage(var msg: TMavMessage; const func: byte=$24);
begin
  CreateStandardPartMsg(msg, 2);
  msg.msgbytes[2]:=1;
  msg.msgbytes[3]:=YGCsysID;                            {SysId YGC}
  msg.msgbytes[5]:=2;                                   {to Gimbal}
  msg.msgbytes[7]:=2;                                   {MsgID}
  msg.msgbytes[8]:=func;                                {YGC_Type}
  msg.msgbytes[8]:=func;                                {YGC_Command}
  SetCRC(msg, LengthFixPartFE, CRC_EXTRA_FE);
end;

procedure CreateYGCcommandMessageLong(var msg: TMavMessage; const YGCtype: byte);
var
  i: integer;

begin
  CreateStandardPartMsg(msg, 33);                       {Good for 15 int16 values}
  msg.msgbytes[2]:=1;
  msg.msgbytes[3]:=YGCsysID;                            {SysId YGC}
  msg.msgbytes[5]:=2;                                   {to Gimbal}
  msg.msgbytes[7]:=2;                                   {MsgID}
  msg.msgbytes[8]:=YGCtype;                             {YGC_Type}
  if (YGCtype=4) or (YGCtype=5) then
    for i:=11 to 16 do
      msg.msgbytes[i]:=$64;                             {100 decimal, placeholder?}
  SetCRC(msg, LengthFixPartFE, CRC_EXTRA_FE);
end;

procedure CreateE90StandardPartMsg(var msg: TMAVmessage; const MsgLength: byte);
begin
  ClearMAVmessage(msg);
  msg.msgbytes[0]:=MagicFD;
  msg.msgbytes[1]:=MsgLength;
  msg.msglength:=MsgLength;
  msg.msgbytes[5]:=1;                                    {SysId FC}
  msg.sysid:=msg.msgbytes[5];
end;

procedure CreateE90commandMessage(var msg: TMavMessage;
                                  const SequenceNumber: byte; func: byte=$FE);
begin
  CreateE90StandardPartMsg(msg, 50);
  msg.msgbytes[4]:=SequenceNumber;
  SetUInt16ToMsg(msg, 7, 5002);        {MsgID GimbalCali 5002}
  msg.msgbytes[10]:=func;
  SetCRC(msg, LengthFixPartFD, CRC_EXTRA_cmd5002);
  msg.valid:=true;
end;

procedure CreateE90HeartBeat(var msg: TMavMessage; const SequenceNumber: byte);
begin
  CreateE90StandardPartMsg(msg, 9);
  msg.msgbytes[4]:=SequenceNumber;
  msg.msgbytes[18]:=3;
  SetCRC(msg, LengthFixPartFD, CRC_EXTRA_heartbeat);
  msg.valid:=true;
end;


procedure CreateGUIheartbeat(var msg: TMAVmessage);
begin
  CreateStandardGUIMsg(msg, 9);
  msg.msgbytes[10]:=2;
  msg.msgbytes[11]:=3;
  msg.msgbytes[12]:=4;
  msg.msgbytes[13]:=4;                                   {Sys status ?}
  msg.msgbytes[14]:=3;                                   {MAV Version ?}
  SetCRC_BC(msg);
end;

procedure CreateGUIEmptyMsg(var msg: TMAVmessage; const MsgId, MsgLen: byte);
begin
  CreateStandardGUIMsg(msg, MsgLen);                     {Payload filled with 0x00}
  msg.msgbytes[5]:=MsgID;
  SetCRC_BC(msg);
end;

procedure CreateGUI_SYS_STATUS(var msg: TMAVmessage);    {From GUI with default values}
begin
  CreateStandardGUIMsg(msg, 31);
  msg.msgbytes[5]:=1;
  SetUInt16ToMsg(msg, 18, 500);                          {Load 50%}
  SetUInt16ToMsg(msg, 20, 11000);                        {Voltage 11V}
  SetUInt16ToMsg(msg, 22, $FFFF);                        {Current n/a}
  msg.msgbytes[36]:=$FF;                                 {Batt cap % n/a}
  SetCRC_BC(msg);
end;

procedure CreateGUI_PARAM_REQUEST_LIST(var msg: TMAVmessage);
begin
  CreateStandardGUIMsg(msg, 2);
  msg.msgbytes[5]:=21;
  msg.msgbytes[6]:=1;
  SetCRC_BC(msg);
end;

procedure CreateGUI_MISSION_REQUEST_INT(var msg: TMAVmessage; const target: byte);
begin
  CreateStandardGUIMsg(msg, 3);
  msg.msgbytes[5]:=51;
  msg.msgbytes[6]:=target;
  msg.msgbytes[8]:=1;
  SetCRC_BC(msg);
end;

procedure CreateGUI_PARAM_SET(var msg: TMAVmessage; const parameter: shortstring; value: single);
var
  i: byte;

begin
  CreateStandardGUIMsg(msg, 23);
  msg.msgbytes[5]:=23;
  MavFloatToBytes(msg, 6, value);
  msg.msgbytes[10]:=1;
  msg.msgbytes[11]:=1;
  for i:=1 to length(parameter) do
    msg.msgbytes[i+11]:=Ord(parameter[i]);
  msg.msgbytes[28]:=9;
  SetCRC_BC(msg);
end;

procedure CreateGUI_COMMAND_LONG(var msg: TMAVmessage; const command: TCommandLong);
var
  i: byte;

begin
  CreateStandardGUIMsg(msg, 33);
  msg.msgbytes[5]:=76;
  for i:=0 to 6 do
    MavFloatToBytes(msg, 4*i+6, command.params[i]);
    SetUInt16ToMsg(msg, 34, command.commandID);
    for i:=36 to 38 do
  msg.msgbytes[i]:=1;
  SetCRC_BC(msg);
end;

end.
