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

MsgID	MsgID	*	MAVlink Message
0	$000		HEARTBEAT
1	$001	*       SYS_STATUS
2	$002	*	SYSTEM_TIME
4	$004	*	PING
24	$018	*	GPS_RAW_INT
25	$019	*	GPS_STATUS
29      $01D    *       SCALED_PRESSURE
30	$01E	*	ATTITUDE
31	$01F		ATTITUDE_QUATERNION
32	$020	*	LOCAL_POSITION_NED
33	$021	*	GLOBAL_POSITION_INT
36	$024		SERVO_OUTPUT_RAW
65	$041		RC_CHANNELS
69	$045		MANUAL_CONTROL
70	$046		RC_CHANNELS_OVERRIDE
74	$04A	*	VFR_HUD
76	$04C		COMMAND_LONG
77	$04D		COMMAND_ACK
83	$053		ATTITUDE_TARGET
85	$055		POSITION_TARGET_LOCAL_NED
87	$057		POSITION_TARGET_GLOBAL_INT
105	$069		HIGHRES_IMU
111	$06F		TIMESYNC
140	$08C            ACTUATOR_CONTROL_TARGET
141	$08D		ALTITUDE
147	$093		BATTERY_STATUS
148	$094		AUTOPILOT_VERSION
230	$0E6		ESTIMATOR_STATUS
231	$0E7		WIND_COV
241	$0F1		VIBRATION
242	$0F2		HOME_POSITION
245	$0F5		EXTENDED_SYS_STATE
253	$0FD	*	STATUSTEXT
259	$103		CAMERA_INFORMATION
260	$104		CAMERA_SETTINGS
261	$105		STORAGE_INFORMATION
262	$106		CAMERA_CAPTURE_STATUS
264	$108		FLIGHT_INFORMATION
265	$109		MOUNT_ORIENTATION
322	$142		PARAM_EXT_VALUE
323	$143		PARAM_EXT_SET
324	$144		PARAM_EXT_ACK
340	$154		UTM_GLOBAL_POSITION

*)

unit mav_msg;                                       {Decode MAVlink messages}

{$mode objfpc}{$H+}

interface

uses
  sysutils, mav_def, DateUtils;

{Public functions and procedures}
// MAV link
procedure SYS_STATUS(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
procedure SYS_TIME(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
function  PING(const msg: TMAVmessage; offset: byte; var data: TGPSdata): byte;
procedure GPS_RAW_INT(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
procedure GPS_STATUS(const msg: TMAVmessage; offset: byte; var data: TGPSdata);  {Possibly never used by Yuneec or empty}
procedure SCALED_PRESSURE(const msg: TMAVmessage; offset: byte; var data: THWstatusData);
procedure ATTITUDE(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
procedure LOCAL_POSITION_NED(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
procedure GLOBAL_POSITION_INT(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
procedure VRF_HUD(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
procedure MOUNT_ORIENTATION(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);

function STATUSTEXT(const msg: TMAVmessage; offset: byte; SeveritySeparator: char='|'): string;
function YuneecTimeStampInSeconds(const msg: TMAVmessage): uint64;

// Gimbal UART messages
function GetCAM_SERIAL(msg: TMAVmessage): string;
function GetGIMBAL_FW_VERSION(msg: TMAVmessage): string;
function GetTEXT_MESSAGE(msg: TMAVmessage): string;

// Send YGC messages
procedure SetUInt16ToMsg(var msg: TMavMessage; const pos, value: uint16);
procedure CreateStandardPartMsg(var msg: TMAVmessage; const MsgLength: byte);
procedure SetCRC(var msg: TMAVmessage);
procedure CreateFCHeartBeat(var msg: TMavMessage; SequenceNumber: byte);
procedure CreateYGCcommandMessage(var msg: TMavMessage; const func: byte=$24);
procedure CreateYGCcommandMessageLong(var msg: TMavMessage; const YGCtype: byte);


implementation

procedure SYS_STATUS(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
var
  SensorFlags: THWFlags;

begin
  with SensorFlags do begin
    sensor_present:=MavGetUInt32(msg, offset);
    sensor_enabled:=MavGetUInt32(msg, offset+4);
    sensor_healthy:=MavGetUInt32(msg, offset+8);

    if sensor_present>0 then begin
      data.gps_present:=((sensor_present and $04)>0) and           {mag}
                        ((sensor_present and $20)>0);              {gps}
      data.sensors_OK:=(sensor_present=sensor_enabled) and
                       (sensor_present=sensor_healthy);
    end;
  end;

  data.load:=MavGetUInt16(msg, offset+12);
  data.voltage:=MavGetUInt16(msg, offset+14);


end;

procedure SYS_TIME(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
begin
  data.timeUTC:=UnixToDateTime(MavGetUInt64(msg, offset) div 1000000);  {us --> s}
  if msg.msglength=10 then
    data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, offset+8) and $FFFF)
  else
    if msg.msglength=11 then
      data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, offset+8) and $FFFFFF)
    else
      data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, offset+8));
end;

function PING(const msg: TMAVmessage; offset: byte; var data: TGPSdata): byte;
begin
  data.boottime:=MavGetUInt64(msg, offset)/MilliSecondsPerDay/1000;
  result:=msg.msgbytes[offset+8];                        {Ping_sequence_number}
end;

procedure GPS_RAW_INT(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
begin
  data.boottime:=MavGetUInt64(msg, offset)/MilliSecondsPerDay/1000;
  data.lat:=MavGetInt32(msg, offset+8);
  data.lon:=MavGetInt32(msg, offset+12);
  data.altMSL:=MavGetInt32(msg, offset+16);
  data.eph:=MavGetUInt16(msg, offset+20);
  data.epv:=MavGetUInt16(msg, offset+22);
  data.vel:=MavGetUInt16(msg, offset+24);
  data.cog:=MavGetUInt16(msg, offset+26);
  data.fix_type:=msg.msgbytes[offset+28];
  data.sats_visible:=msg.msgbytes[offset+29];

  if msg.msglength>30 then begin
    data.alt_ellipsoid:=MavGetInt32(msg, offset+30);
    data.h_acc:=MavGetUInt32(msg, offset+34);
    data.v_acc:=MavGetUInt32(msg, offset+38);
    data.vel_acc:=MavGetUInt32(msg, offset+42);

    if msg.msglength=49 then begin                       {Yuneec specific}
      data.hdg_acc:=MavGetUInt16(msg, offset+46);
      data.yaw:=msg.msgbytes[offset+48];
    end else begin                                       {msg length = 52}
      data.hdg_acc:=MavGetUInt32(msg, offset+46);
      data.yaw:=MavGetUInt16(msg, offset+50);
    end;
  end;
end;

procedure GPS_STATUS(const msg: TMAVmessage; offset: byte; var data: TGPSdata); {Possibly never used by Yuneec or empty}
var
  i: integer;

begin
  data.sats_visible:=msg.msgbytes[offset];
  for i:=0 to MAVsatCount do begin
    data.sat_prn[i]:=msg.msgbytes[i+offset+1];
    data.sat_used[i]:=msg.msgbytes[i+offset+21];
    data.sat_elevation[i]:=msg.msgbytes[i+offset+41];
    data.sat_azimuth[i]:=msg.msgbytes[i+offset+61];
    data.sat_snr[i]:=msg.msgbytes[i+offset+81];
  end;
end;

procedure SCALED_PRESSURE(const msg: TMAVmessage; offset: byte; var data: THWstatusData);
begin
  data.boottime:=MavGetUInt32(msg, offset)/MilliSecondsPerDay;
  data.pressure_abs:=MavGetFloat(msg, offset+4);
  data.pressure_diff:=MavGetFloat(msg, offset+8);
  data.baro_temp:=MavGetUInt16(msg, offset+12);
end;

procedure ATTITUDE(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
begin
  data.boottime:=MavGetUInt32(msg, offset)/MilliSecondsPerDay;

  data.roll:=RadToDegree180(MavGetFloat(msg, offset+4));
  data.pitch:=RadToDegree180(MavGetFloat(msg, offset+8));
  data.yaw:=RadToDegree360(MavGetFloat(msg, offset+12));

  data.rollspeed:=MavGetFloat(msg, offset+16);
  data.pitchspeed:=MavGetFloat(msg, offset+20);
  data.yawspeed:=MavGetFloat(msg, offset+24);
end;

procedure LOCAL_POSITION_NED(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
begin
  data.boottime:=MavGetUInt32(msg, offset)/MilliSecondsPerDay;

  data.posx:=MavGetFloat(msg, offset+4);
  data.posy:=MavGetFloat(msg, offset+8);
  data.posz:=MavGetFloat(msg, offset+12);

  data.vx:=MavGetFloat(msg, offset+16);
  data.vy:=MavGetFloat(msg, offset+20);
  data.vz:=MavGetFloat(msg, offset+24);
end;

procedure GLOBAL_POSITION_INT(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
begin
  data.boottime:=MavGetUInt32(msg, offset)/MilliSecondsPerDay;

  data.lat:=MavGetInt32(msg, offset+4);
  data.lon:=MavGetInt32(msg, offset+8);
  data.altMSL:=MavGetInt32(msg, offset+12);
  data.alt_rel:=MavGetInt32(msg, offset+16);

  data.vx:=MavGetInt16(msg, offset+20);
  data.vy:=MavGetInt16(msg, offset+22);
  data.vz:=MavGetInt16(msg, offset+24);
  data.hdg:=MavGetUInt16(msg, offset+26);
end;

procedure VRF_HUD(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
begin
  data.airspeed:=MavGetFloat(msg, offset);
  data.groundspeed:=MavGetFloat(msg, offset+4);
  data.altmsl:=MavGetFloat(msg, offset+8);               {For Yuneec: altitude relative}
  data.climbrate:=MavGetFloat(msg, offset+12);

  if msg.msglength=20 then begin
    data.heading:=MavGetInt16(msg, offset+16);
    data.throttle:=MavGetUInt16(msg, offset+18);         {Throttle [%]}
  end else
    data.heading:=msg.msgbytes[offset+16];               {Yuneec specific}
end;

procedure MOUNT_ORIENTATION(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
begin
  data.boottime:=MavGetUInt32(msg, offset)/MilliSecondsPerDay;

  data.roll:=MavGetFloat(msg, offset+4);
  data.pitch:=MavGetFloat(msg, offset+8);
  data.yaw:=MavGetFloat(msg, offset+12);
  data.yaw_abs:=MavGetFloat(msg, offset+16);
end;


function STATUSTEXT(const msg: TMAVmessage; offset: byte; SeveritySeparator: char='|'): string;
var
  i: integer;
begin
  result:=SeverityToStr(msg.msgbytes[offset])+SeveritySeparator;
  for i:=1 to msg.msglength-1 do begin
    if msg.msgbytes[offset+i] in [10, 13, 32..127] then
      result:=result+chr(msg.msgbytes[offset+i]);
  end;
end;

function YuneecTimeStampInSeconds(const msg: TMAVmessage): uint64;
begin                                                    {+2 Time starts after! CRC}
  if (msg.msgformat=3) or (msg.msgformat=4) or (msg.msgformat=5) then
    result:=MavGetUInt64(msg, msg.msglength+LengthFixPartFD+2) div 1000
  else
    result:=MavGetUInt64Reverse(msg, msg.msglength+LengthFixPartFD+2) div 1000000;
end;

function GetCAM_SERIAL(msg: TMAVmessage): string;
var
  i: uint32;

begin
  i:=MavGetUInt32(msg, 9);
  result:=IntToHex(i, 8)+'-';
  i:=MavGetUInt32(msg, 13);
  result:=result+IntToHex(i, 8)+'-';
  i:=MavGetUInt32(msg, 17);
  result:=result+IntToHex(i, 8);
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

procedure SetUInt16ToMsg(var msg: TMavMessage; const pos, value: uint16);
var
  v: uint16;

begin
  v:=value;
  msg.msgbytes[pos]:=v and $00FF;   {value low}
  v:=v shr 8;
  msg.msgbytes[pos+1]:=v and $00FF; {value high}
end;

procedure CreateStandardPartMsg(var msg: TMAVmessage; const MsgLength: byte);
begin
  ClearMAVmessage(msg);
  msg.msgbytes[0]:=MagicFE;
  msg.msgbytes[1]:=MsgLength;
  msg.msglength:=msg.msgbytes[1];
  msg.msgbytes[3]:=1;                                  {SysId FC}
end;

procedure SetCRC(var msg: TMAVmessage);
var
  crc: uint16;

begin
  crc:=CRC16MAV(msg, LengthFixPartFE);
  SetUInt16ToMsg(msg, msg.msglength+LengthFixPartFE, crc);
  msg.valid:=true;
end;

procedure CreateFCHeartBeat(var msg: TMavMessage; SequenceNumber: byte);
begin
  CreateStandardPartMsg(msg, 5);
  msg.msgbytes[2]:=SequenceNumber;
  msg.msgbytes[12]:=1;
  SetCRC(msg);
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
  SetCRC(msg);
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
  SetCRC(msg);
end;

end.
