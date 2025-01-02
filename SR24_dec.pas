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



{Data format UART messages (data packages)

byte idx val   desrcription
 0       $55   header 1
 1       $55   header 2
 2       len   24/43 length data after len byte inclusive type and CRC8, max 64
 3       0..3  Msg type: CHANNELDATA12      = 0                len $18  24
	                 CHANNELDATA24      = 1
                         Telemetry to RC    = 2                len $26  38
	                 TRANSMITTERGPSDATA = 3                len $2B  43
                         BIND               = 4                len 8
                         Addition data      = 20               len minimal 7
 4       Counter         0 for old SR24 FW (Q500)
 5       ??    Random?   0 for old SR24 FW (Q500)
 6       RSSI  (in % ?)
 7       Package counter  (lost packages?)
 8       from here on Payload bytes (Channels / GPS data) ...
...
 len+2   $xx   CRC8

Example:
 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 num bytes
 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 idx bytes
         |---------------------------------------------------------------------|   No bytes in Ln   Ln=24 for type 0 / Ln=43 for type 3 (GPS data)
      |---------------------------------------------------------------------|      bytes for CRC8
h1 h2 Ln Tp Cntr  Ri Pc Ch0 Ch1  Ch2 Ch3  Ch4 Ch5  Ch6 Ch7  Ch8 Ch9  Ch10Ch11 CRC8                                                     CRC8

Known Action types in Command messages (msg type 20):
   5 - Sonar config
   9 - LED config
  10 - GPS (Ask for config)
  11 - Home altitude

Others may be from ACTION_TYPE in MissionData.java
   0 - Request
   1 - Response
   2 - Feedback
   3 - Setting CCC
   4 - Settig ROI
   6 - One key take off
   7 - Setting JOUR
   8 - Real Sense depth
}

unit SR24_dec;

{$mode objfpc}{$H+}

interface

uses sysutils;

const
  header1=$55;                                          {Message start ID}
  header2=$55;
  maxlen=67;                                            {according ST24.h define ST24_DATA_LEN_MAX 64}
  ValidMsgTypes=[0..4, 20];
  BindMessage: array [0..10] of byte =
               (header1, header2, 8, 4, 0, 0, $42, $49, $4E, $44, $B0);
               {                len type       B    I    N    D   CRC}
  DefTelemetry: array [0..39] of byte =                 {Default telemetry message for initialization}
               (header1, header2, $26, 2, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     { 6: lat, lon, alt}
                0, 0, 0, 0, 0, 0,                       {18: vx, vy, vz }
                0, 110, 0,                              {24: nsat, voltage (16V), current}
                0, 0, 0, 0, 0, 0,                       {27: roll, pitch, yaw }
	        63, 96, 85,                             {33: motor status, IMU status, Preessure compass}
	        16, 5, 0, 40);                          {36: flight mode, vehicle type, error flags, gps_AccH}

  Telemetry_csvheader_common='Date/Time;Msg_Type;Counter;_?_;RSSI[%];PackageCtnr';
  Telemetry_csvheader_channels=';CH0;CH1;CH2;CH3;CH4;CH5;CH6;CH7;CH8;CH9;CH10;CH11;lat;lon;alt;acc;speed;angle;NumSats';

  rsUndef='Undef';
  rsUnknown='Unknown';
  rsGPSoff='-GPSoff';
  rsGPSlost='-GPSlost';
  fmAngle='Angle';
  fmSmart='Smart';

type
  TPayLoad = array[0..maxlen] of byte;                  {Message array}

function  SR24_CRC8(data: TPayLoad; len: byte): byte;   {Create CRC8 checksum}
function  TestCRC8(data: TPayLoad; len: byte): boolean; inline;  {Check if dataset is valid}

function  GetFloatFromBuf(data: TPayLoad; idx: byte): single;              {Get 4 byte float as single}
function  GetIntFromBuf(data: TPayLoad; idx, len: byte): integer;          {Get len bytes as integer big endian}
procedure IntToTelemetry(var data: TPayload; w: integer; pos, len: byte);  {Convert integer to byte array}
function  GetChValue(data: TPayLoad; chnr: byte; pos: integer=8): uint16;  {Get value from on Channel, channel no starts with 1}
function  CoordToFloat(coord: integer): single;                            {Convert integer cooerdinates to float}
function  GetGPSdata(data: TPayLoad; pos: integer;
                     var lat, lon, alt: single): boolean;  {Get lat, lon and alt from GPS data}

function  VoltToByte(v: single): byte;                  {Convert volt to byte}
function  CurrentToByte(a: single): byte;               {Convert ampere to byte}
function  CoordToInt(coord: single): int32;             {Convert coordinates to integer represetation}
function  AltitudeToInt(alt: single): int32;            {Convert Altitude, 4 byte}
function  SpeedToInt(alt: single): int16;               {Speed 2 byte}
function  GetRSSI(data: TPayLoad): int16;               {Get receiver RSSI in %}
function  MessageTypeToStr(msg_type: byte):string;      {Known message types as string}
function  RawData(data: TPayLoad; len: byte): string;
function  ChannelValues(data: TPayLoad; numch: byte; separator: char=';'): string;
function  ActionTypeToStr(at: byte): string;
function  SwitchPos(sw: byte): string;
function  F_ModeToStr(const f: byte): string;        {Q500, YTH and all other legacy}
procedure GetInt123(data: TPayload; pos: integer; var v1, v2, v3: int16);
function  GetPackageCounter(data: TPayload): byte;

implementation


function SR24_CRC8(data: TPayLoad; len: byte): byte;    {Compute CRC8 checksum}
var
  b, i: byte;

begin                                                   {Translated from ST24.c}
  result:=0;
  for i:=2 to len+1 do begin                            {i points to databyte in array}
    b:=$80;
    repeat
      if (result and $80) <>0 then begin
        result:=result shl 1;
        result:=result xor 7;
      end else
        result:=result shl 1;
      if (data[i] and b)<>0 then
        result:=result xor 7;
      b:=b shr 1;
    until b=0;
  end;
end;

function TestCRC8(data: TPayLoad; len: byte): boolean; inline;
begin
  result:=(data[len+2]=SR24_CRC8(data, len));
end;

{http://forum.lazarus-ide.org/index.php?topic=42182.0
 Direkter Typecast mit dem Zieldatentyp oder die Deklaration mittels absolute}

function GetFloatFromBuf(data: TPayLoad; idx: byte): single; {Position, Länge immer 4}
var i: byte;
    wfl: packed array[0..3] of Byte;
    wx: Single absolute wfl;                            {May result NaN}

begin
  result:=0;
  for i:=0 to 3 do
    wfl[i]:=data[idx+i];                                {Get 4 byte from array}
  result:=wx;                                           {Typecast by absolute}
//  if IsNaan(result) then                                {Optional: Could except NaN}
//    result:=0;
end;

{Integer represent byts to integer values, MSB is right (big endian)}

function GetIntFromBuf(data: TPayLoad; idx, len: byte): integer;   {len is 2, 4 or 8}
var
  i: byte;

begin
  result:=0;                                            {Lowest byte}
  for i:=len-1 downto 1 do
    result:=(result+data[idx+i]) shl 8;
  result:=result+data[idx];
  if ((data[idx+len-1] and $80)<>0) and
     (result>0) then                                    {Check if negative value}
    result:=-result;
end;

procedure IntToTelemetry(var data: TPayload; w: integer; pos, len: byte);
                                                        {Convert integer to byte array}
var
  i, x: integer;
begin
  x:=w;
  data[pos]:=x and $FF;
  for i:=1 to len-1 do begin
    x:=x shr 8;
    data[pos+i]:=x and $FF;
  end;
  if w<0 then
    data[pos+len-1]:=data[pos+len-1] or $80;
end;

function GetChValue(data: TPayLoad; chnr: byte; pos: integer=8): uint16; {Channel no from 1..12 or 1..24}
var
  n: byte;

begin
  n:=((chnr-1) div 2)*3+pos;
  if (chnr and 1)=0 then begin                           {even channel no Ch0...}
    result:=lo(data[n+1])*256+data[n+2];
  end else begin                                         {uneven channel no Ch1...}
    result:=data[n]*16+hi(data[n+1]);
  end;
end;

function CoordToFloat(coord: integer): single;
begin
  result:=coord/10000000;
end;

{Get most important GPS data: Latitude, longitude, altitude (ASL)}

function GetGPSdata(data: TPayLoad; pos: integer; var lat, lon, alt: single): boolean;
var                                                     {Write into lat, lon and alt (ASL)}
  la, lo: int32;

begin
  result:=false;
  la:=GetIntFromBuf(data, pos, 4);
  lo:=GetIntFromBuf(data, pos+4, 4);
  lat:=CoordToFloat(la);
  lon:=CoordToFloat(lo);
  alt:=GetFloatFromBuf(data, pos+8);
  if (la<>0) or (lo<>0) then
    result:=true;
end;

function VoltToByte(v: single): byte;                   {Yuneec voltage representation}
begin
  result:=round((v-5)*10);
end;

function CurrentToByte(a: single): byte;                {Current in A}
begin
  result:=round(a/2);
end;

function CoordToInt(coord: single): int32;              {Convert coordinates from single to integer}
begin
  result:=round(coord*10000000);
end;

function AltitudeToInt(alt: single): int32;             {Convert Altitude in m to interger}
begin
  result:=round(alt*100);
end;

function SpeedToInt(alt: single): int16;                {Convert speed in m/s to integer}
begin
  result:=round(alt*100);
end;

function GetRSSI(data: TPayLoad): int16;                {Get receiver RSSI in %}
begin
  result:=round(data[6]*100/255);                       {in %}
end;

function MessageTypeToStr(msg_type: byte):string;       {Known message types as string}
begin
  result:=rsUnknown+' '+IntToStr(msg_type)+' ($'+HexStr(msg_type, 2)+')';
  case msg_type of
    0:  result:='ChannelData12';
    1:  result:='ChannelData24';
    2:  result:='Telemetry_2.4GHz';
    3:  result:='C-GPS_2.4GHz';
    4:  result:='Bind mode';
    20: result:='AdditionalData';
  end;
end;

function RawData(data: TPayLoad; len: byte): string;
var
  i: byte;
begin
  result:='';
  for i:=0 to len do
    result:=result+HexStr(data[i], 2)+' ';
end;

function ChannelValues(data: TPayLoad; numch: byte; separator: char=';'): string;
var
  i: byte;

begin
  result:='';
  for i:=1 to numch do begin
    result:=result+IntToStr(GetChValue(data, i))+separator;
  end;
end;

{ from  MissionData.java
 public static final int ACTION_TYPE_FEEDBACK = 2;
 public static final int ACTION_TYPE_GOHOME_CONFIG = 11;
 public static final int ACTION_TYPE_LED_CONFIG = 9;
 public static final int ACTION_TYPE_ONEKEY_TAKEOFF = 6;
 public static final int ACTION_TYPE_REALSENSE_DEPTH = 8;
 public static final int ACTION_TYPE_REQUEST = 0;
 public static final int ACTION_TYPE_RESPONSE = 1;
 public static final int ACTION_TYPE_SETTING_CCC = 3;
 public static final int ACTION_TYPE_SETTING_JOUR = 7;
 public static final int ACTION_TYPE_SETTING_ROI = 4;
 public static final int ACTION_TYPE_SONAR_CONFIG = 5; }

function ActionTypeToStr(at: byte): string;
begin
  result:=rsUnknown+' '+IntToStr(at)+' (0x'+HexStr(at, 2)+')';
  case at of
    0:  result:='REQUEST';
    1:  result:='RESPONSE';         {Up and down; Sonar?, Hearbeat? all zero}
    2:  result:='FEEDBACK';         {Comes from FC: 0 0 0 }
    3:  result:='SETTING_CCC';      {down Gimbaldaten?}
    4:  result:='SETTING_ROI';
    5:  result:='Sonar switch';     {Up 2Byte}
    6:  result:='ONEKEY_TAKEOFF';
    7:  result:='SETTING_JOUR';
    8:  result:='REALSENSE_DEPTH';  {down, all zero}
    9:  result:='LED switch';
    10: result:='GPS switch';
    11: result:='Home altitude set';
  end;
end;

function SwitchPos(sw: byte): string;
begin
  result:='off';
  if sw=1 then
    result:='on';
end;

function F_ModeToStr(const f: byte): string;        {Q500, YTH and all other legacy}
begin
  result:='';
  case f of
     0: result:='Stability';
     1: result:='Blue flashing'+rsGPSoff;
     2: result:='Blue'+rsGPSlost;
     3: result:=fmAngle+' (Purple solid)';
     4: result:=fmAngle+' (Purple flashing)'+rsGPSoff;
     5: result:=fmAngle+' (Purple solid)'+rsGPSlost;
     6: result:=fmSmart;
     7: result:=fmSmart+rsGPSlost;
     8: result:='Motor start';
     9: result:='Temperature cali';
    10: result:='Pressure cali';
    11: result:='Accelerometer cali';
    12: result:='Emergency';
    13: result:='RTH coming';
    14: result:='RTH landing';
    15: result:='Bind';
    16: result:='Init';                    {Ready to start}
    17: result:='Wait on RC';
    18: result:='Mag cali';
    19: result:=rsUnknown;
    20: result:='Rate';                         {Rate}
    21: result:=fmSmart+' - Follow me';
    22: result:=fmSmart+' - Follow me'+rsGPSlost;
    23: result:=fmSmart+' - Camera tracking';
    24: result:='Camera tracking'+rsGPSlost;
    26: result:='Task Curve Cable Cam';
    27: result:='Task Journey';
    28: result:='Task Point of Interest';
    29: result:='Task Orbit';
    32: result:='IPS';                             {FMODE_ANGLE_MODE_IPS_ONLY:I = 0x20}
    33: result:='Waypoints';
  end;
end;

procedure GetInt123(data: TPayload; pos: integer; var v1, v2, v3: int16);
begin
  v1:=GetIntFromBuf(data, pos, 2);
  v2:=GetIntFromBuf(data, pos+2, 2);
  v3:=GetIntFromBuf(data, pos+4, 2);
end;

function  GetPackageCounter(data: TPayload): byte;
begin
  result:=0;
  if (data[3]=0) or (data[3]=3) or (data[3]=1) then   {Message ID for channel data}
    result:=data[7];
end;

end.
