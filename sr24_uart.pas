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
                                                                                      
                                                                                      
{From ST24.h and ST24.c PX4 Autopilot

 Preparations:
 -------------
 Add to /boot/config.txt:
 [Switch-off bluetooth to get serial]
  dtoverlay=pi3-disable-bt

 Switch-off UART console:
 sudo raspi-config > Interface Options > Serial port >
 "Would you like a login shell to be accessible over serial?" --> No
 "Would you like the serial port hardware to be enabled?" --> Yes


 Read data from SR24
 -------------------

 Uses non-standard package "Synapse" (install per Online package manager).
 Then open package file laz_synapse.lpk and Add to project.

}

unit sr24_uart;

{$mode objfpc}{$H+}

interface

uses
  synaser, SR24_dec;

const
  timeout=300;
  UARTSpeed=115200;                                     {SR24 default speed}
  uartport='/dev/ttyAMA0';                              {Default port for Raspberry Pi}
  USBport='/dev/ttyUSB0';                               {Default port for Serial to USB converter}

var
  sr24ser: TBlockSerial;                                {UART access}

function  ConnectUART(port: string; speed: uint32; var UARTconnected: boolean): string;
procedure DisconnectUART(var UARTconnected: boolean);   {Disconnect SR24}
function  UARTcanRead: boolean;                         {Check if ready to receive}
function  UARTcanWrite: boolean;                        {Check if ready to transmit}
function  UARTreadByte: byte;                           {receive one byte}
procedure UARTwriteByte(b: byte);                       {send one byte}
function  UARTreadMsg(var data: TPayLoad): boolean;     {Read one message}
procedure UARTsendMsg(data: TPayload);                  {Send one telemetry dataset}
procedure SendBind;                                     {Send one binding message}

implementation

function ConnectUART(port: string; speed: uint32; var UARTconnected: boolean): string;
begin
  result:='';
  if not UARTconnected then begin                       {UART Tx, GPIO 14, pin 8}
    sr24ser:=TBlockSerial.Create;                       {UART Rx, GPIO 15, pin 10}
    {$ifdef Linux}
      sr24ser.LinuxLock:=false;
    {$endif}
    sr24ser.Connect(port);                              {Port for Raspi: /dev/ttyAMA0}
//    sleep(50);
    sr24ser.Config(speed, 8, 'N', SB1, false, false);   {Config default 115200 baud, 8N1}
    if SR24ser.LastError=0 then
      UARTConnected:=true;
    result:='Device: '+SR24ser.Device+' Status: '+SR24ser.LastErrorDesc;
  end;
end;

procedure DisconnectUART(var UARTconnected: boolean);   {Disconnect and free UART}
begin
  if UARTConnected then begin
    try
      sr24ser.CloseSocket;                              {Close UART connection}
    finally
      sr24ser.Free;
    end;
    UARTConnected:=false;
  end;
end;

function UARTcanRead: boolean;                          {Wrapper for simple UART routines}
begin
  result:=sr24ser.CanRead(timeout);
  result:=sr24ser.CanReadEx(timeout);
end;

function UARTcanWrite: boolean;
begin
  result:=sr24ser.CanWrite(timeout);
end;

function UARTreadByte: byte;
begin
  result:=sr24ser.RecvByte(timeout);
end;

procedure UARTwriteByte(b: byte);
begin
  sr24ser.SendByte(b);
end;

function UARTreadMsg(var data: TPayLoad): boolean;      {Detect and read one message from data stream}
const
  empty: array [0..2] of byte = (255, 255, 255);        {Buffer for header bytes to check if message starts here}

var
  i, z: byte;
  buf: array[0..2] of byte;

begin
  result:=false;
  z:=0;                                                 {Counter for unsynced bytes}
  buf:=empty;                                           {Reset buffer}
  repeat
    buf[0]:=UARTreadByte;                               {read byte by byte}
    if (buf[2]=header1) and                             {check if valid message (header+plausible length)}
       (buf[1]=header2) and
       (buf[0]<maxlen) and                              {Check message length}
       (buf[0]>0) then begin
      data[0]:=buf[2];                                  {Copy header and length to message data}
      data[1]:=buf[1];
      data[2]:=buf[0];
      for i:=3 to buf[0]+2 do                           {Read the other bytes of the dataset (payload + CRC)}
        data[i]:=UARTreadByte;
      z:=0;
      result:=true;
    end else begin                                      {Shift buffer right}
      buf[2]:=buf[1];
      buf[1]:=buf[0];
      inc(z);                                           {Count bytes to prevent overflow}
    end;
  until result or                                       {Valid message but w/o CRC check}
       (z>maxlen);                                      {Too long message}
end;

procedure UARTsendMsg(data: TPayload);                  {Send one telemetry dataset}
var
  crc, i: byte;

begin
  for i:=0 to data[2]+1 do
    sr24ser.SendByte(data[i]);
  crc:=SR24_CRC8(data, data[2]);
  sr24ser.SendByte(crc);
end;

procedure SendBind;                                     {Send one BIND message}
var
  i: integer;

begin
  if sr24ser.CanWrite(timeout) then begin
    for i:=0 to 10 do begin
      sr24ser.SendByte(BindMessage[i]);
    end;
  end;
end;

end.
