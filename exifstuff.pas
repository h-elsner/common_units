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


{Common EXIF definitions and routines

 Needed component: https://sourceforge.net/p/lazarus-ccr/svn/HEAD/tree/components/fpexif/
                   plus fpEXIF patch for Yuneec + XMP from wp_XYZ in r8204 (2022-03-09)
 See also: https://www.lazarusforum.de/viewtopic.php?f=18&t=13356

 EXIF tags:        https://exiftool.org/TagNames/EXIF.html }

unit exifstuff;                                  

{$mode objfpc}{$H+}

interface

uses
  sysutils, classes, fpeExifData, fpeMetaData, fpeTags, fpeUtils;

const
{Used EXIF tags}
  exMake=  'Make';
  exModel= 'Model';
  exSoftw= 'Software';
  exVersn= 'ExifVersion';
  exTime1= 'DateTimeOriginal';
  exTime2= 'DateTime';
  exTime3= 'DateTimeDigitized';
  exUser=  'UserComment';
  exOrientation='Orientation';
  exDescription='ImageDescription';
  exOwner= 'OwnerName';
  exSerial='SerialNumber';
  exCamTilt='CameraElevationAngle';

  exLat=   'GPSLatitude';
  exLon=   'GPSLongitude';
  exAlt=   'GPSAltitude';
  exSpeedRef='GPSSpeedRef';                        {'K' = km/h; 'M' = mph; 'N' = knots}
  exSpeed= 'GPSSpeed';
  exHeadingRef='GPSImgDirectionRef'; 	           {'M' = Magnetic North; 'T' = True North}
  exHeading='GPSImgDirectionRef';

  myVersion='0220';                                {EXIF version if EXIF was newly created}

{Public functions and procedures}
  function ReadTime(var RdData: TImgInfo;          {TimeFormat defines time sting}
                    TagName, timeformat, ErrorMsg: string): string;
  function GetEXIFtime(var RdData: TImgInfo): TDateTime;   {Get date/time from EXIF}
  function ReadString(var rddata: timginfo; tagname, errormsg: string): string;
  function ReadFloat(var rddata: timginfo; tagname: string): double;
  function ReadFloatAsString(var RdData: TImgInfo;
                             TagName, OutFormat, ErrorMsg: string): string;
  function ReadCoordinates(var RdData: TImgInfo; var nlat, nlon: double): boolean;

  procedure CreateStringTag(var WrData: TImgInfo; id, NewValue: string);
  procedure CreateFloatTag(var WrData: TImgInfo; id: string; NewValue: double);
  procedure CreateTimeTag(var WrData: TImgInfo; id: string; tme: TDateTime);
  procedure CreateMetadata(var WrData: TImgInfo;   {Create a new EXIF set}
                           nMaker, nModel: string;
                           tme1, tme2: TDateTime);

  procedure WriteTagAsString(var WrData: TImgInfo; {EXIF data set}
                           id, NewValue: string;   {Tag name and new string}
                           Overwrite: boolean=false);    {Name, value, allow}
  procedure WriteTagAsFloat(var WrData: TImgInfo;  {EXIF data set}
                            id: string;            {Tag name to write into}
                            NewValue: double;      {New value}
                            Overwrite: boolean=false);   {Name, value, allow}
  procedure WriteAltitude(var WrData: TImgInfo;    {EXIF data set}
                          alt: double;             {Cover negative valueus for Altitude}
                          ov: boolean=false);      {overwrite}
  procedure WriteCoordinates(var WrData: TImgInfo; const nlat, nlon: double;
                             ov: boolean=false);   {overwrite}
  procedure WriteEXIFTime(var WrData: TImgInfo;    {EXIF data set}
                          id: string;              {TimeTags}
                          tme: TDateTime;          {New time stamp}
                          Overwrite: boolean=false); {Overwrite or update if empty}
{ Handling XMP data in jpg files; needs fpEXIF from version r8202 on }
  procedure GetXMPlist(var XData: TImgInfo; Xlist: TStringList);
  procedure PutXMPlist(var XData: TImgInfo; XStream: TStream);
  procedure XMPListToStream(var Xlist: TStringList; XStream: TStream);

  function GetCoords(const lats, lons: string;     {Check and transform coordinates}
                     var la, lo: double): boolean;

implementation

function ReadTime(var RdData: TImgInfo;            {TimeFormat defines time sting}
                  TagName, timeformat, ErrorMsg: string): string;
var TagToRead: TTag;
begin
  result:=ErrorMsg;
  TagToRead:=RdData.ExifData.TagByName[TagName];
  if (TagToRead<>nil) and (TagToRead is TDateTimeTag) then
    result:=FormatDateTime(timeformat, TDateTimeTag(TagToRead).AsDateTime);
end;

function GetExifTime(var RdData: timginfo): tdatetime; {Get date/time from exif}
var timetag: ttag;
begin
  result:=0;                                       {invalid date far in the past as default}
  timetag:=rddata.exifdata.tagbyname[extime1];     {try mostly used tag}
  if timetag=nil then
    timetag:=rddata.exifdata.tagbyname[extime2];   {try another tag}
  if timetag=nil then
    timetag:=rddata.exifdata.tagbyname[extime3];
  if timetag is tdatetimetag then
    result:=tdatetimetag(timetag).asdatetime;
end;

function ReadString(var RdData: timginfo; tagname, errormsg: string): string;
var tagtoread: ttag;
begin
  result:=errormsg;                                {error message}
  tagtoread:=rddata.exifdata.tagbyname[tagname];
  if tagtoread<>nil then                           {check if tag is available}
    result:=tagtoread.asstring;
end;

function ReadFloat(var RdData: timginfo; tagname: string): double;
var tagtoread: ttag;
begin
  result:=0;
  tagtoread:=rddata.exifdata.tagbyname[tagname];
  if tagtoread<>nil then                           {check if tag is available}
    result:=tagtoread.asfloat;
end;

function ReadFloatAsString(var RdData: TImgInfo;
                           TagName, OutFormat, ErrorMsg: string): string;
var tagtoread: ttag;
begin
  result:=ErrorMsg;
  TagToRead:=RdData.ExifData.TagByName[TagName];
  if TagToRead<>nil then                           {Check if tag is available}
    result:=FormatFloat(OutFormat, TagToRead.AsFloat);
end;

function ReadCoordinates(var RdData: TImgInfo; var nlat, nlon: double): boolean;
begin
  result:=false;
  try
    nlat:=rdData.ExifData.GPSLatitude;
    nlon:=rdData.ExifData.GPSLongitude;
  except
    on e: Exception do begin
      nlat:=0;
      nlon:=0;
    end;
  end;
  result:=(nlat<>0) or (nLon<>0);                  {Valid coordinates}
end;

{Use Create tasks only if CreateMetaData was already done!}
procedure CreateStringTag(var WrData: TImgInfo; id, NewValue: string);
var newtag: TTag;
begin
  newtag:=WrData.EXIFdata.AddTagByName(id);
  newtag.AsString:=NewValue;
end;

procedure CreateFloatTag(var WrData: TImgInfo; id: string; NewValue: double);
var newtag: TTag;
begin
  newtag:=WrData.EXIFdata.AddTagByName(id);
  newtag.AsFloat:=NewValue;
end;

procedure CreateTimeTag(var WrData: TImgInfo; id: string; tme: TDateTime);
var newtag: TTag;
begin
  if tme>1 then begin
    newtag:=WrData.EXIFdata.AddTagByName(id);
    if newtag is TDateTimeTag then                 {Write date/time to EXIF}
      TDateTimeTag(newtag).AsDateTime:=tme;
  end;
end;

{Has to be done first}
procedure CreateMetadata(var WrData: TImgInfo;     {Create a new EXIF set}
                         nMaker, nModel: string;
                         tme1, tme2: TDateTime);
var newtag: TTag;
begin
  newtag:=WrData.CreateExifData().AddTagByName(exMake);
  newtag.AsString:=nMaker;                         {Manufacturer}
  if nModel<>'' then
    CreateStringTag(WrData, exModel, nModel);      {Camera type}
  CreateTimeTag(WrData, exTime1, tme1);            {Write original date/time to EXIF}
  CreateTimeTag(WrData, exTime2, tme2);            {Write date/time to EXIF}
  CreateStringTag(WrData, exVersn, myVersion);     {EXIF version}
end;

procedure WriteTagAsString(var WrData: TImgInfo;   {EXIF data set}
                         id, NewValue: string;     {Tag name and new string}
                         Overwrite: boolean=false);{Name, value, allow}
var WantedTag: TTag;
begin
  WantedTag:=WrData.ExifData.TagByName[id];
  if WantedTag=nil then begin                      {Check if tag is missing}
    CreateStringTag(WrData, id, NewValue);
  end else begin                                   {If tag was already there}
    if Overwrite then
      WantedTag.AsString:=NewValue;                {Overwrite value if allowed}
  end;
end;

procedure WriteTagAsFloat(var WrData: TImgInfo;    {EXIF data set}
                          id: string;              {Tag name to write into}
                          NewValue: double;        {New value}
                          Overwrite: boolean=false);   {Name, value, allow}
var WantedTag: TTag;
begin
  WantedTag:=WrData.ExifData.TagByName[id];
  if WantedTag=nil then begin                      {Check if tag is missing}
    CreateFloatTag(WrData, id, NewValue);
  end else begin                                   {If tag was already there}
    if Overwrite then
      WantedTag.AsFloat:=NewValue;                 {Overwrite value if allowed}
  end;
end;

procedure WriteAltitude(var WrData: TImgInfo;      {EXIF data set}
                        alt: double;               {Cover negative valueus for Altitude}
                        ov: boolean=false);        {overwrite}
var WantedTag: TTag;
begin
  if ov then begin
    wrData.EXIFdata.GPSAltitude:=alt;
  end else begin
    WantedTag:=WrData.ExifData.TagByName[exAlt];
    if WantedTag=nil then
      wrData.EXIFdata.GPSAltitude:=alt;            {Will be automatically created}
  end;
end;

procedure WriteCoordinates(var WrData: TImgInfo; const nlat, nlon: double;
                           ov: boolean=false);     {overwrite}
var WantedTag: TTag;
begin
  if ov then begin                                 {Overwrite}
    wrData.EXIFdata.GPSLatitude:=nlat;
    wrData.EXIFdata.GPSLongitude:=nlon;
  end else begin                                   {Write only if missing}
    WantedTag:=WrData.ExifData.TagByName[exLat];
    if WantedTag=nil then
      wrData.EXIFdata.GPSLatitude:=nlat;           {Will be automatically created}
    WantedTag:=WrData.ExifData.TagByName[exLon];
    if WantedTag=nil then
      wrData.EXIFdata.GPSLongitude:=nlon;
  end;
end;

procedure WriteEXIFTime(var WrData: TImgInfo;      {EXIF data set}
                        id: string;                {TimeTags}
                        tme: TDateTime;            {New time stamp}
                        Overwrite: boolean=false); {Overwrite or update if empty}
var WantedTag: TTag;
begin
  if tme>1 then begin
    WantedTag:=WrData.ExifData.TagByName[id];
    if WantedTag=nil then begin                    {Check if tag is missing}
      WantedTag:=WrData.EXIFdata.AddTagByName(id);
      if WantedTag is TDateTimeTag then
        TDateTimeTag(WantedTag).AsDateTime:=tme;   {Insert the new value}
    end else begin                                 {If tag was already there}
      if (WantedTag is TDateTimeTag) and
         (Overwrite or                             {Overwrite is allowed or tag empty}
         (TDateTimeTag(WantedTag).AsDateTime<1)) then
        TDateTimeTag(WantedTag).AsDateTime:=tme;   {Write time stamp into}
    end;
  end;
end;

{ Handling XMP data in jpg files; needs fpEXIF from version r8203 on }
procedure GetXMPlist(var XData: TImgInfo; Xlist: TStringList);
var
  XStream: TStream;
begin
  XStream:=TMemoryStream.Create;
  try
    Xlist.TextLineBreakStyle:=tlbsLF;              {#10 as line ending}
    Xdata.XmpData.SaveToStream(XStream);
    XStream.Position:=0;
    Xlist.LoadFromStream(XStream, true);
  finally
    XStream.Free;
  end;
end;

procedure PutXMPlist(var XData: TImgInfo; XStream: TStream);
begin
  XStream.Position:=0;
  XData.XmpData.LoadFromStream(XStream);
  XStream.Position:=0;
end;

procedure XMPListToStream(var Xlist: TStringList; XStream: TStream);
begin
  XStream.Size:=0;
  XStream.Position:=0;
  Xlist.SaveToStream(XStream, true);
  XStream.Position:=0;
  XStream.Size:=XStream.Size-1;                    {Remove last line ending}
end;

function GetCoords(const lats, lons: string;       {Coordinates as strings}
                   var la, lo: double): boolean;   {to Coordinates as float}
begin
  la:=StrToFloatDef(lats, 0);
  lo:=StrToFloatDef(lons, 0);
  result:=(la<>0) or (lo<>0);                      {Valid coordinates}
end;

end.



