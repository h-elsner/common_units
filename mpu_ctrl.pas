{MPU6050 control 

 Read/write register of MPU6050 by terminal commands at Raspberry Pi
 --------------------------------------------------------------------------------

 Descrition of register
 https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

 Preparations:
 -------------
 Enable I2C: sudo raspi-config > Interface Options > I2C > Yes
 Possibly you need user access:
 sudo usermod -aG i2c pi (pi or any other user)

 Connect MPU and check wiring (bus 1).
 pin1  Vcc
 pin3  SDA
 pin5  SCL
 pin6  GND

 Check if MPU6050 is available:	i2cdetect -y 1 > should be appear at address 0x68

 Some useful terminal commands:
 ------------------------------
 Read a byte from MPU:	i2cget -y 1 0x68 0x75 (Who am I, it's own address)
 Read a word from MPU:	i2cget -y 1 0x68 65 w (result comes as big endian)
 Write a byte to MPU register:	i2cset y 1 0x68 107 0 (wake-up command)
 Read temperature cyclic (raw):	watch -n 0.5 'i2cget -y 1 0x68 65 w'

 #############################################################################

 https://www.raspberry-pi-geek.de/ausgaben/rpg/2016/04/teil-9-analog-digital-wandler-pcf8591/

 Control byte PCF8591
 7 6 5 4 3 2 1 0
 0 | +-+ 0 |  + + --- Auswahl des A/D-Kanals. 00: Kanal 0, 01: Kanal 1, 10: Kanal 2, 11: Kanal 3
   |   |   +--------- Auto-Inkrement-Funktion (1 = aktiv)
   |   +------------- Programmieren der Analogkanäle (Details siehe Datenblatt, Fig. 4)
   +----------------- Analog-Output aktivieren (1 = aktiv)


 DAC output:
 i2cset -y 1 0x48 0x40 w (w ist 0x00 bis 0xFF)

 ADC 3  einlesen (Befehl muss zweimal eingeben werden,
                 einmal zum Setzen des Registers und das zweite mal, um auszulesen.
                 Das erste Ergebnis ist die vorherige Messung,
                 das zweite Ergebnis ist der aktuelle Wert):
 i2cget -y 1 0x48 0x43
}

unit mpu_ctrl;

{$mode objfpc}{$H+}

interface

uses
   sysutils, strutils, process;

const
  MPUadr='0x68';                                   {IMU MPU6050}
  ISTadr='0x0E';                                   {Compass}
  ISTID= '0x10';
  HMCadr='0x1E';                                   {Compass Q500}
  AS5adr='0x36';                                   {Contactless poti AS5600}
  ADCadr='0x48';                                   {ADC PFC8591  0x48 .. 0x4F possible}
  BMPadr='0x77';							       {BMP280 Pressure sensor}
  intfac='I²C';

  bus='1';                                         {Used I2C bus, default}
{Read/write register MPU6050}
  rwregs=[13..16, 25..28, 35..52, 55, 56, 99..104, 106..108, 114..116];
  roregs=[53, 54, 58..96, 117];                    {Read-only registers}
  rwIST=[10..12, 65, 66];                          {Register IST8310}
  roIST=[0, 2..9, 28, 29];
  rwHMC=[0..2];                                    {Register HMC5883}
  roHMC=[3..12];
  rst107=$40;                                      {Reset value for power management}

  rwpAS5=[1..8];
  roAS5=[0, 11..15, 26..28];
  rwAS5=255;                                       {Write: Burn_Angle=$80, Burn_Setting=$40}

  i2cdct='i2cdetect';
  i2cget='i2cget';
  i2cset='i2cset';
  i2cdump='i2cdump';
  yes='-y';                                        {Disable interactive mode}
  hexidc='0x';
  hexidp='$';

  ADCVdd=3.3;

function XtoByte(w: string): byte;
function GetAdrStrMPU: boolean;                    {Check MPU address from register WHO_AM_I}
function GetAdrStrIST: boolean;                    {Check IST8310 address from register WHO_AM_I}
function GetAdrStrHMC: boolean;                    {Check HMC5883 address frm ID register A}
function GetAdrStrAS5: boolean;                    {Check AS5600 address if burn is 0}
function GetAdrStrADC(adr: string): boolean;       {Check PFC8591 addresses $48 - $4F}
function GetReg(adr: string; r: byte): byte;       {Read byte from MPU}
function GetRegWbe(adr: string; r: byte): int16;   {Read word from MPU}
function GetRegWle(adr: string; r: byte): int16;   {Read word from IST little endian}
procedure SetReg(adr: string; r, v: byte);         {Write byte v to register r}
procedure MPUWakeUp;                               {Power management set to wake up}
procedure ISTreset;                                {Soft reset IST8310}
procedure HMCinit;                                 {Derfault initialisation for continous masurement}
function GetFS_SEL: byte;                          {Read scale factor for gyro (27)}
function GetAFS_SEL: byte;                         {Read scale factor for acc (28)}
function GetGain: byte;                            {Get gain from HMC588s, ConfReg B}
function ConvTemp(temp: int16): double;            {Convert temperature}
function TempToStr: string;                        {Show chip temperature as string}
function ConvGyro(fs: byte; gy: int16): double;    {Convert gyro values according datasheet}
function ConvAcc(afs: byte; acc: int16;            {Convert acc values according datasheet}
                 mg: boolean=false): double;       {alternativ: output in mG}
function ConvHMC(raw: int16; gain: byte): double;  {Convert raw to Gauss}
function GainToStr(gain: byte): string;            {Just for information about gain}
function afsToStr(afs: byte): string;              {Just for information about used acc scale}
function fsToStr(fs: byte): string;                {Just for information about used gyro scale}
function AdrToChip(adr: string): string;           {Find chip type on I2C bus 1}
function ScanI2C(intf: char='1'): string;          {Scan a I2C interface to find all connected chips}
function ReadADC(adr: string; ch: byte): byte;     {Read ADC channel, value between 0 and 255}
function SetDAC(adr: string; w: byte): boolean;    {Set output DAC to value between 0 and 255}
function GetVolt(w: byte; factor: double=1.0): double;

implementation

function XtoByte(w: string): byte;
var
  s: string;

begin
  s:=ReplaceText(trim(w), hexidc, hexidp);
  result:=StrToIntDef(s, 255);
end;

function GetAdrStrMPU: boolean;                    {Check MPU address from register WHO_AM_I}
var
  s: string;

begin
  s:='';
  RunCommand(i2cdct, [yes, bus], s);
  RunCommand(i2cget, [yes, bus, MPUadr, '117'], s);
  result:=trim(s)=MPUadr;
end;

function GetAdrStrIST: boolean;                    {Check IST8310 address from register WHO_AM_I}
var
  s: string;

begin
  s:='';
  RunCommand(i2cdct, [yes, bus], s);
  RunCommand(i2cget, [yes, bus, ISTadr, '0'], s);
  result:=trim(s)=ISTID;
end;

function GetAdrStrHMC: boolean;                    {Check HMC5883 address frm ID register A}
var
  s: string;

begin
  s:='';
  RunCommand(i2cdct, [yes, bus], s);
  RunCommand(i2cget, [yes, bus, HMCadr, '0x0A'], s);
  result:=trim(s)='0x48';
end;

function GetAdrStrAS5: boolean;                    {Check AS5600 address if burn is 0}
var
  s: string;

begin
  s:='';
  RunCommand(i2cdct, [yes, bus], s);
  RunCommand(i2cget, [yes, bus, AS5adr, '0xFF'], s);
  result:=trim(s)='0x00';
end;

function GetAdrStrADC(adr: string): boolean;       {Check PFC8591 addresses $48 - $4F}
var
  s: string;

begin
  s:='';
  RunCommand(i2cget, [yes, bus, adr, '0x40'], s);  {try to read DAC}
  result:=(pos(hexidc, s)=1);
end;

function GetReg(adr: string; r: byte): byte;       {Read byte from MPU}
var
  s: string;

begin
  RunCommand(i2cget, [yes, bus, adr, IntToStr(r)], s);
  result:=XtoByte(s);
end;

function GetRegWbe(adr: string; r: byte): int16;   {Read word from MPU}
var
  s: string;
  w: int16;

begin
  RunCommand(i2cget, [yes, bus, adr, IntToStr(r), 'w'], s);
//  RunCommand(i2cget, [yes, bus, adr, IntToStr(r or $80), 'w'], s);
  s:=ReplaceText(trim(s), hexidc, hexidp);         {Word from i2cget is big endian}
  w:=StrToIntDef(s, $FFFF);
  result:=BEtoN(w);
end;

function GetRegWle(adr: string; r: byte): int16;   {Read word from IST little endian}
var
  s: string;

begin
  RunCommand(i2cget, [yes, bus, adr, IntToStr(r), 'w'], s);
  s:=ReplaceText(trim(s), hexidc, hexidp);
  result:=StrToIntDef(s, $FFFF);
end;

procedure SetReg(adr: string; r, v: byte);         {Write byte v to register r}
var
  s: string;

begin
  RunCommand(i2cset, [yes, bus, adr, IntToStr(r), IntToStr(v)], s);
end;

procedure MPUWakeUp;                               {Power management set to wake up}
begin
  SetReg(MPUadr, 107, 0);
end;

procedure ISTreset;                                {Soft reset IST8310}
begin
  SetReg(ISTAdr, 11, 13);                          {Control register2 Soft reset}
end;

procedure HMCinit;                                 {Default initialisation for continous masurement}
begin
  SetReg(HMCadr, 0, $10);                          {Defaults: 1 sample, 15Hz, normal conf}
  SetReg(HMCadr, 1, $20);                          {Default gain 1: +/-1.3Gauss}
  SetReg(HMCadr, 2, 0);                            {Continuous mode}
end;

function GetFS_SEL: byte;                          {Read scale factor for gyro}
var
  b: byte;

begin
  b:=GetReg(MPUadr, 27);                           {Gyro_CONFIG}
  result:=(b and $18) shr 3;                       {Gyro Scale 0..3}
end;

function GetAFS_SEL: byte;                         {Read scale factor for Acc}
var
  b: byte;

begin
  b:=GetReg(MPUadr, 28);                           {Acc_CONFIG}
  result:=(b and $18) shr 3;                       {Acc Scale 0..3}
end;

function GetGain: byte;                            {Get gain from HMC588s, ConfReg B}
var
  b: byte;

begin
  b:=GetReg(HMCadr, 1);
  result:=(b and $E0) shr 5;                       {gain 0..7}
end;

{Temperature in °C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53}
function ConvTemp(temp: int16): double;
begin
  result:=temp/340+36.53;
end;

function TempToStr: string;                        {Show chip temperature as string}
begin
  result:=FormatFloat('0.0', ConvTemp(GetRegWbe(MPUadr, 65)))+'°C';
end;

function ConvGyro(fs: byte; gy: int16): double;    {Convert gyro values according datasheet}
begin
  case fs of
    0: result:=gy/131;                             {+/- 250°/s}
    1: result:=gy/65.5;
    2: result:=gy/32.8;
    3: result:=gy/16.4;                            {+/- 2000 °/S}
  end;
end;

function ConvAcc(afs: byte; acc: int16;            {Convert acc values according datasheet}
                 mg: boolean=false): double;       {output in mG}
begin
  case afs of
    0: result:=acc/16384;                          {+/- 2G}
    1: result:=acc/8192;
    2: result:=acc/4096;
    3: result:=acc/2048;                           {+/- 16G}
  end;
  if mg then
    result:=result*1000;
end;

function ConvHMC(raw: int16; gain: byte): double;  {Convert raw to Gauss, gain 0..7}
begin
  case gain of
    0: result:=raw/1370;
    1: result:=raw/1090;                           {Default}
    2: result:=raw/820;
    3: result:=raw/660;
    4: result:=raw/440;
    5: result:=raw/390;
    6: result:=raw/330;
    7: result:=raw/230;
  else
    result:=0;
  end;
end;

function afsToStr(afs: byte): string;              {Just for information about used acc scale}
begin
  case afs of
    0: result:='+/- 2G';
    1: result:='+/- 4G';
    2: result:='+/- 8G';
    3: result:='+/- 16G';
  end;
end;

function GainToStr(gain: byte): string;            {Just for information about gain}
begin
  case gain of
    0: result:='+/- 0.88Ga';
    1: result:='+/- 1.3Ga';                        {Default}
    2: result:='+/- 1.9Ga';
    3: result:='+/- 2.5Ga';
    4: result:='+/- 4.0Ga';
    5: result:='+/- 4.7Ga';
    6: result:='+/- 5.6Ga';
    7: result:='+/- 8.1Ga';
  else
    result:='';
  end;
end;

function AdrToChip(adr: string): string;           {Find chip type from address}
var
  a, d: integer;

begin
  result:='unknown';
  if adr=MPUadr then begin
    result:='MPU6050';                             {Gyro, Acc}
    exit
  end;
  if adr=ISTadr then begin
    result:='IST8310';                             {Magnetometer}
    exit
  end;
  if adr=HMCadr then begin                         {Magnetometer}
    result:='HMC5883';
    exit;
  end;
  if adr=AS5adr then begin
    result:='AS5600';                              {Magnetic rotary position}
  end;
  a:=XtoByte(adr);
  d:=XtoByte(ADCadr);
  if (a and d)=d then begin
    result:='PCF8591';                             {4-channel ADC, 1-channel DAC}
  end;
end;

function fsToStr(fs: byte): string;                {Just for information about used gyro scale}
begin
  case fs of
    0: result:='+/- 250°/s';
    1: result:='+/- 500°/s';
    2: result:='+/- 1000°/s';
    3: result:='+/- 2000°/s';
  end;
end;

function ScanI2C(intf: char=bus): string;          {Scan a I2C interface to find all connected chips}
var
  i: byte;
  s, adr: string;

begin
  result:='';
  RunCommand(i2cdct, [yes, intf], s);
  for i:=3 to 127 do begin
    adr:=hexidc+IntToHex(i, 2);                    {Test all possible addresses}
    RunCommand(i2cget, [yes, intf, adr, '0'], s);
    if pos(hexidc, s)>0 then
      result:=result+adr+' ';
  end;
  result:=trim(result);
end;

function ReadADC(adr: string; ch: byte): byte;     {Read ADC channel, value between 0 and 255}
var
  s: string;

begin
  RunCommand(i2cget, [yes, bus, adr, '0x4'+IntToStr(ch and 3)], s);  {Init channel}
  RunCommand(i2cget, [yes, bus, adr, '0x4'+IntToStr(ch and 3)], s);  {Get value}
  result:=XtoByte(s);
end;

function SetDAC(adr: string; w: byte): boolean;    {Set output DAC to value between 0 and 255}
var
  s: string;

begin
  RunCommand(i2cset, [yes, bus, adr, '0x40', '0x'+IntToHex(w, 2)], s);
  result:=trim(s)='';
end;

function GetVolt(w: byte; factor: double=1.0): double;
begin
  result:=factor*ADCVdd*w/255;
end;

end.
