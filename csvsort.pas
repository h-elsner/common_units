// https://www.lazarusforum.de/viewtopic.php?f=18&t=14829
// uses math, variants
 
type TSortKind = (soABC, soInteger, soFloat);

procedure SortCSV (var csv: TStringList; ADeli: char; ColIdx: Integer;
                   SortKind: TSortKind; Ascending: Boolean);

{$IFDEF WINDOWS}
const
  MinExtended:=3.4e-4932;
{$ENDIF}				   
				   
				   

var
  start, stop : Integer;

  function Str2Int(s: string): Int64;
  var 
    err : integer;
    
  begin
    val (s, result, err);
    if err <> 0 then 
      result := -MaxInt;                                // Error case, sorted as min
  end;

  function Str2Float(s: string): extended;
  var 
    err : integer;
    
  begin
    val (s, result, err);
    if err <> 0 then 
      result := MinExtended;                           // Error case, sorted as min
  end;

  function getIdx(row: integer): string;
  var 
    deli : TStringList;
    
  begin
    deli := TStringList.Create;
    deli.Delimiter:= ADeli;
    deli.StrictDelimiter:= true;
    deli.DelimitedText:= csv[row];
    result:= deli[ColIdx];
    deli.free;
  end;

  function SetVal(idx: Integer): variant;
  var 
    tmp:string;
    
  begin
    tmp:= getIdx (idx);
    case sortkind of
      soABC     : result := tmp;
      soInteger : result := Str2Int(tmp);
      soFloat   : result := Str2Float(tmp);
    end;                                                // case
  end;

  Function isGreater (i: Integer; v :variant): boolean;
  var mi: 
    variant;
    
  begin
    mi:= setVal(i);
    result := mi > v;
  end;

  Function isLess(i: Integer; v :variant): boolean;
  var 
    mi: variant;
    
  begin
    mi:= setVal(i);
    result := mi < v;
  end;

  procedure QuickSort(LoPara, HiPara: Integer);
  var
    Lo, Hi: Integer;
    PivotIdx: integer;
    Pivot : variant;
    
  begin
    Lo := LoPara;
    Hi := HiPara;
    PivotIdx := (Lo + Hi) div 2;
    Pivot := SetVal (PivotIdx);

    repeat
      if Ascending then begin
        while isLess (lo,pivot) do 
          inc (lo);
        while isGreater(hi,pivot) do 
          dec(hi);
      end else begin                                    // Decenting
        while isGreater(lo,pivot) do 
          inc (lo);
        while isLess (hi,pivot) do 
          dec(hi);
      end;

      if Lo <= Hi then begin
        csv.Exchange(lo,hi);
        Inc(Lo) ;
        Dec(Hi) ;
      end;
    until Lo > Hi;
    
    application.ProcessMessages;
    if Hi > LoPara then 
      QuickSort(LoPara, Hi) ;
    if Lo < HiPara then 
      QuickSort(Lo, HiPara) ;
  end;                                                  // quicksort

begin
  if csv.Count <= 1 then 
    exit;
  start := 0;
  stop := csv.Count-1;
  quicksort(start, stop);
end;  // SortCSV




