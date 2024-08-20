{Directory List Edit helper procedures
 
 Like TDirectoryEdit but with ComboBox instead of edit field
 to store an amount of directories

}

unit directorylistedit;

{$mode objfpc}{$H+}

interface

uses
  classes, StdCtrls, Buttons;

const
  DefaultMax=12;
  btnspace=4;
  
  
procedure DirItems(cbx: TComboBox;                  {Items in ComboBox}
                   maxAnzahl: integer=DefaultMax;   {Number of stored directories}
                   sorted: boolean = false);        {List sorted or not}
procedure SizeSpeedBtn(var btn: TSpeedbutton;
                           cbx: TComboBox);         {Arrange button related to ComboBox}

implementation

procedure DirItems(cbx: TComboBox; maxAnzahl: integer; sorted: boolean = false);
var
  tmplist: TStringList;

begin                                               {Fill drop-down list}
  if (cbx.Text<>'') and
     (cbx.Items.IndexOf(cbx.Text)<0) then           {Only if not yet in item list}
    cbx.Items.Insert(0, cbx.Text);
  if cbx.Items.Count>MaxAnzahl then                 {Limit bumber items in list}
    cbx.Items.Delete(MaxAnzahl);
  if sorted then begin                              {Sort list if needed}
    tmplist:=TStringlist.Create;
    try
    tmplist.Assign(cbx.Items);
    tmplist.Sort;
    cbx.Items.Assign(tmplist);
    finally
      tmplist.Free;
    end;
  end;
end;

procedure SizeSpeedBtn(var btn: TSpeedbutton; cbx: TComboBox);  {Arrange button related to ComboBox}
begin
  btn.Height:=cbx.Height;
  btn.Width:=btn.Height;
  btn.Top:=cbx.Top;
  btn.Left:=cbx.Left+cbx.Width+btnspace;
end;

end.
