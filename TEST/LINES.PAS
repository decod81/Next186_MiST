uses crt;
var
  i : integer;
  x : word;
  y : byte;
begin
  asm
    mov ax, 13h
    int 10h
  end;
  for y := 0 to 199 do begin
    mem[$a000:0+y*320] := 13;
    mem[$a000:319+y*320] := 13;
  end;
  for x := 0 to 319 do begin
    mem[$a000:x] := 13;
    mem[$a000:x+199*320] := 13;
  end;
  x := 2;
  i := 1;
  repeat
    while (port[$3da] and 8)<>0 do;
    while (port[$3da] and 8)=0 do;
    for y := 0 to 199 do
      mem[$a000:x+y*320] := 2;
    if x = 318 then i := -i;
    if x = 1 then i := 0;
    inc(x, i);
    for y := 0 to 199 do
      mem[$a000:x+y*320] := 15;
  until keypressed;
  asm
    mov ax, 3
    int 10h
  end;
end.