uses crt;
var
  x, y, z : word;
  i, j, k : integer;
begin
  asm
    cli
    mov ax, 13h
    int 10h
  end;

  x := 0;
  i := 1;
  repeat
    while (port[$3da] and 8)<>0 do;
    while (port[$3da] and 8)=0 do;
    for y := 0 to 199 do
      mem[$a000:x+y*320] := 0;
    inc(x, i);
    if x = 0 then i := -i;
    if x = 319 then i := -i;
    for y := 0 to 199 do
      mem[$a000:x+y*320] := 15;

    port[$3c8] := 0;
    port[$3c9] := 63;
    port[$3c9] := 0;
    port[$3c9] := 0;

    while (port[$3da] and 1)<>0 do;
    while (port[$3da] and 1)=0 do;
    port[$3c8] := 0;
    port[$3c9] := 0;
    port[$3c9] := 0;
    port[$3c9] := 0;

    while (port[$3da] and 1)<>0 do;
    while (port[$3da] and 1)=0 do;
    port[$3c8] := 0;
    port[$3c9] := 0;
    port[$3c9] := 63;
    port[$3c9] := 0;

    while (port[$3da] and 1)<>0 do;
    while (port[$3da] and 1)=0 do;
    port[$3c8] := 0;
    port[$3c9] := 0;
    port[$3c9] := 0;
    port[$3c9] := 0;

    while (port[$3da] and 1)<>0 do;
    while (port[$3da] and 1)=0 do;
    port[$3c8] := 0;
    port[$3c9] := 63;
    port[$3c9] := 63;
    port[$3c9] := 0;

    while (port[$3da] and 1)<>0 do;
    while (port[$3da] and 1)=0 do;
    port[$3c8] := 0;
    port[$3c9] := 0;
    port[$3c9] := 0;
    port[$3c9] := 0;
    for y := 0 to 99 do begin
      while (port[$3da] and 1)<>0 do;
      while (port[$3da] and 1)=0 do;
      port[$3c8] := 0;
      port[$3c9] := 63;
      port[$3c9] := 0;
      port[$3c9] := 63;

      while (port[$3da] and 1)<>0 do;
      while (port[$3da] and 1)=0 do;
      port[$3c8] := 0;
      port[$3c9] := 0;
      port[$3c9] := 0;
      port[$3c9] := 0;
    end;
  until x = 0;

  asm
    sti
    mov ax, 3
    int 10h
  end;
end.