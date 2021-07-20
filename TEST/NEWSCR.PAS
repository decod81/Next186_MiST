uses crt;
var
  a       : word;
  b, c    : byte;
  x, y, z : word;
  i, j, k : integer;
begin
  asm
    cli
    mov ax, 0dh
    int 10h
  end;

  port[$3d4] := $13;
  port[$3d5] := 976 shr 4; { 976 pixels wide }

  { fetch line compare bits 8 and 9 registers }
  port[$3d4] := $07; b := port[$3d5] and $ef;
  port[$3d4] := $09; c := port[$3d5] and $bf;

  portw[$3c4] := $0102; { memory plane }
  for x := 1 to 40-2 do
    for y := 8 to 200-9 do
      mem[$a000:x + y*122] := random(256);
  portw[$3c4] := $0202;
  for x := 1 to 122-2 do
    for y := 8 to 400-9 do
      mem[$a000:x + y*122] := random(256);
  portw[$3c4] := $0402;
  for x := 1 to 122-2 do
    for y := 8 to 400-9 do
      mem[$a000:x + y*122] := random(256);
  portw[$3c4] := $0802;
  for x := 1 to 122-2 do
    for y := 8 to 400-9 do
      mem[$a000:x + y*122] := random(256);

  x := 0;
  y := 0;
  z := 1;
  i := 1;
  j := 1;
  k := 2;
  repeat
    while (port[$3da] and 8)=0 do;
    while (port[$3da] and 8)<>0 do;

    inc(y, j);
    if (y = 0) or (y = 200) then
      j := -j;

    inc(x, i);
    if (x = 0) or (x = 660) then
      i := -i;

    inc(z, k);
    if (z = 1) or (z = 101) then
      k := -k;

    { line compare bits and pixel panning/shift }
    asm
      mov bx, z
      mov dx, 3d4h
      mov al, 18h
      mov ah, bl
      out dx, ax

      mov al, 7
      mov ah, b
      and bx, 100h
      jz @skip
      or ah, 10h
@skip:
      out dx, ax
      mov al, 9
      mov ah, c
      out dx, ax

      mov dx, 3c0h
      mov al, 30h
      out dx, al
      mov al, 31h
      out dx, al
      mov al, 33h
      out dx, al
      mov ax, x
      and al, 7
      out dx, al
    end;

    { start address high and low }
    a := y * 122 + x shr 3;
    asm
      mov bx, a
      mov dx, 3d4h
      mov al, 0ch
      mov ah, bh
      out dx, ax
      mov al, 0dh
      mov ah, bl
      out dx, ax
    end;
  until keypressed;

  asm
    sti
    mov ax, 3
    int 10h
  end;
end.