@echo off
c:\masm611\bin\ml /AT /c /Fl bios.asm
c:\masm611\bin\link /TINY bios,bios.com,,,,
del Next186.ROM
ren bios.com Next186.ROM
