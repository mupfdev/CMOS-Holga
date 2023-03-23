@echo off
setlocal enabledelayedexpansion
for /r %%f in (RAW\*.PGM) do (
  if not exist DCIM_%%~nf.JPG (
    convert %%f DCIM_%%~nf.JPG
  )
)
endlocal
