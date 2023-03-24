@echo off
setlocal enabledelayedexpansion
for /r %%f in (RAW\*.PGM) do (
  if not exist DCIM_%%~nf.JPG (
    convert -scale 512x504 %%f DCIM_%%~nf.JPG
  )
)
endlocal
