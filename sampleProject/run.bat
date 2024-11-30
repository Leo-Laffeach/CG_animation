echo off

echo %~dp0

IF NOT "%~2"=="" (
  IF "%~2" == "ccr" (
    CALL :do_cmake
    CALL :do_make %~1
  ) ELSE (
    IF "%~2" == "cr" (
      CALL :do_make %~1
    )
  )
)
CALL :do_run "%~1"

EXIT /B %ERRORLEVEL%

:do_cmake
if not exist "%~dp0\..\sfmlGraphicsPipeline\build" mkdir "%~dp0\..\sfmlGraphicsPipeline\build"
cd "%~dp0\..\sfmlGraphicsPipeline\build"
cmake -G "MinGW Makefiles" ../

if not exist "%~dp0\build" mkdir "%~dp0\build"
cd "%~dp0\build"
cmake -G "MinGW Makefiles" ../

cd "%~dp0"

EXIT /B 0

:do_make
cd "%~dp0\..\sfmlGraphicsPipeline\build"
make CC=g++ -j6

cd "%~dp0\build"
make %~1 CC=g++ -j6
cd "%~dp0"
EXIT /B 0

:do_run
cd "%~dp0\build"
start %~dp0build\%~1.exe
cd "%~dp0"
EXIT /B 0
