
@ECHO OFF
set ip=192.168.43.1
set port=5555

echo attempting to connect to ip %ip% on port %port%

set connectionString=%ip%:%port%

%LocalAppData%\Android\sdk\platform-tools\adb.exe connect %connectionString%