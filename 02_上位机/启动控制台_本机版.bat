@echo off
rem ============================================================
rem  TASK56 控制台启动器（本机适配版）
rem  ----------------------------------------------------------
rem  用途：起一个静态服务，把 webapp_task56\index.html 用
rem        http://localhost:8001 打开（Web Bluetooth 只认
rem        localhost 或 https，双击 html 文件连不上蓝牙）。
rem
rem  为什么不用组长那两个 .bat：
rem    · webapp\start_webapp.bat      写死 %USERPROFILE%\.cache\
rem      codex-runtimes\...python.exe，本机没有
rem    · 一键启动控制台_TASK56.bat    写死 C:\Users\Administrator\
rem      ...Xiaomi MiMo\...python.exe，本机路径不同
rem  本版本按「where python / py」顺序查找，并额外兜底
rem  WorkBuddy 内置 Python，本机实测可用。
rem
rem  端口 8001（避让我方主控制台的 8000）。
rem  注意：蓝牙同一时刻只能被一个页面持有——
rem        用这个页面之前先把端口 8000 的那个页面关掉或断开。
rem ============================================================
setlocal enabledelayedexpansion
title TASK56 Console - port 8001

cd /d "%~dp0"
if not exist "index.html" (
  echo [ERROR] index.html not found next to this .bat
  echo         ^(this .bat must sit inside webapp_task56\^)
  pause
  exit /b 1
)

rem ---- 已经在跑？直接开页面 ----
netstat -ano | findstr /R /C:":8001 .*LISTENING" >nul 2>nul
if not errorlevel 1 (
  echo Server already on port 8001 - opening the page.
  start "" "http://localhost:8001/index.html"
  timeout /t 2 >nul
  exit /b 0
)

rem ---- 按优先级查找 Python ----
set "PY="
where python >nul 2>nul && set "PY=python"
if not defined PY ( where py >nul 2>nul && set "PY=py" )
if not defined PY (
  if exist "C:\Users\POLIARS\.workbuddy\binaries\python\versions\3.13.12\python.exe" (
    set "PY=C:\Users\POLIARS\.workbuddy\binaries\python\versions\3.13.12\python.exe"
  )
)
if not defined PY (
  echo [ERROR] Python 3 not found.
  echo         Tried: python, py, and the WorkBuddy bundled runtime.
  pause
  exit /b 1
)

echo.
echo   Python  : !PY!
echo   Console : http://localhost:8001/index.html
echo.
echo   Chrome / Edge required.  Keep this window open.
echo   IMPORTANT: close or disconnect the baseline page (port 8000)
echo              first - two pages cannot hold the BLE link at once.
echo.

rem ---- 延迟 4 秒再开浏览器，等服务绑定端口 ----
start "" /b cmd /c "timeout /t 4 >nul & start "" http://localhost:8001/index.html"

"!PY!" -m http.server 8001 --bind 127.0.0.1

echo.
echo Server stopped. If the page showed ERR_CONNECTION_REFUSED, press F5.
pause
