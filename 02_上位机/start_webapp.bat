@echo off
rem ===================================================================
rem  网页控制台 一键启动（★蓝牙必须走 http://localhost，不能双击 html 打开）
rem  用法：双击本文件 -> 浏览器自动打开 http://localhost:8000/
rem  关闭：在本窗口按 Ctrl+C 或直接关掉窗口
rem ===================================================================
cd /d "%~dp0"
set PY=C:\Users\POLIARS\.workbuddy\binaries\python\envs\default\Scripts\python.exe
if not exist "%PY%" set PY=python
echo.
echo   网页控制台已启动： http://localhost:8000/
echo   蓝牙（Web Bluetooth）只在 localhost / https 下可用，请用这个地址
echo   停止服务：按 Ctrl+C
echo.
start "" http://localhost:8000/index.html
"%PY%" -m http.server 8000 --bind 127.0.0.1
