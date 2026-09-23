@echo off
chcp 65001 >nul
title 三维模型查看与智能模具系统
cd /d "%~dp0web_box_generator"

echo ============================================================
echo   三维模型查看 + 智能模具生成系统
echo   端口: 5002      浏览器打开: http://127.0.0.1:5002
echo ------------------------------------------------------------
echo   注意：请在本窗口（普通 CMD 窗口）里运行服务。
echo   如果在"受限/沙箱化"的终端里启动，解析子进程池建不起来，
echo   上传模型会报 500（页面提示 Unexpected token ... is not valid JSON）。
echo ============================================================
echo.

"%~dp0venv\Scripts\python.exe" app.py --port 5002

echo.
echo [服务已退出] 按任意键关闭窗口...
pause >nul
