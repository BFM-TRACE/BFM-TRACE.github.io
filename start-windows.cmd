@echo off
setlocal
cd /d "%~dp0"
where node >nul 2>nul
if errorlevel 1 (
  echo Node.js 20 or newer is required. Install Node.js, then reopen this launcher.
  pause
  exit /b 1
)

echo Open http://127.0.0.1:4173 in your browser.
echo Keep this window open. Press Ctrl+C to stop.
node server.mjs
set "TRACE_EXIT_CODE=%ERRORLEVEL%"
if not "%TRACE_EXIT_CODE%"=="0" pause
exit /b %TRACE_EXIT_CODE%
