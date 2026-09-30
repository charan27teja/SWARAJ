@echo off
REM Resolve the Python interpreter: project venv first, then PATH.
if exist "%~dp0..\.venv\Scripts\python.exe" (set "PY=%~dp0..\.venv\Scripts\python.exe") else (set "PY=python")
