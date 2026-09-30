@echo off
REM All tests: pytest (unit + fast-runtime scenarios + live processes) and Playwright UI smoke tests.
setlocal
call "%~dp0_py.cmd"
pushd "%~dp0.."
set "PYTHONPATH=%CD%"
"%PY%" -m pytest
if errorlevel 1 goto :end
pushd dashboard
if not exist dist\index.html call npm run build
call npx playwright test
popd
:end
popd
endlocal
