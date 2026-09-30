@echo off
REM Judges' demo on the live runtime, 5 robots (world + one process per robot + passive bridge).
REM
REM   scripts\run_demo.cmd                 full guided demo, spec section 11 steps 1-7 (~8 min),
REM                                        each step verified from the metrics recorder
REM   scripts\run_demo.cmd --fast          same, shorter steps
REM   scripts\run_demo.cmd --steps 2,4     selected steps only
REM   scripts\run_demo.cmd demo            one scenario, runs until Ctrl+C (inject faults yourself:
REM                                        python -m amr.faults --help). Scenarios (5 robots each):
REM                                        demo, circular_wait, head_on, blockage, node_loss, dead_zone, partition
REM
REM Dashboard: http://localhost:8080 (opens automatically)
setlocal
call "%~dp0_py.cmd"
pushd "%~dp0.."
set "PYTHONPATH=%CD%"
if not exist "dashboard\dist\index.html" (
  echo [run_demo] building dashboard ...
  pushd dashboard
  if not exist node_modules call npm install
  call npm run build
  popd
)
set "FIRST=%~1"
if "%FIRST%"=="" goto walkthrough
if "%FIRST:~0,2%"=="--" goto walkthrough
start "" "http://localhost:8080"
"%PY%" -m amr.runtime.launcher --scenario %FIRST% --recorder --apply-faults
goto end
:walkthrough
"%PY%" -m amr.runtime.walkthrough %*
:end
popd
endlocal
