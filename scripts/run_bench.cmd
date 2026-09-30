@echo off
REM Benchmark sweep (fast runtime), 5 robots: ours vs baseline B0,
REM congestion medium/high x faults none/node_loss x 5 seeds, then the report.
REM Usage:  scripts\run_bench.cmd                  the 5-robot benchmark (about 2 minutes)
REM         scripts\run_bench.cmd --quick          2 seeds only
REM Extra args are passed to python -m amr.bench.sweep (see --help).
setlocal
call "%~dp0_py.cmd"
pushd "%~dp0.."
set "PYTHONPATH=%CD%"
"%PY%" -m amr.bench.sweep %*
if errorlevel 1 goto :end
"%PY%" -m amr.bench.report
:end
popd
endlocal
