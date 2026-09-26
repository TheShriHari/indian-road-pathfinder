@echo off
REM Indian Road Pathfinder - SIH / MathWorks Demo Reproduction Script
echo === Reproducing Pathfinder Telemetry & Benchmarks ===
matlab -batch "cd('../matlab'); audit_seed_438_forensics();"
echo === Finished Seed 438 Audit ===
