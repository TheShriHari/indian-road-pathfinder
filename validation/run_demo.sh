#!/usr/bin/env bash
# Indian Road Pathfinder - SIH / MathWorks Demo Reproduction Script
# Executes Seed 438 forensic comparison & 1000-trial verification

echo '=== Reproducing Pathfinder Telemetry & Benchmarks ==='
matlab -batch "cd('../matlab'); audit_seed_438_forensics();"
echo '=== Finished Seed 438 Audit ==='
