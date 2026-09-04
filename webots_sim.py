"""
webots_sim.py — Launcher & Bridge Coordinator for Webots Simulation
=============================================================================
SIH PS-26037: Adaptive Path Planning for Unstructured Indian Roads

Usage:
  python webots_sim.py                   # Launch Webots with 3D Rural Road Scene
  python webots_sim.py --mode=realtime   # Run simulation immediately in real-time
  python webots_sim.py --bridge          # Enable TCP Bridge server on port 20000 for MATLAB
  python webots_sim.py --batch           # Non-blocking batch mode (no popups)
=============================================================================
"""

import os
import sys
import subprocess
import argparse
import time

def find_webots_executable():
    """Locates webots.exe on the system."""
    # 1. Check environment variable
    webots_home = os.environ.get("WEBOTS_HOME")
    if webots_home:
        candidates = [
            os.path.join(webots_home, "msys64", "mingw64", "bin", "webots.exe"),
            os.path.join(webots_home, "bin", "webots.exe"),
            os.path.join(webots_home, "webots.exe"),
        ]
        for c in candidates:
            if os.path.isfile(c):
                return c, webots_home

    # 2. Check standard Windows installation paths
    std_paths = [
        r"C:\Program Files\Webots",
        r"C:\Program Files (x86)\Webots",
        r"D:\Program Files\Webots",
    ]
    for sp in std_paths:
        c = os.path.join(sp, "msys64", "mingw64", "bin", "webots.exe")
        if os.path.isfile(c):
            return c, sp

    return None, None

def main():
    parser = argparse.ArgumentParser(description="Webots Indian Rural Road Simulator Launcher")
    parser.add_argument("--world", type=str, default=None, help="Path to .wbt world file")
    parser.add_argument("--mode", type=str, default="realtime", choices=["pause", "realtime", "fast"],
                        help="Startup simulation mode (default: realtime)")
    parser.add_argument("--batch", action="store_true", default=True, help="Prevent blocking dialog popups")
    parser.add_argument("--no-rendering", action="store_true", help="Disable 3D rendering for headless speed")
    parser.add_argument("--bridge", action="store_true", help="Signal supervisor to accept MATLAB TCP connection")
    parser.add_argument("--port", type=int, default=20000, help="MATLAB TCP bridge port")

    args = parser.parse_args()

    project_root = os.path.dirname(os.path.abspath(__file__))
    if not args.world:
        args.world = os.path.join(project_root, "webots", "worlds", "indian_rural_road.wbt")

    if not os.path.isfile(args.world):
        print(f"[ERROR] World file not found: {args.world}")
        sys.exit(1)

    webots_exe, webots_home = find_webots_executable()
    if not webots_exe:
        print("[ERROR] Webots executable not found!")
        print("Please set WEBOTS_HOME environment variable to your Webots installation directory.")
        sys.exit(1)

    print("=" * 65)
    print("  Starting Webots Simulation — Indian Rural Road Scenario")
    print("  SIH PS-26037: Adaptive Path Planning for Indian Roads")
    print("=" * 65)
    print(f"  Webots Binary : {webots_exe}")
    print(f"  World File    : {args.world}")
    print(f"  Startup Mode  : {args.mode}")
    print(f"  Bridge Mode   : {'ENABLED (port 20000)' if args.bridge else 'STANDALONE AUTONOMOUS'}")
    print("=" * 65 + "\n")

    # Clean up any lingering Webots processes to free port 1234
    if sys.platform == "win32":
        try:
            subprocess.run(["taskkill", "/F", "/IM", "webots.exe", "/IM", "webots-bin.exe", "/IM", "webotsw.exe"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.5)
        except Exception:
            pass

    # Configure environment
    env = os.environ.copy()
    env["WEBOTS_HOME"] = webots_home
    env["PYTHONPATH"] = os.path.join(webots_home, "lib", "controller", "python") + os.pathsep + env.get("PYTHONPATH", "")
    env["PATH"] = os.path.join(webots_home, "msys64", "mingw64", "bin") + os.pathsep + \
                  os.path.join(webots_home, "lib", "controller") + os.pathsep + env.get("PATH", "")
    if args.bridge:
        env["WEBOTS_BRIDGE_MODE"] = "1"
    else:
        env["WEBOTS_BRIDGE_MODE"] = "0"

    cmd = [webots_exe]
    if args.mode:
        cmd.append(f"--mode={args.mode}")
    if args.batch:
        cmd.append("--batch")
    if args.no_rendering:
        cmd.append("--no-rendering")
    cmd.extend(["--stdout", "--stderr"])
    cmd.append(args.world)

    print(f"[LAUNCH] Executing: {' '.join(cmd)}\n")
    try:
        proc = subprocess.Popen(cmd, env=env)
        proc.wait()
    except KeyboardInterrupt:
        print("\n[INFO] Terminating Webots...")
        proc.terminate()
        proc.wait()

if __name__ == "__main__":
    main()
