#!/usr/bin/env python3
"""
start_carla.py  —  CARLA Simulator Low-Effort / Low-Graphics Launcher
=====================================================================
SIH PS-26037: Adaptive Path Planning for Indian Roads

Launches CarlaUE4 with optimized flags to:
  1. Prevent Unreal Engine D3D Device Lost (0x887A0020) crashes.
  2. Force DirectX 11 (bypasses unstable DX12 pipelines on Windows).
  3. Set quality level to Low (dramatically reduces shader passes & VRAM usage).
  4. Cap framerate to 20 FPS (prevents 100% GPU spikes and TDR timeouts).
  5. Optionally run headless with -RenderOffScreen for automated testing.

Usage:
  python start_carla.py
  python start_carla.py --headless
  python start_carla.py --fps 20 --quality Low --res 800x600
  python start_carla.py --carla-dir "C:\\CARLA_0.9.15"
=====================================================================
"""

import os
import sys
import glob
import argparse
import subprocess


def find_carla_executable(custom_dir=None, custom_bin=None):
    """Searches common installation locations for CarlaUE4 executable."""
    if custom_bin and os.path.isfile(custom_bin):
        return os.path.abspath(custom_bin)

    is_windows = (os.name == 'nt')
    exe_name = 'CarlaUE4.exe' if is_windows else 'CarlaUE4.sh'

    search_dirs = []
    if custom_dir:
        search_dirs.append(custom_dir)

    env_root = os.environ.get('CARLA_ROOT')
    if env_root:
        search_dirs.append(env_root)

    # Relative directory searches
    script_dir = os.path.dirname(os.path.abspath(__file__))
    search_dirs.extend([
        script_dir,
        os.path.join(script_dir, '..', 'carla'),
        os.path.join(script_dir, '..', 'CARLA'),
        os.path.join(script_dir, '..', '..', 'carla'),
        os.path.join(script_dir, '..', '..', 'CARLA'),
    ])

    # Windows root drives
    if is_windows:
        for drive in ['C:', 'D:', 'E:']:
            search_dirs.extend(glob.glob(f'{drive}\\CARLA*'))
            search_dirs.extend(glob.glob(f'{drive}\\carla*'))
            search_dirs.extend([
                f'{drive}\\Program Files\\CARLA',
                f'{drive}\\CARLA_0.9.15',
                f'{drive}\\CARLA_0.9.14',
                f'{drive}\\CARLA_0.9.13',
            ])
    else:
        search_dirs.extend([
            os.path.expanduser('~/CARLA*'),
            os.path.expanduser('~/carla*'),
            '/opt/carla',
        ])

    for d in search_dirs:
        if not d or not os.path.isdir(d):
            continue
        candidate = os.path.join(d, exe_name)
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
        # Search immediate subdirectories
        for sub in glob.glob(os.path.join(d, '**', exe_name), recursive=True):
            if os.path.isfile(sub):
                return os.path.abspath(sub)

    return None


def main():
    parser = argparse.ArgumentParser(
        description='Launch CARLA in Low-Graphics / Low-Effort Mode to prevent D3D GPU crashes.')
    parser.add_argument('--carla-bin', default=None,
                        help='Direct path to CarlaUE4.exe or CarlaUE4.sh')
    parser.add_argument('--carla-dir', default=None,
                        help='CARLA root directory containing CarlaUE4')
    parser.add_argument('--quality', choices=['Low', 'Epic'], default='Low',
                        help='Graphics quality level (default: Low)')
    parser.add_argument('--fps', type=int, default=20,
                        help='Framerate limit to prevent GPU overload (default: 20)')
    parser.add_argument('--res', default='800x600',
                        help='Window resolution WxH (default: 800x600)')
    parser.add_argument('--port', type=int, default=2000,
                        help='CARLA RPC port (default: 2000)')
    parser.add_argument('--dx11', action='store_true', default=True,
                        help='Force DirectX 11 backend on Windows (default: True)')
    parser.add_argument('--vulkan', action='store_true', default=False,
                        help='Force Vulkan backend')
    parser.add_argument('--headless', '--offscreen', dest='headless', action='store_true',
                        help='Run headless with -RenderOffScreen (no display window, lowest GPU usage)')

    args = parser.parse_args()

    print('=' * 72)
    print('  CARLA Low-Graphics / Low-Effort Launcher (SIH PS-26037)')
    print('=' * 72)

    carla_exe = find_carla_executable(args.carla_dir, args.carla_bin)
    if not carla_exe:
        print('\n[ERROR] CarlaUE4 executable not found automatically!')
        print('Please specify its location using one of the following methods:')
        print('  1. Set the CARLA_ROOT environment variable: set CARLA_ROOT=C:\\path\\to\\CARLA')
        print('  2. Pass --carla-bin: python start_carla.py --carla-bin "C:\\CARLA\\CarlaUE4.exe"')
        print('  3. Pass --carla-dir: python start_carla.py --carla-dir "C:\\CARLA"')
        print('\nAlternatively, run CarlaUE4.exe directly with these recommended flags:')
        print('  CarlaUE4.exe -dx11 -quality-level=Low -benchmark -fps=20 -windowed -ResX=800 -ResY=600\n')
        sys.exit(1)

    print(f'[FOUND] CARLA executable: {carla_exe}')

    # Parse resolution
    try:
        resx, resy = args.res.lower().split('x')
        resx, resy = int(resx), int(resy)
    except Exception:
        resx, resy = 800, 600

    cmd = [carla_exe]

    # Graphics API selection
    if os.name == 'nt':
        if args.vulkan:
            cmd.append('-vulkan')
        elif args.dx11:
            cmd.append('-dx11')

    # Low graphics & stability flags
    cmd.append(f'-quality-level={args.quality}')
    cmd.append('-benchmark')
    cmd.append(f'-fps={args.fps}')
    cmd.append(f'-carla-rpc-port={args.port}')

    if args.headless:
        cmd.append('-RenderOffScreen')
        print('[MODE] Running HEADLESS (-RenderOffScreen): No viewport window created.')
    else:
        cmd.append('-windowed')
        cmd.append(f'-ResX={resx}')
        cmd.append(f'-ResY={resy}')
        print(f'[MODE] Windowed mode: {resx}x{resy} @ {args.fps} FPS, Quality={args.quality}')

    print(f'\n[LAUNCHING] {" ".join(cmd)}\n')

    try:
        proc = subprocess.Popen(cmd, cwd=os.path.dirname(carla_exe))
        print('[RUNNING] CARLA is now running in low-effort mode.')
        print('          Press Ctrl+C in this terminal to stop CARLA.\n')
        proc.wait()
    except KeyboardInterrupt:
        print('\n[STOPPING] Terminating CARLA process...')
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        print('[STOPPED] CARLA shutdown cleanly.')
    except Exception as e:
        print(f'[ERROR] Failed to start CARLA: {e}')
        sys.exit(1)


if __name__ == '__main__':
    main()
