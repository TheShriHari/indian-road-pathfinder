---
name: video-review
description: Inspect and audit rendered MP4 video files by sampling keyframes, contact sheets, and identifying UI, telemetry, and timing bugs.
---

# Video Review Skill

Execute structured review of a rendered composition:
1. Extract keyframes: `python .agents/skills/video-review/scripts/extract_frames.py "<video_path>" ".tmp/review_frames" 1.0`
2. Inspect `.tmp/review_frames/montage.jpg` and targeted frames.
3. Check for text overlaps, speed/state mismatches, layout occlusions, and pacing drags.
