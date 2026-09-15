"""
fetch_web_docs.py -- Scrape and save key dataset/resource web pages as Markdown.
Run from project root: python resources/fetch_web_docs.py
"""

import sys
import io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

import os
from pathlib import Path
import trafilatura

# ── Directories ───────────────────────────────────────────────────────────────
out_dir = Path(__file__).parent / "web_docs"
out_dir.mkdir(parents=True, exist_ok=True)

# ── Pages to scrape ───────────────────────────────────────────────────────────
PAGES = {
    "idd_dataset_overview.md":       "https://idd.insaan.iiit.ac.in/",
    "argoverse_overview.md":         "https://www.argoverse.org/",
    "oxford_road_boundaries.md":     "https://oxford-robotics-institute.github.io/road-boundaries-dataset/",
    "autoware_wiki.md":              "https://github.com/CPFL/Autoware/wiki",
    "pythonrobotics_readme.md":      "https://github.com/AtsushiSakai/PythonRobotics",
    "argoverse_api_readme.md":       "https://github.com/argoai/argoverse-api",
}

print("=" * 65)
print("  SIH PS 26037 — Web Documentation Scraper (trafilatura)")
print("=" * 65)
print(f"\nSaving to: {out_dir.resolve()}\n")

ok_count   = 0
fail_count = 0

for filename, url in PAGES.items():
    out_path = out_dir / filename
    if out_path.exists() and out_path.stat().st_size > 200:
        print(f"[SKIP]  {filename} — already scraped")
        ok_count += 1
        continue

    print(f"[FETCH] {url}")
    try:
        downloaded = trafilatura.fetch_url(url)
        text = trafilatura.extract(
            downloaded,
            include_links=True,
            include_tables=True,
            favor_recall=True,
        ) if downloaded else None

        if text:
            content = f"# Source: {url}\n\n{text}"
            out_path.write_text(content, encoding="utf-8")
            print(f"[SAVED] {out_path.stat().st_size // 1024} KB → {filename}\n")
            ok_count += 1
        else:
            print(f"[EMPTY] Could not extract content from {url}\n")
            fail_count += 1
    except Exception as exc:
        print(f"[ERR]   {exc}\n")
        fail_count += 1

print("=" * 65)
print(f"  Scraped: {ok_count} / {len(PAGES)}  |  Failed: {fail_count}")
print(f"  Output:  {out_dir.resolve()}")
print("=" * 65)
