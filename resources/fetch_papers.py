"""
fetch_papers.py -- Download all SIH PS 26037 research PDFs and convert to Markdown.
Run from the project root: python resources/fetch_papers.py
"""

import sys
import io
import os

# Force UTF-8 output on Windows (avoids cp1252 UnicodeEncodeError)
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

from pathlib import Path
import requests
import pymupdf4llm

# ── Directories ───────────────────────────────────────────────────────────────
base_dir = Path(__file__).parent
pdf_dir  = base_dir / "papers"
md_dir   = base_dir / "papers_md"

pdf_dir.mkdir(parents=True, exist_ok=True)
md_dir.mkdir(parents=True, exist_ok=True)

# ── Download Manifest ─────────────────────────────────────────────────────────
# All papers that have freely accessible PDFs (arXiv / CMU / open institutional)
PAPERS = {
    # ── Core Control / Planning Papers ────────────────────────────────────────
    "Snider_2009_PurePursuit_CMU-RI-TR-09-08.pdf": (
        "https://www.ri.cmu.edu/pub_files/2009/2/"
        "Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf"
    ),
    "Paden_2016_MotionPlanningSurvey_arXiv1604.07446.pdf": (
        "https://arxiv.org/pdf/1604.07446"
    ),
    "Likhachev_2008_DynamicFeasibleManeuvers.pdf": (
        "https://www.roboticsproceedings.org/rss04/p57.pdf"
    ),
    "Zhu_2015_ConvexElasticSmoothing_arXiv1506.01085.pdf": (
        "https://arxiv.org/pdf/1506.01085"
    ),

    # ── Dataset Papers ────────────────────────────────────────────────────────
    "Varma_2018_IDD_IndiaDrivingDataset_arXiv1811.10200.pdf": (
        "https://arxiv.org/pdf/1811.10200"
    ),

    # ── Unstructured Road Papers ──────────────────────────────────────────────
    "Ososinski_2015_IllDefinedRoads_preprint.pdf": (
        "https://arxiv.org/pdf/1412.2953"   # arXiv preprint of J.Field Robotics paper
    ),
}

HEADERS = {
    "User-Agent": (
        "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
        "AppleWebKit/537.36 (KHTML, like Gecko) "
        "Chrome/120.0.0.0 Safari/537.36"
    )
}

# ── Step 1: Download PDFs ─────────────────────────────────────────────────────
print("=" * 65)
print("  SIH PS 26037 — Research Paper Downloader")
print("=" * 65)
print(f"\nSaving PDFs to: {pdf_dir.resolve()}\n")

download_ok  = []
download_fail = []

for filename, url in PAPERS.items():
    dest = pdf_dir / filename
    if dest.exists() and dest.stat().st_size > 10_000:
        print(f"[SKIP]  {filename} — already downloaded ({dest.stat().st_size // 1024} KB)")
        download_ok.append(filename)
        continue

    print(f"[GET]   {filename}")
    print(f"        {url}")
    try:
        resp = requests.get(url, headers=HEADERS, timeout=30, allow_redirects=True)
        if resp.status_code == 200 and len(resp.content) > 5_000:
            dest.write_bytes(resp.content)
            print(f"[SAVED] {dest.stat().st_size // 1024} KB → {dest.name}\n")
            download_ok.append(filename)
        else:
            print(f"[FAIL]  HTTP {resp.status_code} / {len(resp.content)} bytes — skipping\n")
            download_fail.append((filename, f"HTTP {resp.status_code}"))
    except Exception as exc:
        print(f"[ERR]   {exc}\n")
        download_fail.append((filename, str(exc)))

# ── Step 2: Convert PDFs to Markdown ─────────────────────────────────────────
print("\n" + "=" * 65)
print("  Converting PDFs -> Markdown (PyMuPDF4LLM)")
print("=" * 65)
print(f"\nSaving Markdown to: {md_dir.resolve()}\n")

all_pdfs = sorted(pdf_dir.glob("*.pdf"))
convert_ok   = []
convert_fail = []

for pdf_path in all_pdfs:
    out_md = md_dir / f"{pdf_path.stem}.md"
    if out_md.exists() and out_md.stat().st_size > 500:
        print(f"[SKIP]  {out_md.name} — already converted")
        convert_ok.append(out_md.name)
        continue

    print(f"[PARSE] {pdf_path.name} …")
    try:
        md_text = pymupdf4llm.to_markdown(str(pdf_path))
        out_md.write_text(md_text, encoding="utf-8")
        print(f"[SAVED] {out_md.stat().st_size // 1024} KB → {out_md.name}\n")
        convert_ok.append(out_md.name)
    except Exception as exc:
        print(f"[ERR]   {pdf_path.name}: {exc}\n")
        convert_fail.append((pdf_path.name, str(exc)))

# ── Summary ───────────────────────────────────────────────────────────────────
print("\n" + "=" * 65)
print("  SUMMARY")
print("=" * 65)
print(f"  PDFs downloaded : {len(download_ok)} / {len(PAPERS)}")
if download_fail:
    for f, reason in download_fail:
        print(f"    ✗ {f}: {reason}")
print(f"  Markdowns saved : {len(convert_ok)} / {len(all_pdfs)}")
if convert_fail:
    for f, reason in convert_fail:
        print(f"    ✗ {f}: {reason}")
print(f"\n  Papers dir : {pdf_dir.resolve()}")
print(f"  Markdown dir: {md_dir.resolve()}")
print("=" * 65)
