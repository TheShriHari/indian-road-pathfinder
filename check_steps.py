import json

with open(r'C:\Users\toshr\.gemini\antigravity-ide\brain\0ae3e06b-0200-4342-a56a-3dfa9f9186e0\.system_generated\logs\transcript_full.jsonl', 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()

for line_idx in [138, 171, 245, 267, 268]:
    if line_idx < len(lines):
        obj = json.loads(lines[line_idx])
        content = str(obj.get('content', ''))
        print(f"=== Line {line_idx} (step {obj.get('step_index')}) ===")
        print(content[:600])
        print("\n")
