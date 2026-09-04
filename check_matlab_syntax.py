import glob
import re

for filepath in glob.glob('matlab/*.m'):
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()
    for i in range(len(lines) - 1):
        line1 = lines[i].strip()
        line2 = lines[i + 1].strip()
        if line1.endswith('...') and line1[:-3].strip().endswith("'"):
            # Line 1 ends with ' ...
            if line2.startswith("'"):
                # Check if it was enclosed in [ ... ]
                prefix = lines[i][:lines[i].rfind('...')]
                if '[' not in prefix:
                    print(f"Potential syntax error in {filepath} line {i+1}-{i+2}:")
                    print(f"  {line1}")
                    print(f"  {line2}")

print("Syntax scan complete.")
