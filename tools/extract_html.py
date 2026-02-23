#!/usr/bin/env python3
"""Extract DASHBOARD_HTML and TEACH_HTML from dashboard.py into dashboard_templates.py."""
import os

base = os.path.join(os.path.dirname(__file__), '..', 'src', 'risabot_automode', 'risabot_automode')
src = os.path.join(base, 'dashboard.py')
tpl = os.path.join(base, 'dashboard_templates.py')

with open(src, 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Locate the triple-quoted string boundaries
dash_start = dash_end = teach_start = teach_end = None

for i, line in enumerate(lines):
    stripped = line.strip()
    if stripped.startswith('DASHBOARD_HTML') and '"""' in line and dash_start is None:
        dash_start = i
    elif stripped.startswith('TEACH_HTML') and '"""' in line and teach_start is None:
        teach_start = i

# Find closing """ for DASHBOARD_HTML (search backwards from TEACH_HTML)
for i in range(teach_start - 1, dash_start, -1):
    if lines[i].rstrip('\r\n').endswith('"""'):
        dash_end = i
        break

# Find closing """ for TEACH_HTML (search forward)
for i in range(teach_start + 1, len(lines)):
    if lines[i].rstrip('\r\n').endswith('"""'):
        teach_end = i
        break

print(f'DASHBOARD_HTML: lines {dash_start+1}-{dash_end+1}')
print(f'TEACH_HTML:     lines {teach_start+1}-{teach_end+1}')
print(f'Total HTML:     {(dash_end - dash_start + 1) + (teach_end - teach_start + 1)} lines')

# Extract the two blocks
dash_html_block = lines[dash_start:dash_end+1]
teach_html_block = lines[teach_start:teach_end+1]

# Write dashboard_templates.py
with open(tpl, 'w', encoding='utf-8') as f:
    f.write('# dashboard_templates.py\n')
    f.write('# Auto-extracted HTML templates for the RISA-bot dashboard.\n')
    f.write('# Edit these to change the dashboard UI appearance.\n\n')
    f.write('# ======================== Main HTML Dashboard ========================\n')
    f.writelines(dash_html_block)
    f.write('\n\n')
    f.write('# ======================== TEACH HTML Dashboard ========================\n')
    f.writelines(teach_html_block)
    f.write('\n')

# Rewrite dashboard.py with import instead of inline HTML
new_lines = lines[:dash_start]
new_lines.append('from .dashboard_templates import DASHBOARD_HTML, TEACH_HTML\n')
new_lines.append('\n')
# Skip all HTML content and blank lines between teach_end and next real code
skip_to = teach_end + 1
while skip_to < len(lines) and lines[skip_to].strip() == '':
    skip_to += 1
new_lines.extend(lines[skip_to:])

with open(src, 'w', encoding='utf-8') as f:
    f.writelines(new_lines)

print(f'\ndashboard.py:           {len(new_lines)} lines (was {len(lines)})')
print(f'dashboard_templates.py: {len(dash_html_block) + len(teach_html_block) + 7} lines')
print('Done!')
