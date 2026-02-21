import sys

file_path = r'c:\Users\eemrull\RISA-bot\src\risabot_automode\risabot_automode\dashboard.py'

with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

new_lines = []
skip = False

for i, line in enumerate(lines):
    if line.startswith('DASHBOARD_HTML ='):
        skip = True
        new_lines.append('from risabot_automode.dashboard_templates import DASHBOARD_HTML, TEACH_HTML\n\n')
        continue
    
    if line.startswith('TEACH_HTML ='):
        skip = True
        continue
    
    if skip and line.strip() == '\"\"\"':
        skip = False
        continue
        
    if not skip and not line.startswith('# ======================== TEACH HTML Dashboard'):
        new_lines.append(line)

with open(file_path, 'w', encoding='utf-8') as f:
    f.writelines(new_lines)

print('Done')
