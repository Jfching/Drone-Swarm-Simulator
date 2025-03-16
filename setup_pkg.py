#!/usr/bin/env python3
import os
import glob
import re

def fix_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Replace hyphenated name with underscore name
    fixed_content = content.replace('multi-drone-system', 'multi_drone_system')
    
    # Only write if changes were made
    if fixed_content != content:
        print(f"Fixing references in {filepath}")
        with open(filepath, 'w') as f:
            f.write(fixed_content)

# Directory to search
search_dir = 'multi_drone_system'

# Find all python files
for ext in ['*.py', '*.xml', '*.launch.py', '*.txt']:
    for filepath in glob.glob(f"{search_dir}/**/{ext}", recursive=True):
        fix_file(filepath)

print("Done fixing files")
EOF