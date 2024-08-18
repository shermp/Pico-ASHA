#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

def ms_to_ts(ms: str) -> str:
    millis = int(ms)
    seconds, milliseconds = divmod(millis, 1000)
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    return (f'{hours:02}:{minutes:02}:{seconds:02}.{milliseconds:03}')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('hci_dump', type=Path)
    parser.add_argument('output', type=Path)

    args = parser.parse_args()

    with open(args.hci_dump, 'r') as hci:
        lines = hci.readlines()
    
    out_lines: list[str] = []
    for line in lines:
        m = re.match(r'\[ASHA\s+(ERROR|INFO|AUDIO|SCAN)\s+:\s+(\d+) ms\] (.+)', line)
        if m:
            out_lines.append(f'[{ms_to_ts(m.group(2))}] LOG -- ASHA {m.group(1)}: {m.group(3)}\n')
        else:
            out_lines.append(line)
    
    with open(args.output, 'w') as out:
        out.writelines(out_lines)
    