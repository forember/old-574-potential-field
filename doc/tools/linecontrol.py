#!/usr/bin/env python3

import re

code_title_re = re.compile(r'^\s*---\s*(.+?)\s*(\+=)?\s*$')
reference_re = re.compile(r'^\s*@\{.+?\}$')

def main():
    import sys
    if len(sys.argv) != 2:
        print("Usage: linecontrol.py <lit file>")
        raise SystemExit(1)
    filename = sys.argv[1]
    with open(filename) as f:
        for lineno, line in enumerate(f, 1):
            print(line, end='')
            if code_title_re.match(line) or reference_re.match(line):
                print('#line {} "{}"'.format(lineno + 1, filename))

if __name__ == '__main__':
    main()
