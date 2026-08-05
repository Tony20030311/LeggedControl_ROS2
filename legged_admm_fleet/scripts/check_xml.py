#!/usr/bin/env python3
"""Reject XML that only Gazebo's lenient parser would accept.

Written after four separate runs died in phase 1 on one mistake: a literal "--" inside an XML
comment, which is illegal and which expat rejects but TinyXML2 (what gz sim uses) sometimes
waves through. The failure surfaces 180 s later as "odom missing for robot1", nowhere near the
cause.

  python3 scripts/check_xml.py            # every .sdf/.xacro/.urdf under the package
  python3 scripts/check_xml.py FILE ...   # just these
"""
import re
import sys
import xml.parsers.expat
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def check(path):
    text = path.read_text(encoding='utf-8')
    bad = [c for c in re.findall(r'<!--(.*?)-->', text, re.S) if '--' in c]
    if bad:
        # Report where, since a comment can be forty lines long.
        line = text[:text.index(bad[0])].count('\n') + 1
        return f'{path}: "--" inside the XML comment starting near line {line}'
    p = xml.parsers.expat.ParserCreate()
    try:
        p.Parse(text, True)
    except xml.parsers.expat.ExpatError as e:
        return f'{path}: {e}'
    return None


def main(argv):
    if argv:
        paths = [Path(a) for a in argv]
    else:
        paths = sorted(p for ext in ('*.sdf', '*.xacro', '*.urdf')
                       for p in ROOT.rglob(ext))
    errors = [e for e in (check(p) for p in paths) if e]
    for e in errors:
        print(e, file=sys.stderr)
    print(f'{len(paths) - len(errors)}/{len(paths)} files parse')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
