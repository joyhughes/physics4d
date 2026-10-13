#!/usr/bin/env python3
"""Build dist/index.html for publishing as a claude.ai artifact: the artifact
host supplies the doctype, html/head/body and the charset/viewport metas."""
import re, os
src = open('index.html').read()
for pat in [r'<!doctype html>\s*', r'<html[^>]*>\s*', r'</?head>\s*', r'<body>\s*', r'</body>\s*', r'</html>\s*',
            r'<meta charset="utf-8">\s*', r'<meta name="viewport"[^>]*>\s*']:
    src = re.sub(pat, '', src, flags=re.I)
os.makedirs('dist', exist_ok=True)
open('dist/index.html', 'w').write(src)
print('dist/index.html', len(src), 'bytes')
