import sys, io, json
_old_stdout = sys.stdout
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
sys.stdout._keep = _old_stdout

def parse(text):
    tokens = []
    i = 0; n = len(text)
    while i < n:
        c = text[i]
        if c in ' \t\r\n':
            i += 1
        elif c == '(':
            tokens.append('('); i += 1
        elif c == ')':
            tokens.append(')'); i += 1
        elif c == '"':
            j = i + 1; buf = []
            while j < n:
                if text[j] == '\\':
                    buf.append(text[j+1]); j += 2
                elif text[j] == '"':
                    j += 1; break
                else:
                    buf.append(text[j]); j += 1
            tokens.append(('S', ''.join(buf))); i = j
        else:
            j = i
            while j < n and text[j] not in ' \t\r\n()"':
                j += 1
            tokens.append(('S', text[i:j])); i = j
    stack = [[]]
    for t in tokens:
        if t == '(':
            stack.append([])
        elif t == ')':
            done = stack.pop(); stack[-1].append(done)
        else:
            stack[-1].append(t[1])
    return stack[0]

def head(n):
    return n[0] if isinstance(n, list) and n and isinstance(n[0], str) else None

def find_all(node, name, out=None):
    if out is None: out = []
    if isinstance(node, list):
        if head(node) == name: out.append(node)
        for c in node:
            if isinstance(c, list): find_all(c, name, out)
    return out

def kids(node, name):
    return [c for c in node if isinstance(c, list) and head(c) == name]

def kid(node, name):
    for c in node:
        if isinstance(c, list) and head(c) == name: return c
    return None

def atoms(node):
    return [c for c in node if isinstance(c, str)]
