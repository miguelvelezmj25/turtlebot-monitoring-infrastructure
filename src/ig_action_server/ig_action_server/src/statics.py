# statics - takes an IG AST and does some static checks

from constants import *
from parserIG import Node, Program, Vertex, Content, Action, Condition

# helper to find a vertex with the given index
def findn(vs, n):
  for v in vs:
    assert(isinstance(v, Vertex))
    (n2, c) = v.params
    if n == n2:
      return v
  raise Exception("Error: Couldn't find vertex with index %d" %n)

# returns a set of defined vertices
def defined(vlist):
  if vlist == []: return set()
  (v, vs) = vlist[0], vlist[1:]
  assert(isinstance(v, Vertex))
  (n, c) = v.params
  U = defined(vs)
  if n in U:
    raise Exception("Error: Found multiple vertices with index %d" %n)
  U.add(n)
  return U

# returns a set of connected vertices
def connected(vs, Uv, n):
  if n in Uv:
    U = defined(vs)
    if not Uv.issubset(U):
      raise Exception("Error: Visited an undefined vertex...?")
    return set()
  v = findn(vs, n)
  assert(isinstance(v, Vertex))
  (n, c) = v.params
  if c.operator == END:
    U = defined(vs)
    if not Uv.issubset(U):
      raise Exception("Error: Visited an undefined vertex...?")
    res = set()
    res.add(n)
    return res
  elif c.operator == DOONCE:
    (a, n2) = c.params
    Uv.add(n)
    U = connected(vs, Uv, n2)
    U.add(n)
    return U
  elif c.operator == DOUNTIL:
    (a, cnd, n2) = c.params
    Uv.add(n)
    U = connected(vs, Uv, n2)
    U.add(n)
    return U
  elif c.operator == IFELSE:
    (cnd, n2, n3) = c.params
    Uv2 = Uv.copy()
    Uv.add(n)
    U = connected(vs, Uv, n2)
    Uv2.add(n)
    Uv2.update(U)
    U2 = connected(vs, Uv2, n3)
    U2.add(n)
    U2.update(U)
    return U2
  elif c.operator == GOTO:
    (n2,) = c.params
    Uv.add(n)
    U = connected(vs, Uv, n2)
    U.add(n)
    return U
  else:
    raise Exception("Error: Unknown Vertex operator")

# checks validity. return true if valid, else raise Exception
def valid(ast):
  assert(isinstance(ast, Program))
  (v, vs) = ast.params
  assert(isinstance(v, Vertex))
  (s, c) = v.params
  U = defined([v]+vs)
  U2 = connected([v]+vs, set(), s)
  if U != U2:
    raise Exception("Error: Defined vertices are not equal to connected vertices.")
  return True
