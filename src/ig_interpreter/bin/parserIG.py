# parser

from lexerIG import tokens
from constants import *
import publisher

# abstract node class for operators in the AST
class Node(object):
  def __init__(self, operator, params):
    self.operator = operator
    self.params = params
  # TODO: this is broken, please fix
  def pprint(self, tabinit):
    print tabinit + self.operator
    for param in self.params:
      if type(param) == float:
        print tabinit + "  " + str(param)
      elif type(param) == str:
        print tabinit + "  " + param
      else:
        param.pprint(tabinit + "  ")
  def prettyprint(self):
    self.pprint("")

# sort-specific nodes. We don't need one for Vertices because that's a list.
class Program(Node):
  def __init__(self, operator, params):
    super(Program, self).__init__(operator, params)
    assert(operator in [P])

class Vertex(Node):
  def __init__(self, operator, params):
    super(Vertex, self).__init__(operator, params)
    assert(operator in [V])

class Content(Node):
  def __init__(self, operator, params):
    super(Content, self).__init__(operator, params)
    assert(operator in [DOONCE, DOUNTIL, IFELSE, GOTO, END])

class Action(Node):
  def __init__(self, operator, params):
    super(Action, self).__init__(operator, params)
    assert(operator in [MOVE, SAY, MOVETO, LOCATE])

class Condition(Node):
  def __init__(self, operator, params):
    super(Condition, self).__init__(operator, params)
    assert(operator in [VISIBLE, STOP])

# parsing rules
def p_program(t):
  """program : P LPAR vertex COMMA vertices RPAR"""
  t[0] = Program(P, (t[3], t[5]))
  
def p_vertices(t):
  """vertices : NIL
              | vertex CONS vertices"""
  if t[1] == "nil":
    t[0] = []
  else:
    t[0] = [t[1]] + t[3] # "Cons'ing" a vertex onto my list

def p_vertex(t):
  """vertex : V LPAR NUM COMMA content RPAR"""
  if not t[3].is_integer():
    raise Exception("Error: Vertex index %s is not an integer" %t[3])
  t[0] = Vertex(V, (t[3], t[5]))

def p_content(t):
  """content : DO action THEN NUM
             | DO action UNTIL cnd THEN NUM
             | IF cnd THEN NUM ELSE NUM
             | GOTO NUM
             | END"""
  if t[1] == "end":
    t[0] = Content(END, ())
  elif t[1] == "goto":
    if not t[2].is_integer():
      raise Exception("Error: Vertex index %s is not an integer" %t[2])
    t[0] = Content(GOTO, (t[2],))
  elif t[1] == "do" and t[3] == "then":
    if not t[4].is_integer():
      raise Exception("Error: Vertex index %s is not an integer" %t[4])
    t[0] = Content(DOONCE, (t[2], t[4]))
  elif t[3] == "until":
    if not t[6].is_integer():
      raise Exception("Error: Vertex index %s is not an integer" %t[6])
    t[0] = Content(DOUNTIL, (t[2], t[4], t[6]))
  elif t[1] == "if":
    if not t[4].is_integer():
      raise Exception("Error: Vertex index %s is not an integer" %t[4])
    if not t[6].is_integer():
      raise Exception("Error: Vertex index %s is not an integer" %t[6])
    t[0] = Content(IFELSE, (t[2], t[4], t[6]))

def p_action(t):
  """action : MOVE LPAR NUM    COMMA NUM     COMMA NUM    COMMA NUM    COMMA NUM RPAR
            | SAY  LPAR STRING RPAR
            | MOVETO LPAR NUM COMMA NUM RPAR
            | LOCATE LPAR NUM COMMA NUM RPAR"""
  if t[1] == "Move":
    t[0] = Action(MOVE, (t[3], t[5], t[7], t[9], t[11]))
  elif t[1] == "MoveTo":
    t[0] = Action(MOVETO, (t[3], t[5]))
  elif t[1] == "Locate":
    t[0] = Action(LOCATE, (t[3], t[5]))
  else:
    t[0] = Action(SAY, (t[3],))

def p_cnd(t):
  """cnd : VISIBLE LPAR STRING RPAR
         | STOP    LPAR NUM    COMMA  STRING RPAR"""
  if t[1] == "Visible":
    t[0] = Condition(VISIBLE, (t[3],))
  else:
    t[0] = Condition(STOP, (t[3], t[5]))

# error
def p_error(t):
  print t
  print "Found syntax error in input!"
  publisher.publish("Found syntax error in input!")
  raise Exception("Parser Error")
