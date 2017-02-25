# lexer

# the tokens in our IG language
tokens = [
  "LPAR",
  "RPAR",
  "COMMA",
  "NUM",
  "STRING",
  "P",
  "NIL",
  "CONS",
  "V",
  "DO",
  "THEN",
  "UNTIL",
  "IF",
  "ELSE",
  "GOTO",
  "END",
  "MOVE",
  "SAY",
  "LOCATE",
  "MOVETO",
  "MOVEABS",
  "MOVEREL",
  "TURNABS",
  "TURNREL",
  "VISIBLE",
  "STOP",
  "FORWARD"
]

# simple tokens
t_LPAR =         r"\("
t_RPAR =         r"\)"
t_COMMA =        r","
t_P =            r"P"
t_NIL =          r"nil"
t_CONS =         r"::"
t_V =            r"V"
t_DO =           r"do"
t_THEN =         r"then"
t_UNTIL =        r"until"
t_IF =           r"if"
t_ELSE =         r"else"
t_GOTO =         r"goto"
t_END =          r"end"
t_MOVE =         r"Move"
t_SAY =          r"Say"
t_VISIBLE =      r"Visible"
t_STOP =         r"Stop"
t_MOVETO =       r"MoveTo"
t_LOCATE =       r"Locate"
t_MOVEABS =      r"MoveAbs"
t_MOVEREL =      r"MoveRel"
t_TURNABS =      r"TurnAbs"
t_TURNREL =      r"TurnRel"
t_FORWARD =      r"Forward"
# more complex tokens

def t_NUM(t):
  r"(-)?\d+(\.\d+)?"
  t.value = float(t.value)
  return t

def t_STRING(t):
  r'".*"'
  t.value = eval(t.value)
  return t

# ignore whitespace
t_ignore = " \t\n"

# error
def t_error(t):
  print "Unknown character '%s'" %t.value[0]
  print "Token: %s" %t
  raise Exception("Lexer Error")
