import ply.lex as lex
import lexerIG
import ply.yacc as yacc
import parserIG
import statics
import dynamics

import sys

def parse_and_execute (file):
  igfile = open(file)
  igcode = igfile.read();
  igfile.close ();
  
  lexer = lex.lex(module=lexerIG)
  parser = yacc.yacc(module=parserIG)
  ast = parser.parse(igcode)
  assert(statics.valid(ast))
  dynamics.eval(ast)
  
