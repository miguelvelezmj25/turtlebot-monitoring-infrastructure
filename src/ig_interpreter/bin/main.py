#!/usr/bin/env python2
# main driver to interpret IGs

import ply.lex as lex
import lexerIG
import ply.yacc as yacc
import parserIG
import statics
import dynamics
import publisher

import sys

lexer = lex.lex(module=lexerIG)
parser = yacc.yacc(module=parserIG)

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print "Use: main.py <IG program>"
    exit(1)
  # Initializes node if not already initialized.
  publisher.initialize()
  # Publishes started message.
  publisher.publish("Starting")
  try:
    igfile = open(sys.argv[1], "r")
    igcode = igfile.read()
  except Exception as e:
    print e
    print "Could not open file for reading!"
    publisher.publish("Could not open file for reading!")
    exit(1)
  
  publisher.publish("Parsing and validating instructions")
  ast = parser.parse(igcode)
  assert(statics.valid(ast))
  publisher.publish("Running the instruction graph")
  dynamics.eval(ast)
  igfile.close()
