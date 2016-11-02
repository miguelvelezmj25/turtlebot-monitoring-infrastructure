# interpreter - executes an IG

from constants import *
from statics import findn
import turtlebot_actions as turtlebot

def doaction(action):
  # we currently only support moving and saying in this simulation
  if action.operator == MOVE:
    (distance, angular) = action.params
    print "Moving for distance %s at rotation %s" %(distance, angular)
    turtlebot.move(distance, angular)
  elif action.operator == SAY:
    (s,) = action.params
    turtlebot.say(s)
  else:
    raise Exception("Runtime Error: Unsupported action!")

def checkcond(cond):
  # we currently only support checking for visible objects and if an object is
  # nearby
  if cond.operator == VISIBLE:
    print "Checking if %s is visible..." %cond.params[0]
    print "Is %s visible?" %cond.params[0]
    ans = raw_input()
    return ans in ("yes", "y", "", "\n")
  elif cond.operator == STOP:
    print "Checking if %s is within %s distance..." %(cond.params[1],
                                                      cond.params[0])
    print "Is %s within %s distance?" %(cond.params[1], cond.params[0])
    ans = raw_input()
    return ans in ("yes", "y", "", "\n")

def trystep(config):
  (n, vs, I, O) = config
  v = findn(vs, n)
  (n, c) = v.params
  if c.operator == END:
    return (TERMINATED, None)
  elif I == [] and (c.operator in (DOUNTIL, IFELSE)):
    return (WAITING, None)
  elif c.operator == DOONCE:
    (a, n2) = c.params
    doaction(a)
    return (STEP, (n2, vs, I, [a] + O))
  elif c.operator == DOUNTIL:
    (a, cnd, n2) = c.params
    b = checkcond(cnd)
    doaction(a)
    if b:
      return (STEP, (n2, vs, I, [a] + O))
    else:
      return (STEP, (n, vs, I, [a] + O))
  elif c.operator == IFELSE:
    (cnd, n2, n3) = c.params
    b = checkcond(cnd)
    if b:
      return (STEP, (n2, vs, I, O))
    else:
      return (STEP, (n3, vs, I, O))
  elif c.operator == GOTO:
    (n2,) = c.params
    return (STEP, (n2, vs, I, O))
  else:
    raise Exception("Runtime Error: Unknown Content Operator?")

def eval(ast, node):
  (v, vs) = ast.params
  (n, c) = v.params
  config = (n, [v]+vs, [True], [])
  while True:
    (status, config2) = trystep(config)
    if status == WAITING:
      print "Robot is waiting for input! But this shouldn't happen in this simulation! What's going on?"
      break
    elif status == TERMINATED:
      print "Finished!"
      break
    else:
      config = config2
  (_, _, _, O) = config
  print "End output: ",
  print O
  return
