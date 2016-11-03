
# parsetab.py
# This file is automatically generated. Do not edit.
_tabversion = '3.8'

_lr_method = 'LALR'

_lr_signature = '0850E90D7E566BCDDE0BB480E9D946FA'
    
_lr_action_items = {'LPAR':([1,5,22,23,26,27,],[3,7,29,30,33,34,]),'CONS':([9,25,],[12,-4,]),'END':([14,],[16,]),'GOTO':([14,],[17,]),'NIL':([6,12,],[8,8,]),'DO':([14,],[18,]),'MOVE':([18,],[22,]),'STOP':([20,32,],[26,26,]),'ELSE':([42,],[48,]),'RPAR':([8,10,15,16,19,21,37,38,41,49,50,51,52,],[-2,13,-3,-9,25,-8,44,-5,47,53,-6,54,-7,]),'P':([0,],[1,]),'SAY':([18,],[23,]),'NUM':([7,17,29,31,33,35,43,45,48,],[11,21,36,38,40,42,49,50,52,]),'COMMA':([4,11,25,36,40,],[6,14,-4,43,46,]),'VISIBLE':([20,32,],[27,27,]),'V':([3,6,12,],[5,5,5,]),'STRING':([30,34,46,],[37,41,51,]),'IF':([14,],[20,]),'THEN':([24,28,39,44,47,53,54,],[31,35,45,-11,-12,-10,-13,]),'UNTIL':([24,44,53,],[32,-11,-10,]),'$end':([2,13,],[0,-1,]),}

_lr_action = {}
for _k, _v in _lr_action_items.items():
   for _x,_y in zip(_v[0],_v[1]):
      if not _x in _lr_action:  _lr_action[_x] = {}
      _lr_action[_x][_k] = _y
del _lr_action_items

_lr_goto_items = {'vertex':([3,6,12,],[4,9,9,]),'vertices':([6,12,],[10,15,]),'content':([14,],[19,]),'cnd':([20,32,],[28,39,]),'program':([0,],[2,]),'action':([18,],[24,]),}

_lr_goto = {}
for _k, _v in _lr_goto_items.items():
   for _x, _y in zip(_v[0], _v[1]):
       if not _x in _lr_goto: _lr_goto[_x] = {}
       _lr_goto[_x][_k] = _y
del _lr_goto_items
_lr_productions = [
  ("S' -> program","S'",1,None,None,None),
  ('program -> P LPAR vertex COMMA vertices RPAR','program',6,'p_program','parserIG.py',52),
  ('vertices -> NIL','vertices',1,'p_vertices','parserIG.py',56),
  ('vertices -> vertex CONS vertices','vertices',3,'p_vertices','parserIG.py',57),
  ('vertex -> V LPAR NUM COMMA content RPAR','vertex',6,'p_vertex','parserIG.py',64),
  ('content -> DO action THEN NUM','content',4,'p_content','parserIG.py',70),
  ('content -> DO action UNTIL cnd THEN NUM','content',6,'p_content','parserIG.py',71),
  ('content -> IF cnd THEN NUM ELSE NUM','content',6,'p_content','parserIG.py',72),
  ('content -> GOTO NUM','content',2,'p_content','parserIG.py',73),
  ('content -> END','content',1,'p_content','parserIG.py',74),
  ('action -> MOVE LPAR NUM COMMA NUM RPAR','action',6,'p_action','parserIG.py',97),
  ('action -> SAY LPAR STRING RPAR','action',4,'p_action','parserIG.py',98),
  ('cnd -> VISIBLE LPAR STRING RPAR','cnd',4,'p_cnd','parserIG.py',105),
  ('cnd -> STOP LPAR NUM COMMA STRING RPAR','cnd',6,'p_cnd','parserIG.py',106),
]