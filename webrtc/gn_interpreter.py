#!/usr/bin/env python

import gn_parse
import sys

class Scope(object):
    pass

def eval(statement, scope):
    if type(statement) is gn_parse.Conditional:
        result = eval(statement.condition, scope)
        return None
    else:
        print type(statement)
        return None

def evalFile(f):
    statements = gn_parse.parseFile(f)

    scope = Scope()
    for statement in statements:
        eval(statement, scope)



evalFile(sys.argv[1])
