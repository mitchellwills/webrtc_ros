#!/usr/bin/env python

import gn_parse
import sys


def func_import(args, body):
    file_name = args[0]
    return file_name

def func_config(args, body):
    return False



class Scope(object):
    pass



def eval(statement, scope):
    if type(statement) is gn_parse.Condition:
        result = eval(statement.condition, scope)
        return None
    if type(statement) is gn_parse.Call:
        print "Calling: ", statement.name, statement.args
        func = globals().get("func_"+statement.name, None)
        if func is None:
            raise Exception("Could not find function: " + statement.name)
        return func(statement.args, statement.body)
    else:
        print type(statement)
        return None

def evalFile(f):
    parser = gn_parse.build_parser()
    statements = parser.parse(f.read())

    scope = Scope()
    for statement in statements:
        print eval(statement, scope)



evalFile(file(sys.argv[1]))
