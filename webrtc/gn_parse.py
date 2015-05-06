#!/usr/bin/env python

import sys
from pyparsing import Word, alphas, restOfLine, OneOrMore, Or, QuotedString, ZeroOrMore, Optional, Group, Literal, delimitedList, Empty, ParseResults, Forward, operatorPrecedence, opAssoc,oneOf,NotAny, nums

comment = Literal("#") + restOfLine

variable = Word(alphas+"_", alphas+"_"+nums)
integer = Word(nums).setParseAction( lambda s,l,t: [ int(t[0]) ] )
string = QuotedString(quoteChar="\"", escChar="\\")

value = Forward()
expression = Forward()
operand = Or([variable, value])
list_value = operand # don't include operations cause it slows things down significantly

gn_list = Literal("[").suppress() \
         + Group(Or([delimitedList(list_value, delim=","), ZeroOrMore(list_value + Literal(",").suppress())]))("list") \
          + Literal("]").suppress()
value << Or([integer, string, gn_list])

operation = operatorPrecedence(operand,
                               [
                                   (Literal('!'), 1, opAssoc.RIGHT),
                                   (oneOf('* /'), 2, opAssoc.LEFT),
                                   (oneOf('+ -'), 2, opAssoc.LEFT),
                                   (oneOf(['==','!=','>','<','<=','>=']), 2, opAssoc.LEFT),
                                   (oneOf(['||','&&']), 2, opAssoc.LEFT),
                                   (oneOf(['-=','+=','=']), 2, opAssoc.LEFT)
                               ]
).setResultsName("operation")
expression << Or([operation, operand])

statementOp = operatorPrecedence(operand,
                                 [
                                     (oneOf(['-=','+=','=']), 2, opAssoc.LEFT)
                                 ]
                             ).setResultsName("operation")

function = Forward()
conditional = Forward()
statement = Or([statementOp, conditional, function])
statements = ZeroOrMore(statement)

conditional << Group(Literal("if").suppress()+Literal("(").suppress()+expression("condition")+Literal(")").suppress()
                     + Literal("{").suppress() + statements("body") + Literal("}").suppress()
                     + Optional(Or([
                         Literal("else").suppress() + Literal("{").suppress() + statements("else_body") + Literal("}").suppress(),
                         Literal("else").suppress() + Group(statement)("else_body")
                     ]))
                 )("conditional")



function_name = ~Literal("if") + Word(alphas+"_")
function_block = statements
function_arg = expression
function_args = Group(delimitedList(function_arg ,delim=','))
function << Group(function_name("name")+Literal("(").suppress()+function_args("args")+Literal(")").suppress()
                  + Optional(Literal("{").suppress() + function_block("body") + Literal("}").suppress()))("function")

gn_file = statements.ignore(comment)

def astToString(o, prefix=''):
    if hasattr(o, "toString"):
        return o.toString(prefix)
    elif type(o) is str:
        return prefix + "\"" + str(o) + "\""
    else:
        return prefix + str(o)

class Statement(object):
    def __init__(self, var, op, value):
        self.var = var
        self.op = op
        self.value = value
    def __repr__(self):
        return "Statement("+str(self.var)+" "+self.op+" "+str(self.value)+")"

class Operation(object):
    def __init__(self, op, arg1, arg2=None):
        self.op = op
        self.arg1 = arg1
        self.arg2 = arg2
    def __repr__(self):
        if self.arg2 is None:
            return "Operation("+str(self.op)+" "+str(self.arg1) +")"
        else:
            return "Operation("+str(self.arg1)+" "+str(self.op)+" "+str(self.arg2) +")"

class Func(object):
    def __init__(self, name, args, body=None):
        self.name = name
        self.args = args
        self.body = body
    def __repr__(self):
        s = "Func("+self.name+": "+str(self.args)+")"
        if self.body is not None:
            s += "{ " + str(self.body) + " }"
        return s
    def toString(self, prefix):
        s = prefix + "Func("+self.name+": "+ ", ".join([astToString(arg) for arg in self.args]) +")"
        if self.body is not None:
            s += "{\n"
            for statement in self.body:
                s += astToString(statement, prefix+"\t") + "\n"
            s += "}"
        return s

class Variable(object):
    def __init__(self, name):
        self.name = name
    def __repr__(self):
        return "Variable("+self.name+")"

class Conditional(object):
    def __init__(self, condition, body, else_body = None):
        self.condition = condition
        self.body = body
        self.else_body = else_body
    def __repr__(self):
        s = "Condition("+str(self.condition)+")" + "{ " + str(self.body) + " }"
        if self.else_body is not None:
            s += "else { " + str(self.else_body) + " }"
        return s
    def toString(self, prefix):
        s = prefix + "Condition("+ astToString(self.condition) +")"
        if self.body is not None:
            s += "{\n"
            for statement in self.body:
                s += astToString(statement, prefix+"\t") + "\n"
            s += "}"
        return s

def parseOperation(e):
    return Statement(e.variable, e.op, parseOperand(e.value))

def parseValue(e):
    if type(e) == ParseResults:
        if e.getName() == "list":
            return e.asList()
        elif len(e) == 1:
            return e[0]
        else:
            raise Exception("unknown value: " + repr(e))
    return e

def parseVariable(e):
    return Variable(e)

def parseOperand(e):
    if type(e) == ParseResults:
        if e.getName() == "variable":
            return parseVariable(e)
    return parseValue(e)

def parseExpression(e):
    if type(e) == ParseResults:
        if e.getName() == "operation":
            if len(e) == 2:
                return Operation(e[0], parseExpression(e[1]))
            elif len(e) == 3:
                return Operation(e[1], parseExpression(e[0]), parseExpression(e[2]))
            else:
                raise Exception("expected operation length to be 2 or 3: " + repr(e))
    return parseOperand(e)

def parseFunction(e):
    args = []
    for arg in e.args:
        args.append(parseExpression(arg))
    if e.body == "":
        body = None
    else:
        body = []
        for statement in e.body:
            body.append(parseStatement(statement))
    return Func(e.name[0], args, body)

def parseConditional(e):
    body = []
    for statement in e.body:
        body.append(parseStatement(statement))

    else_body = []
    if "else_body" not in e or e.else_body == "":
        else_body = None
    else:
        else_body = []
        for statement in e.else_body:
            else_body.append(parseStatement(statement))
    return Conditional(parseExpression(e.condition), body, else_body)

def parseStatement(e):
    if e.getName() == "function":
        return parseFunction(e)
    elif e.getName() == "conditional":
        return parseConditional(e)
    elif e.getName() == "operation":
        return parseOperation(e)
    else:
        raise Exception("unknown statement: " + repr(e))

def parseStatements(e):
    statements = []
    for element in e:
        statements.append(parseStatement(element))
    return statements

def parseFile(f):
    return parseStatements(gn_file.parseFile(sys.argv[1], True))
