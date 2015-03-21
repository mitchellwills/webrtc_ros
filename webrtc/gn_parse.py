#!/usr/bin/env python

import sys
from pyparsing import Word, alphas, restOfLine, OneOrMore, Or, QuotedString, ZeroOrMore, Optional, Group, Literal, delimitedList, Empty, ParseResults, Forward

comment = Literal("#") + restOfLine

variable = Word(alphas+"_")
string = QuotedString(quoteChar="\"", escChar="\\")

non_list_value = Or([string])
gn_list = Literal("[").suppress() \
          + Group(Or([delimitedList(non_list_value, delim=","), ZeroOrMore(non_list_value + Literal(",").suppress())]))("list") \
          + Literal("]").suppress()
value = Or([non_list_value, gn_list])

simple_expression = Or([value, variable])

comparison = Forward()
inversion = Literal("!") + simple_expression
expression = Or([simple_expression, inversion, comparison])

comparison << Group(simple_expression("expL") + Or([Literal("!="), Literal("=="), Literal("||")])("op") + simple_expression("expR"))("comparison")

operation = Group(variable("variable") +  Or([Literal("-="), Literal("+="), Literal("=")])("op") + value("value"))("operation")
function = Forward()
conditional = Forward()
statement = Or([operation, conditional, function])
statements = ZeroOrMore(statement)

conditional_condition = expression
conditional_body = statements
conditional << Group(Literal("if").suppress()+Literal("(").suppress()+conditional_condition("condition")+Literal(")").suppress()
                     + Literal("{").suppress() + conditional_body("body") + Literal("}").suppress()
                     + Optional(
                         Literal("else")
                         + Literal("{").suppress() + conditional_body("else_body") + Literal("}").suppress()
                     )
                 )("conditional")



function_name = Word(alphas+"_")
function_block = statements
function_arg = expression
function_args = Group(delimitedList(function_arg ,delim=','))
function << Group(function_name("name")+Literal("(").suppress()+function_args("args")+Literal(")").suppress()
                  + Optional(Literal("{").suppress() + function_block("body") + Literal("}").suppress()))("function")

gn_file = statements.ignore(comment)


class Statement(object):
    def __init__(self, var, op, value):
        self.var = var
        self.op = op
        self.value = value
    def __repr__(self):
        return "Statement("+str(self.var)+" "+self.op+" "+str(self.value)+")"

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

class Comparison(object):
    def __init__(self, expL, op, expR):
        self.expL = expL
        self.op = op
        self.expR = expR
    def __repr__(self):
        return "Comparison("+str(self.expL)+" "+self.op+" "+str(self.expR)+")"

def parseOperation(e):
    return Statement(e.variable, e.op, parseValue(e.value))

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
    # TODO
    return e

def parseComparison(e):
    return Comparison(parseExpression(e.expL), e.op, parseExpression(e.expR))


def parseExpression(e):
    if type(e) == ParseResults:
        if e.getName() == "variable":
            return parseVariable(e)
        elif e.getName() == "comparison":
            return parseComparison(e)
        else:
            raise Exception("unknown expression: " + repr(e))
    else:
        return parseValue(e)

def parseFunction(e):
    args = []
    for arg in e.args:
        args.append(parseValue(arg))
    if e.body == "":
        body = None
    else:
        body = []
        for statement in e.body:
            body.append(parseStatement(statement))
    return Func(e.name, args, body)

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


statements = parseStatements(gn_file.parseFile(sys.argv[1]))
for statement in statements:
    print statement
