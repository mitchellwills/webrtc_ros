#!/usr/bin/env python

import sys
import ply.lex as lex
import ply.yacc as yacc

def astToString(o, prefix=''):
    if hasattr(o, "toString"):
        return o.toString(prefix)
    elif type(o) is str:
        return prefix + "\"" + str(o) + "\""
    else:
        return prefix + str(o)

class Identifier(object):
    def __init__(self, name):
        self.name = name
    def __repr__(self):
        return "Identifier("+self.name+")"

class Member(object):
    def __init__(self, o, name):
        self.o = o
        self.name = name
    def __repr__(self):
        return "Member("+str(self.o)+", "+self.name+")"

class ArrayIndex(object):
    def __init__(self, o, index):
        self.o = o
        self.index = index
    def __repr__(self):
        return "ArrayIndex("+str(self.o)+", "+str(self.index)+")"

class Operation(object):
    def __init__(self, op, arg1, arg2=None):
        self.op = op
        self.arg1 = arg1
        self.arg2 = arg2
    def __repr__(self):
        if self.arg2 is not None:
            return "Operation("+self.op+", " +str(self.arg1)+", " +str(self.arg2)+ ")"
        else:
            return "Operation("+self.op+", " +str(self.arg1)+ ")"

class Assignment(object):
    def __init__(self, var, op, value):
        self.var = var
        self.op = op
        self.value = value
    def __repr__(self):
        return "Statement("+str(self.var)+" "+self.op+" "+str(self.value)+")"

class Call(object):
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

class Condition(object):
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

def build_parser(start="file"):
    reserved = {
       'if' : 'IF',
       'else' : 'ELSE',
    }

    tokens = [
        'INTEGER',
        'BOOL',
        'PLUS',
        'MINUS',
        'EQUALS',
        'PLUS_EQUALS',
        'MINUS_EQUALS',
        'LPAREN',
        'RPAREN',
        'LBRACE',
        'RBRACE',
        'LSBRACE',
        'RSBRACE',
        'IDENTIFIER',
        'STRING',
        'BOOL_OR',
        'BOOL_AND',
        'LT_EQ',
        'GT_EQ',
        'LT',
        'GT',
        'EQ',
        'NOT_EQ',
        'COMMA',
        'DOT',
        'NOT',
    ] + list(reserved.values())

    t_PLUS    = r'\+'
    t_MINUS   = r'-'
    t_COMMA   = r','
    t_EQUALS  = r'='
    t_PLUS_EQUALS  = r'\+='
    t_MINUS_EQUALS  = r'-='
    t_LPAREN  = r'\('
    t_RPAREN  = r'\)'
    t_LBRACE  = r'{'
    t_RBRACE  = r'}'
    t_LSBRACE  = r'\['
    t_RSBRACE  = r'\]'
    t_DOT  = r'\.'
    t_BOOL_OR  = r'\|\|'
    t_BOOL_AND  = r'&&'
    t_LT_EQ  = r'<='
    t_GT_EQ  = r'>='
    t_LT  = r'<'
    t_GT  = r'>'
    t_EQ  = r'=='
    t_NOT_EQ  = r'!='
    t_NOT  = r'!'
    t_ignore_COMMENT = r'\#.*'
    t_ignore = " \t\r"

    def t_INTEGER(t):
        r'-?\d+'
        t.value = int(t.value)
        return t

    def t_STRING(t):
        r'"(?:[^"\\]|\\.)*"'
        # TODO need to handle escape sequences
        t.value = t.value[1:-1].replace('\\"', '"')
        return t

    def t_IDENTIFIER(t):
        r'[a-zA-Z_][a-zA-Z_0-9]*'
        if t.value == 'true':
            t.value = True
            t.type = 'BOOL'
        elif t.value == 'false':
            t.value = False
            t.type = 'BOOL'
        else:
            t.type = reserved.get(t.value, 'IDENTIFIER')
            t.value = Identifier(t.value)
        return t

    # Define a rule so we can track line numbers
    def t_newline(t):
        r'\n+'
        t.lexer.lineno += len(t.value)

    # Error handling rule
    def t_error(t):
        print("Illegal character '%s'" % t.value[0])
        t.lexer.skip(1)

    lexer = lex.lex(lextab="cache_lex_"+start,debug=False,optimize=1)

    def p_file(p):
        '''
        file : statement_list
        file :
        '''
        if len(p) == 2:
            p[0] = p[1]
        else:
            p[0] = None

    # statements
    def p_statement(p):
        '''
        statement : call
                  | assignment
                  | condition
        '''
        p[0] = p[1]

    def p_assignment(p):
        '''
        assignment : IDENTIFIER assign_op expr
        '''
        p[0] = Assignment(p[1].name, p[2], p[3])

    def p_call(p):
        '''
        call : IDENTIFIER LPAREN expr_list RPAREN block_or_empty
        call : IDENTIFIER LPAREN RPAREN block_or_empty
        '''
        if len(p) == 6:
            p[0] = Call(p[1].name, p[3], p[5])
        else:
            p[0] = Call(p[1].name, [], p[4])


    def p_condition(p):
        '''
        condition : IF LPAREN expr RPAREN block
        condition : IF LPAREN expr RPAREN block ELSE condition
        condition : IF LPAREN expr RPAREN block ELSE block
        '''
        if len(p) == 8:
            p[0] = Condition(p[3], p[5], p[7])
        else:
            p[0] = Condition(p[3], p[5])

    def p_block(p):
        '''
        block : LBRACE statement_list RBRACE
        block : LBRACE RBRACE
        '''
        if len(p) == 4:
            p[0] = p[2]
        else:
            p[0] = []

    def p_block_or_empty(p):
        '''
        block_or_empty : block
                       |
        '''
        if len(p) == 1:
            p[0] = None
        else:
            p[0] = p[1]

    def p_statement_list(p):
        '''
        statement_list : statement_list statement
        statement_list : statement
        '''
        if len(p) == 2:
            p[0] = [p[1]]
        else:
            p[0] = p[1]
            p[0].append(p[2])


    # expressions
    def p_expr(p):
        '''
        expr : unary_expr
             | expr PLUS expr
             | expr MINUS expr
             | expr GT expr
             | expr LT expr
             | expr GT_EQ expr
             | expr LT_EQ expr
             | expr EQ expr
             | expr NOT_EQ expr
             | expr BOOL_AND expr
             | expr BOOL_OR expr
        '''
        if len(p) == 2:
            p[0] = p[1]
        else:
            p[0] = Operation(p[2], p[1], p[3])

    def p_unary_expr(p):
        '''
        unary_expr : primary_expr
                   | unary_op unary_expr
        '''
        if len(p) == 2:
            p[0] = p[1]
        else:
            p[0] = Operation(p[1], p[2])


    def p_primary_expr(p):
        '''primary_expr : IDENTIFIER
                | INTEGER
                | STRING
                | BOOL
                | call
                | LPAREN expr RPAREN
                | array '''
        if len(p) == 4:
            p[0] = p[2]
        else:
            p[0] = p[1]
    def p_primary_expr_member(p):
        '''primary_expr : IDENTIFIER DOT IDENTIFIER'''
        p[0] = Member(p[1], p[3].name)
    def p_primary_expr_array_index(p):
        '''primary_expr : IDENTIFIER LSBRACE expr RSBRACE'''
        p[0] = ArrayIndex(p[1], p[3])

    def p_array(p):
        '''
        array : LSBRACE expr_list RSBRACE
        array : LSBRACE expr_list COMMA RSBRACE
        array : LSBRACE RSBRACE
        '''
        if len(p) == 3:
            p[0] = []
        else:
            p[0] = p[2]


    def p_expr_list(p):
        '''
        expr_list : expr_list COMMA expr
        expr_list : expr
        '''
        if len(p) == 2:
            p[0] = [p[1]]
        else:
            p[0] = p[1]
            p[0].append(p[3])


    # operators
    def p_assign_op(p):
        '''
        assign_op : EQUALS
                  | PLUS_EQUALS
                  | MINUS_EQUALS
        '''
        p[0] = p[1]

    def p_unary_op(p):
        '''
        unary_op : NOT
        '''
        p[0] = p[1]

    precedence = (
        ('right', 'NOT'),
        ('left', 'BOOL_OR'),
        ('left', 'BOOL_AND'),
        ('left', 'EQ', 'NOT_EQ'),
        ('left', 'LT', 'LT_EQ', 'GT', 'GT_EQ'),
        ('left', 'PLUS', 'MINUS'),
    )

    def p_error(p):
        print("Syntax error in input!", p)


    parser = yacc.yacc(tabmodule="cache_yacc_"+start,debug=False)
    return parser
