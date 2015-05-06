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

def build_parser():
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
        r'"[^"]*"'
        # TODO need to handle escape sequences
        t.value = t.value[1:-1]
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
        return t

    # Define a rule so we can track line numbers
    def t_newline(t):
        r'\n+'
        t.lexer.lineno += len(t.value)

    # Error handling rule
    def t_error(t):
        print("Illegal character '%s'" % t.value[0])
        t.lexer.skip(1)

    lexer = lex.lex()

    def p_file(p):
        '''
        file : statement_list
        '''
        p[0] = p[1]

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
        p[0] = Assignment(p[1], p[2], p[3])

    def p_call(p):
        'call : IDENTIFIER LPAREN expr_list RPAREN block_or_empty'
        p[0] = Call(p[1], p[3], p[5])


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
             | expr binary_op expr
        '''
        if len(p) == 2:
            p[0] = p[1]
        else:
            p[0] = (p[2], p[1], p[3])

    def p_unary_expr(p):
        '''
        unary_expr : primary_expr
                   | unary_op unary_expr
        '''
        if len(p) == 2:
            p[0] = p[1]
        else:
            p[0] = (p[1], p[2])


    def p_primary_expr(p):
        '''primary_expr : IDENTIFIER
                | INTEGER
                | STRING
                | BOOL
                | call
                | LPAREN expr RPAREN
                | array '''
        p[0] = p[1]
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

    def p_binary_op(p):
        '''
        binary_op : PLUS
                  | MINUS
                  | GT
                  | LT
                  | GT_EQ
                  | LT_EQ
                  | EQ
                  | NOT_EQ
                  | BOOL_AND
                  | BOOL_OR
        '''
        p[0] = p[1]

    precedence = (
        ('left', 'PLUS', 'MINUS'),
        ('left', 'LT', 'LT_EQ', 'GT', 'GT_EQ'),
        ('left', 'EQ', 'NOT_EQ'),
        ('left', 'BOOL_AND'),
        ('left', 'BOOL_OR'),
        ('right', 'NOT'),
    )


    def p_error(p):
        print("Syntax error in input!", p)


    parser = yacc.yacc()
    return parser
