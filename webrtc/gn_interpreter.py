#!/usr/bin/env python

import gn_parse
import sys
import os
import re
import subprocess

class Target:
    def __init__(self, type, scope):
        self.type = type
        self.scope = scope

    def __repr__(self):
        return "Target("+self.type+", "+str(self.scope)+")"

def no_eval_args(func):
    func.no_eval_args = True
    return func

def func_import(args, body, scope):
    file_name = scope.resolvePath(args[0])
    if os.path.isfile(file_name):
        evalFile(file_name, scope)
    else:
        print "Could not find import: " + file_name
    return None

def func_declare_args(args, body, scope):
    declare_scope = Scope(scope)
    for statement in body:
        eval(statement, declare_scope)
    scope.global_scope().default_args.update(declare_scope.values)
    return None

def func_set_default_toolchain(args, body, scope):
    scope.global_scope().set("default_toolchain", args[0])
    return None

def func_assert(args, body, scope):
    condition = args[0]
    message = "" if len(args) < 2 else args[1]
    if not condition:
        raise Exception("Assert failed: " + message)
    return None

@no_eval_args
def func_defined(args, body, scope):
    val = args[0]
    if type(val) == gn_parse.Member:
        o = eval(val.o, scope)
        return o.has(val.name)
    elif type(val) == gn_parse.Identifier:
        return scope.has(val.name)
    else:
        print "!!!!defined does not support: " + type(val)
        return False


def func_template(declaration_args, template_body, declaration_scope):
    name = declaration_args[0]
    def template_func(args, body, scope):
        target_name = args[0]
        call_body_scope = Scope(scope)
        for statement in body:
            eval(statement, call_body_scope)

        template_scope = OverlayScope(scope, {"invoker": call_body_scope, "target_name": target_name})
        for statement in template_body:
            eval(statement, template_scope)

    declaration_scope.declareTemplate(name, template_func)
    return None

def func_getenv(args, body, scope):
    var = args[0]
    return os.environ[var]

def func_rebase_path(args, body, scope):
    input = args[0]
    new_base = "" if len(args) < 2 else args[1]
    current_base = "." if len(args) < 3 else args[2]
    if current_base != ".":
        print "!!!!rebase with current_base not '.' is not supported"
    abspath = scope.resolvePath(input, isFile=False)
    if new_base == "":
        return abspath
    else:
        return os.path.relpath(abspath, new_base)

def func_exec_script(args, body, scope):
    filename = args[0]
    arguments = [] if len(args) < 2 else args[1]
    input_conversion = "" if len(args) < 3 else args[2]
    if len(args) > 3:
        print "!!!!More arguments specified to exec_script: " + str(args)

    command = ["python", scope.resolvePath(filename)] + arguments
    if input_conversion == "":
        subprocess.check_call(command)
    elif input_conversion == "value":
        result = subprocess.check_output(command)
        parser = gn_parse.build_parser("primary_expr")
        return parser.parse(result)
    elif input_conversion == "scope":
        result = subprocess.check_output(command)
        parser = gn_parse.build_parser()
        statements = parser.parse(result)
        scope = Scope(global_scope)
        if statements is not None:
            for statement in statements:
                eval(statement, scope)
        return scope
    else:
        print "!!!!unknown input conversion: ", input_conversion
    return None

def func_toolchain(args, body, scope):
    name = args[0]
    toolchain_scope = Scope(scope)
    for statement in body:
        eval(statement, toolchain_scope)

    scope.declareTarget(name, Target("toolchain", toolchain_scope))

    return None

def func_toolchain_args(args, body, scope):
    args_scope = Scope(scope)
    for statement in body:
        eval(statement, args_scope)

    scope.set("toolchain_args", args_scope)
    return None

def func_tool(args, body, scope):
    name = args[0]
    tool_scope = Scope(scope)
    for statement in body:
        eval(statement, tool_scope)

    scope.declareTarget(name, Target("tool", tool_scope))

    return None



def func_source_set(args, body, scope):
    name = args[0]
    source_set_scope = Scope(scope)
    source_set_scope.set('configs', [])
    source_set_scope.set('sources', [])
    source_set_scope.set('deps', [])
    for statement in body:
        eval(statement, source_set_scope)

    scope.declareTarget(name, Target("source_set", source_set_scope))

    return None

def func_static_library(args, body, scope):
    name = args[0]
    static_library_scope = Scope(scope)
    static_library_scope.set('configs', [])
    static_library_scope.set('sources', [])
    static_library_scope.set('deps', [])
    for statement in body:
        eval(statement, static_library_scope)

    scope.declareTarget(name, Target("static_library", static_library_scope))

    return None

def func_executable(args, body, scope):
    name = args[0]
    executable_scope = Scope(scope)
    executable_scope.set('configs', [])
    executable_scope.set('sources', [])
    executable_scope.set('deps', [])
    for statement in body:
        eval(statement, executable_scope)

    scope.declareTarget(name, Target("executable", executable_scope))

    return None

def func_group(args, body, scope):
    name = args[0]
    group_scope = Scope(scope)
    group_scope.set('deps', [])
    for statement in body:
        eval(statement, group_scope)

    scope.declareTarget(name, Target("group", group_scope))

    return None

def func_config(args, body, scope):
    name = args[0]
    group_scope = Scope(scope)
    for statement in body:
        eval(statement, group_scope)

    scope.declareTarget(name, Target("config", group_scope))

    return None

def func_set_sources_assignment_filter(args, body, scope):
    print "!!!!set_sources_assignment_filter not implemented"
    return None

def func_set_defaults(args, body, scope):
    print "!!!!set_defaults not implemented"
    return None



class Scope(object):
    def __init__(self, parent):
        self.parent = parent
        self.values = dict()
        self.targets = dict()
        self.templates = dict()

    def global_scope(self):
        return self.parent.global_scope()

    def resolvePath(self, path, **kwargs):
        return self.parent.resolvePath(path, **kwargs)

    def declareTarget(self, name, target):
        self.targets[name] = target

    def getTarget(self, name):
        if name in self.targets:
            return self.targets[name]
        else:
            return None

    def declareTemplate(self, name, template):
        self.templates[name] = template

    def getTemplate(self, name):
        if name in self.templates:
            return self.templates[name]
        else:
            return None

    def has(self, name):
        if name in self.values:
            return True
        elif self.parent is not None:
            return self.parent.has(name)
        return False

    def get(self, name):
        if name in self.values:
            return self.values[name]
        elif self.parent is not None:
            return self.parent.get(name)
        else:
            raise Exception("Could not resolve: " + name)

    def set(self, name, value):
        self.values[name] = value

class OverlayScope(object):
    def __init__(self, parent, values):
        self.parent = parent
        self.values = values

    def resolvePath(self, path, **kwargs):
        return self.parent.resolvePath(path, **kwargs)

    def global_scope(self):
        return self.parent.global_scope()

    def declareTarget(self, name, target):
        self.parent.declareTarget(name, target)

    def getTarget(self, name):
        return self.parent.getTarget(name)

    def declareTemplate(self, name, template):
        self.parent.declareTemplate(name, template)

    def getTemplate(self, name):
        return self.parent.getTemplate(name)

    def has(self, name):
        if name in self.values:
            return True
        return self.parent.has(name)

    def get(self, name):
        if name in self.values:
            return self.values[name]
        return self.parent.get(name)

    def set(self, name, value):
        return self.parent.set(name, value)



class FileScope(object):
    def __init__(self, parent, path):
        self.parent = parent
        self.path = path

    def resolvePath(self, path, **kwargs):
        if path.startswith("//") or path.startswith("/"):
            return self.parent.resolvePath(path, **kwargs)
        return self.parent.resolvePath(os.path.join(os.path.dirname(self.path), path), **kwargs)

    def global_scope(self):
        return self.parent.global_scope()

    def declareTarget(self, name, target):
        self.parent.declareTarget(name, target)

    def getTarget(self, name):
        return self.parent.getTarget(name)

    def declareTemplate(self, name, template):
        self.parent.declareTemplate(name, template)

    def getTemplate(self, name):
        return self.parent.getTemplate(name)

    def has(self, name):
        return self.parent.has(name)

    def get(self, name):
        return self.parent.get(name)

    def set(self, name, value):
        return self.parent.set(name, value)


class GlobalScope(Scope):
    def __init__(self, root_dirs):
        super(GlobalScope, self).__init__(None)

        self.root_dirs = root_dirs
        self.default_args = dict()

        self.default_args['os'] = "linux"
        self.default_args['host_os'] = "linux"
        self.default_args['host_cpu'] = "x64"
        self.default_args['current_os'] = "linux"
        self.default_args['current_cpu'] = "x64"
        self.default_args['target_os'] = "linux"
        self.default_args['target_cpu'] = "x64"

        self.default_args['build_cpu_arch'] = "x64"
        self.default_args['cpu_arch'] = "x64"

        self.set("is_clang", False)

    def global_scope(self):
        return self

    def resolvePath(self, path, buildFile=False, isFile=True):
        potential_paths = []

        if path.startswith("//"):
            for root_dir in self.root_dirs:
                potential_paths.append(os.path.join(root_dir, path[2:]))
        elif path.startswith("/"):
            potential_paths.append(path)
        else:
            raise Exception("Cannot resolve non-absolute path against root scope: " + path)

        for potential_path in potential_paths:
            if not isFile or os.path.isfile(potential_path):
                return os.path.normpath(potential_path)
            if buildFile and os.path.isdir(potential_path):
                f = os.path.join(potential_path, "BUILD.gn")
                if os.path.isfile(f):
                    return os.path.normpath(f)
        raise Exception("Could not find file: " + path)

    def has(self, name):
        if super(GlobalScope, self).has(name):
            return True
        elif name in self.default_args:
            return True
        else:
            return False

    def get(self, name):
        if super(GlobalScope, self).has(name):
            return super(GlobalScope, self).get(name)
        elif name in self.default_args:
            return self.default_args[name]
        else:
            raise Exception("Could not resolve: " + name)


def eval(statement, scope):
    if statement is None:
        return None
    if type(statement) is gn_parse.Condition:
        result = eval(statement.condition, scope)
        if result:
            for s in statement.body:
                eval(s, scope)
        elif type(statement.else_body) is gn_parse.Condition:
            return eval(statement.else_body, scope)
        elif statement.else_body is not None:
            for s in statement.else_body:
                eval(s, scope)
        return None
    if type(statement) is gn_parse.Call:
        #print "Calling: ", statement.name, statement.args
        func = globals().get("func_"+statement.name, None)
        if func is None:
            func = scope.getTemplate(statement.name)
        if func is None:
            raise Exception("Could not find function: " + statement.name)
        if hasattr(func, "no_eval_args") and func.no_eval_args:
            args = statement.args
        else:
            args = map(lambda arg: eval(arg, scope), statement.args)
        return func(args, statement.body, scope)
    if type(statement) is gn_parse.Identifier:
        return scope.get(statement.name)
    if type(statement) is gn_parse.Operation:
        if statement.op == '!':
            return not eval(statement.arg1, scope)
        if statement.op == '==':
            return eval(statement.arg1, scope) == eval(statement.arg2, scope)
        if statement.op == '!=':
            return eval(statement.arg1, scope) != eval(statement.arg2, scope)
        if statement.op == '+':
            return eval(statement.arg1, scope) + eval(statement.arg2, scope)
        if statement.op == '||':
            if eval(statement.arg1, scope):
                return True
            elif eval(statement.arg2, scope):
                return True
            else:
                return False
        if statement.op == '&&':
            if not eval(statement.arg1, scope):
                return False
            if not eval(statement.arg2, scope):
                return False
            return True
        if statement.op == '>=':
            return eval(statement.arg1, scope) >= eval(statement.arg2, scope)
        if statement.op == '<=':
            return eval(statement.arg1, scope) <= eval(statement.arg2, scope)
        if statement.op == '>':
            return eval(statement.arg1, scope) > eval(statement.arg2, scope)
        if statement.op == '<':
            return eval(statement.arg1, scope) < eval(statement.arg2, scope)

        print "!!!!Unsupported op: " + statement.op

    if type(statement) is gn_parse.Assignment:
        if statement.op == "=":
            current_value = scope.get(statement.var) if scope.has(statement.var) else None
            if current_value is None or type(current_value) is not list or len(current_value) == 0:
                scope.set(statement.var, eval(statement.value, scope))
            else:
                raise Exception(statement.var + " was not unset or an empty list")
        elif statement.op == "+=":
            scope.get(statement.var).extend(eval(statement.value, scope))
        elif statement.op == "-=":
            to_remove = eval(statement.value, scope)
            value = scope.get(statement.var)
            for item in to_remove:
                if item in value:
                    value.remove(item)
        else:
            print "!!!!Unsupported assignment: " + statement.op
        return None

    if type(statement) is gn_parse.Member:
        return eval(statement.o, scope).get(statement.name)

    if type(statement) is gn_parse.ArrayIndex:
        return eval(statement.o, scope)[statement.index]

    if type(statement) is bool or type(statement) is int or type(statement) is list:
        return statement

    if type(statement) is str:
        find = re.compile(r'\$([a-zA-Z_][a-zA-Z_0-9]*)')
        def id_resolver(match):
            return scope.get(match.group(1))
        return find.sub(id_resolver, statement)

    print "eval", type(statement), statement
    return None

def evalFile(f, scope):
    parser = gn_parse.build_parser()

    scope = FileScope(scope, f)
    print "Loading: ", f
    statements = parser.parse(file(f).read())

    if statements is not None:
        for statement in statements:
            result = eval(statement, scope)
            if result is not None:
                print result

    return scope

# Initialize global scope
root_dirs = ["/home/mitchell/deps_workspace/src/webrtc_ros/webrtc/webrtc_src/",
             "/home/mitchell/deps_workspace/src/webrtc_ros/webrtc/chromium_src/",
             "/home/mitchell/deps_workspace/src/webrtc_ros/webrtc/shim"]
global_scope = GlobalScope(root_dirs)
global_scope.set("root_build_dir" , "/home/mitchell/deps_workspace/src/webrtc_ros/webrtc/build")
global_scope.set("build_with_chromium" , False)
global_scope.set("use_openssl" , True)

evalFile(global_scope.resolvePath("//build/config/BUILDCONFIG.gn"), global_scope)


# load toolchain
if not global_scope.has("current_toolchain"):
    global_scope.set("current_toolchain", global_scope.get("default_toolchain"))




target_cache = dict()


def resolveTarget(name, resolve_scope):
    s = name.split(':')
    if len(s) == 1:
        filename = s[0]
        target_name = ""
    elif len(s) == 2:
        filename = s[0]
        target_name = s[1]
    else:
        raise Exception("Error parsing target name: "+name)

    file_path = resolve_scope.resolvePath(filename, buildFile=True)

    if file_path in target_cache:
        if target_name == "":
            return target_cache.get(file_path)
        else:
            return target_cache.get(file_path).get(target_name)

    scope = Scope(global_scope)
    file_scope = evalFile(file_path, scope)

    target_cache[file_path] = scope.targets

    for (file_target_name, file_target) in scope.targets.iteritems():
        print file_target.type, file_path, file_target_name
        for dep_key in ["deps", "data_deps", "public_deps", "configs", "public_configs"]:
            deps = file_target.scope.get(dep_key) if file_target.scope.has(dep_key) else []
            for dep in deps:
                resolveTarget(dep, file_scope)

    if target_name == "":
        return scope.targets
    else:
        if target_name not in scope.targets:
            raise Exception("Resolved file: " + file_path + ", but did not find target: " + target_name)
        return scope.targets[target_name]




resolveTarget(global_scope.get("current_toolchain"), global_scope)


resolveTarget(os.path.abspath(sys.argv[1]), global_scope)

#for (target_name, target) in result.iteritems():
#    print target.type, target_name
#    deps = target.scope.get("deps") if target.scope.has("deps") else []
#    for dep in deps:
#        print "\t"+dep+" -> " + str(resolveTarget(dep, file_scope)[0])

