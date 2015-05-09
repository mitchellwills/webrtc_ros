#!/usr/bin/env python

import gn_parse
import sys
import os
import re
import subprocess
import itertools
import traceback
import copy

def print_exc_plus():
    """
    Print the usual traceback information, followed by a listing of all the
    local variables in each frame.
    """
    tb = sys.exc_info()[2]
    while 1:
        if not tb.tb_next:
            break
        tb = tb.tb_next
    stack = []
    f = tb.tb_frame
    while f:
        stack.append(f)
        f = f.f_back
    stack.reverse()
    traceback.print_exc()
    print "Locals by frame, innermost last"
    for frame in stack:
        print
        print "Frame %s in %s at line %s" % (frame.f_code.co_name,
                                             frame.f_code.co_filename,
                                             frame.f_lineno)
        if frame.f_code.co_name == "resolveTarget":
            if "file_scope" in frame.f_locals:
                print "\tresolving " + frame.f_locals["name"] + " in " + frame.f_locals["file_scope"].path
            else:
                print "\tresolving " + frame.f_locals["name"]

class TargetDefinition:
    def __init__(self, type, scope):
        self.type = type
        self.scope = scope

    def __repr__(self):
        return "TargetDefinition("+self.type+", "+str(self.scope)+")"

def no_eval_args(func):
    func.no_eval_args = True
    return func

def copyDefaults(name, s):
    if name in s.global_scope().defaults:
        s.copyFrom(s.global_scope().defaults[name])

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
        copyDefaults(name, call_body_scope)
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
    file_dependencies = [] if len(args) < 4 else args[3]

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

    scope.declareTarget(name, TargetDefinition("toolchain", toolchain_scope))

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

    scope.declareTarget(name, TargetDefinition("tool", tool_scope))

    return None


def filter_sources(scope):
    if not scope.has("sources"):
        return
    filters = scope.get("$sources_assignment_filter")
    sources = scope.get("sources")
    def should_keep(item):
        return all(f.match(item) is None for f in filters)
    sources[:] = filter(should_keep, sources)

def func_source_set(args, body, scope):
    name = args[0]
    source_set_scope = Scope(scope)
    source_set_scope.set('configs', [])
    source_set_scope.set('sources', [])
    source_set_scope.set('include_dirs', [])
    source_set_scope.set('deps', [])
    copyDefaults("source_set", source_set_scope)
    for statement in body:
        eval(statement, source_set_scope)

    filter_sources(source_set_scope)
    sources = source_set_scope.get('sources')
    sources[:] = map(lambda f: scope.resolvePath(f, isFile=False), sources) # todo should theoretically care that they are a file, but some names are bad
    include_dirs = source_set_scope.get('include_dirs')
    include_dirs[:] = map(lambda f: scope.resolvePath(f, isFile=False), include_dirs) # todo should theoretically care that they are a file, but some names are bad

    scope.declareTarget(name, TargetDefinition("source_set", source_set_scope))

    return None

def func_static_library(args, body, scope):
    name = args[0]
    static_library_scope = Scope(scope)
    static_library_scope.set('configs', [])
    static_library_scope.set('sources', [])
    static_library_scope.set('include_dirs', [])
    static_library_scope.set('deps', [])
    copyDefaults("static_library", static_library_scope)
    for statement in body:
        eval(statement, static_library_scope)

    filter_sources(static_library_scope)
    sources = static_library_scope.get('sources')
    sources[:] = map(lambda f: scope.resolvePath(f, isFile=False), sources) # todo should theoretically care that they are a file, but some names are bad
    include_dirs = static_library_scope.get('include_dirs')
    include_dirs[:] = map(lambda f: scope.resolvePath(f, isFile=False), include_dirs) # todo should theoretically care that they are a file, but some names are bad

    scope.declareTarget(name, TargetDefinition("static_library", static_library_scope))

    return None

def func_executable(args, body, scope):
    name = args[0]
    executable_scope = Scope(scope)
    executable_scope.set('configs', [])
    executable_scope.set('sources', [])
    executable_scope.set('include_dirs', [])
    executable_scope.set('deps', [])
    copyDefaults("executable", executable_scope)
    for statement in body:
        eval(statement, executable_scope)

    filter_sources(executable_scope)
    sources = executable_scope.get('sources')
    sources[:] = map(lambda f: scope.resolvePath(f, isFile=False), sources) # todo should theoretically care that they are a file, but some names are bad
    include_dirs = executable_scope.get('include_dirs')
    include_dirs[:] = map(lambda f: scope.resolvePath(f, isFile=False), include_dirs) # todo should theoretically care that they are a file, but some names are bad

    scope.declareTarget(name, TargetDefinition("executable", executable_scope))

    return None

def func_config(args, body, scope):
    name = args[0]
    config_scope = Scope(scope)
    config_scope.set('include_dirs', [])
    copyDefaults("config", config_scope)
    for statement in body:
        eval(statement, config_scope)

    include_dirs = config_scope.get('include_dirs')
    include_dirs[:] = map(lambda f: scope.resolvePath(f, isFile=False), include_dirs) # todo should theoretically care that they are a file, but some names are bad

    scope.declareTarget(name, TargetDefinition("config", config_scope))

    return None

def func_group(args, body, scope):
    name = args[0]
    group_scope = Scope(scope)
    copyDefaults("group", group_scope)
    group_scope.set('deps', [])
    for statement in body:
        eval(statement, group_scope)

    scope.declareTarget(name, TargetDefinition("group", group_scope))

    return None

def func_set_defaults(args, body, scope):
    target_type = args[0]
    defaults_scope = Scope(scope)
    for statement in body:
        eval(statement, defaults_scope)

    scope.global_scope().defaults[target_type] = defaults_scope
    return None

def func_set_sources_assignment_filter(args, body, scope):
    filters = map(lambda pattern: re.compile("^"+pattern.replace("*", ".*").replace("\\b", "(?:^|(?:.*/))")+"$"), args[0])
    scope.set("$sources_assignment_filter", filters)
    return None



class Scope(object):
    def __init__(self, parent):
        self.parent = parent
        self.values = dict()
        self.targets = dict()
        self.templates = dict()
        self.sources_assignement_filter = []

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

    def copyFrom(self, other):
        self.values.update(copy.deepcopy(other.values))

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
        self.defaults = dict()
        self.default_args = dict()

        self.default_args['os'] = "linux"
        self.default_args['cpu_arch'] = "x64"

        self.default_args['host_os'] = "linux"
        self.default_args['host_cpu'] = "x64"
        self.default_args['current_os'] = "linux"
        self.default_args['current_cpu'] = "x64"
        self.default_args['target_os'] = "linux"
        self.default_args['target_cpu'] = "x64"
        self.default_args['build_cpu_arch'] = "x64"

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

    if type(statement) is bool or type(statement) is int:
        return statement

    if type(statement) is list:
        return map(lambda item: eval(item, scope), statement)

    if type(statement) is str:
        find = re.compile(r'(?<!\\)\$(?:(?:([a-zA-Z_][a-zA-Z_0-9]*))|(?:\{([a-zA-Z_][a-zA-Z_0-9]*)\}))')
        def id_resolver(match):
            id = match.group(1)
            if id is None:
                id = match.group(2)
            return str(scope.get(id))
        out = find.sub(id_resolver, statement).replace('\\"', '"').replace('\\\\', '\\').replace('\\$', '$')
        #print statement, "->", out
        return out

    print "!!!!!Unknown statement type", type(statement), statement
    return None

parsed_files = dict()

def evalFile(f, scope):
    parser = gn_parse.build_parser()

    scope = FileScope(scope, f)
    if f not in parsed_files:
        print "Parsing: ", f
        statements = parsed_files[f] = parser.parse(file(f).read())
    else:
        statements = parsed_files[f]

    if statements is not None:
        for statement in statements:
            result = eval(statement, scope)
            if result is not None:
                print result

    return scope

# Initialize global scope
root_dirs = ["/home/mitchell/deps_workspace/src/webrtc_ros/webrtc/webrtc_src/",
             "/home/mitchell/deps_workspace/src/webrtc_ros/webrtc/chromium_src/",
             "/home/mitchell/deps_workspace/src/webrtc_ros/webrtc/yuv_src/",
             "/home/mitchell/deps_workspace/src/webrtc_ros/webrtc/shim"]
global_scope = GlobalScope(root_dirs)

evalFile(global_scope.resolvePath("//webrtc_ros/config.gni"), global_scope)

evalFile(global_scope.resolvePath("//build/config/BUILDCONFIG.gn"), global_scope)

# load toolchain
if not global_scope.has("current_toolchain"):
    global_scope.set("current_toolchain", global_scope.get("default_toolchain"))

# based on http://www.peterbe.com/plog/uniqifiers-benchmark
def uniquify(seq):
    seen = set()
    return [x for x in seq if x not in seen and not seen.add(x)]


class Target:
    def __init__(self, type, properties, public_deps):
        self.type = type
        self.properties = properties
        self.public_deps = public_deps

    def __repr__(self):
        return "Target("+self.type+")"

def bfs_list(root, expandFunc):
    queue = list(root)
    result = []
    while queue:
        item = queue.pop(0)
        result.append(item)
        queue.extend(expandFunc(item))
    return result

config_keys = ["cflags", "cflags_c", "cflags_cc", "cflags_objc", "cflags_objcc", "defines", "include_dirs", "ldflags", "lib_dirs", "libs"]
configurable_types = ["static_library", "source_set", "config", "executable"]
inheritable_configurable_types = ["source_set", "config"]
source_types = ["static_library", "source_set", "executable"]
def buildTarget(target, resolved_deps, verbose=False):
    private_deps = list(itertools.chain(*resolved_deps.values()))
    initial_public_deps = list(itertools.chain(*map(lambda t: t[1].public_deps, private_deps)))
    public_deps = uniquify(bfs_list(initial_public_deps, lambda t: t[1].public_deps))
    all_deps = public_deps + private_deps

    properties = {"sources": [], "link": []}

    if target.scope.has("sources"):
        properties["sources"].extend(target.scope.get("sources"))

    if target.type in configurable_types:
        for key in config_keys:
            properties[key] = []
            if target.scope.has(key):
                properties[key].extend(target.scope.get(key))

    if target.type in source_types:
        for dep in all_deps:
            dep_name = dep[0]
            dep_target = dep[1]

            if dep_target.type == "source_set":
                if "sources" in dep_target.properties:
                    properties["sources"].extend(dep_target.properties["sources"])
            if dep_target.type == "static_library":
                properties["link"].append(dep_name)

    if target.type in inheritable_configurable_types:
        for dep in all_deps:
            dep_name = dep[0]
            dep_target = dep[1]

            if dep_target.type in configurable_types:
                for key in config_keys:
                    if key in dep_target.properties:
                        properties[key].extend(dep_target.properties[key])

    properties["sources"] = uniquify(properties["sources"])

    if target.type in configurable_types:
        for key in config_keys:
            properties[key] = uniquify(properties[key])

    return Target(target.type, properties, list(itertools.chain(resolved_deps["public_configs"], resolved_deps["public_deps"])))

file_eval_cache = dict() # cache of files resolved in root scope
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

    if file_path not in file_eval_cache:
        scope = Scope(global_scope)
        file_scope = evalFile(file_path, scope)
        file_eval_cache[file_path] = (file_scope, scope)
    else:
        file_scope, scope = file_eval_cache[file_path]

    if file_path not in target_cache:
        target_cache[file_path] = {}

    for file_target_name in scope.targets:
        if target_name == "" or file_target_name == target_name:
            if file_target_name not in target_cache[file_path]:
                file_target = scope.targets[file_target_name]
                resolved_deps = {}
                for dep_key in ["deps", "public_deps", "data_deps", "configs", "public_configs"]:
                    deps = file_target.scope.get(dep_key) if file_target.scope.has(dep_key) else []
                    resolved_deps[dep_key] = []
                    for dep in deps:
                        resolved = resolveTarget(dep, file_scope)
                        if type(resolved) is list:
                            resolved_deps[dep_key].extend(resolved)
                        else:
                            resolved_deps[dep_key].append(resolved)
                print file_target.type, file_path, file_target_name
                if "webrtc/BUILD" in file_path:
                    target_cache[file_path][file_target_name] = buildTarget(file_target, resolved_deps, True)
                else:
                    target_cache[file_path][file_target_name] = buildTarget(file_target, resolved_deps)

    if target_name == "":
        return map(lambda (name, target): (file_path+":"+name, target), target_cache[file_path].iteritems())
    else:
        if target_name not in target_cache[file_path]:
            raise Exception("Resolved file: " + file_path + ", but did not find target: " + target_name)
        return (file_path+":"+target_name, target_cache[file_path][target_name])


resolveTarget(global_scope.get("current_toolchain"), global_scope)



try:
    webrtc_target = resolveTarget(sys.argv[1], global_scope)

    print
    print

    if type(webrtc_target) is not list:
        webrtc_target = [webrtc_target]

    for target in webrtc_target:
        print target[0] + " ("+target[1].type+")"
        #for source in target[1].properties["sources"]:
        #    print "\t" + source
        print "\tSources: " + str(len(target[1].properties["sources"]))

        for key in config_keys:
            if key in target[1].properties:
                print "\t"+key+": " + str(target[1].properties[key])

        print "\n\tLINK:"
        for link in target[1].properties["link"]:
            print "\t"+link
        print "\n\n\n"

    #for arg in global_scope.default_args:
    #    print arg, "=", global_scope.get(arg)


    build_file = open('CMakeLists.txt', 'w')
    build_file.write(
"""cmake_minimum_required(VERSION 2.8.3)
project(webrtc)

""")

    for target in webrtc_target:
        if target[1].type == "source_set" or target[1].type == "static_library":
            build_file.write("set(CMAKE_C_FLAGS \"${CMAKE_C_FLAGS} " + (" ".join(target[1].properties["cflags"] + target[1].properties["cflags_c"]))+"\")\n")
            build_file.write("set(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} " + (" ".join(target[1].properties["cflags"] + target[1].properties["cflags_cc"]))+"\")\n")

            build_file.write("\n\n")

            for define in target[1].properties["defines"]:
                build_file.write("add_definitions(-D" + define + ")\n")

            build_file.write("\n\n")

            build_file.write("include_directories(\n")
            for include_dir in target[1].properties["include_dirs"]:
                build_file.write("\t\"" + include_dir + "\"\n")
            build_file.write(")\n")

            build_file.write("\n\n")

            library_name = "my_lib"
            build_file.write("add_library("+library_name+"\n")
            for source in filter(lambda f: not f.endswith(".h"), target[1].properties["sources"]):
                build_file.write("\t\"" + source + "\"\n")
            build_file.write(")\n")
            build_file.write("target_link_libraries("+library_name+"\n")
            for lib in target[1].properties["libs"]:
                build_file.write("\t\"" + lib + "\"\n")
            build_file.write(")\n")



except:
    print_exc_plus()

