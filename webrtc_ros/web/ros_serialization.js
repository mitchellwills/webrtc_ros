window.RosSerialization = (function() {
    function Stream(dataView) {
	this.dataView = dataView;
	this.pos = 0;
    }
    Stream.prototype.advance = function(amount) {
	var pos = this.pos;
	this.pos += amount;
	return pos;
    };
    Stream.prototype.viewAdvance = function(size) {
	var view = new DataView(this.dataView.buffer, this.dataView.byteOffset + this.pos, size);
	this.pos += size;
	return view;
    };

    function PrimitiveDataType(type, dataViewAccessor, size) {
	this.type = type;
	this.md5 = "";
	this.dataViewAccessor = dataViewAccessor;
	this.size = size;
    }
    PrimitiveDataType.prototype.deserialize = function(stream){
	return stream.dataView[this.dataViewAccessor](stream.advance(this.size), true);
    };

    function BoolDataType() {
	this.type = "bool";
	this.md5 = "";
    }
    BoolDataType.prototype.deserialize = function(stream){
	var asInt = builtin_types["uint8"].deserialize(stream);
	return asInt != 0;
    };

    function StringDataType() {
	this.type = "string";
	this.md5 = "";
    }
    StringDataType.prototype.deserialize = function(stream){
	var length = builtin_types["uint32"].deserialize(stream);
	return new TextDecoder("utf-8").decode(stream.viewAdvance(length));
    };

    function TimeDataType() {
	this.type = "time";
	this.md5 = "";
    }
    TimeDataType.prototype.deserialize = function(stream){
	var sec = builtin_types["uint32"].deserialize(stream);
	var nsec = builtin_types["uint32"].deserialize(stream);
	return new Date(nsec / 1000 + sec * 1000);
    };

    function DurationDataType() {
	this.type = "time";
	this.md5 = "";
    }
    DurationDataType.prototype.deserialize = function(stream){
	var sec = builtin_types["uint32"].deserialize(stream);
	var nsec = builtin_types["uint32"].deserialize(stream);
	return nsec / 1000 + sec * 1000;
    };

    function ArrayDataType(value_type) {
	this.value_type = value_type
	this.type = value_type.type + "[]";
	this.md5 = "";
    }
    ArrayDataType.prototype.deserialize = function(stream){
	var length = builtin_types["uint32"].deserialize(stream);
	var data = [];
	for(var i = 0; i < length; ++i) {
	    data.push(this.value_type.deserialize(stream));
	}
	return data;
    };

    function Primitive(name, js_type, byte_size) {
	this.name = name;
	this.js_type = js_type;
	this.byte_size = byte_size;
    }
    var primitive_types = [
	new Primitive("byte", "Int8", 1),
	new Primitive("char", "Uint8", 1)
    ];
    // TODO handle 64 bit integers
    [1, 2, 4, /* 8 */].map(function(byte_size) {
	var bit_size = byte_size * 8;
	primitive_types.push(new Primitive("int"+bit_size, "Int"+bit_size, byte_size));
	primitive_types.push(new Primitive("uint"+bit_size, "Uint"+bit_size, byte_size));
    });
    [4, 8].map(function(byte_size) {
	var bit_size = byte_size * 8;
	primitive_types.push(new Primitive("float"+bit_size, "Float"+bit_size, byte_size));
    });
    console.log(primitive_types);

    var builtin_types = {
	"string": new StringDataType(),
	"time": new TimeDataType(),
	"duration": new DurationDataType()
    };
    primitive_types.forEach(function(type) {
	builtin_types[type.name] = new PrimitiveDataType(type.name, "get"+type.js_type, type.byte_size);
    });

    function RosMessageType(type, md5, fields) {
	this.type = type;
	this.md5 = md5;
	this.fields = fields;
    }
    RosMessageType.prototype.deserialize = function(stream){
	var obj = {};
	for(var i in this.fields) {
	    var field = this.fields[i];
	    obj[field.name] = field.type.deserialize(stream);
	}
	return obj;
    };


    var RosSerialization= {
	createStream: function(dataView) {
	    return new Stream(dataView);
	},
	parseMessageDefinition: function(definition) {
	    var info_map = {};
	    for(var i in definition.infos) {
		var info = definition.infos[i];
		info_map[info.type] = info;
	    }
	    var resolved = {};
	    console.log(definition);

	    function resolve(type) {
		if(type in builtin_types) {
		    return builtin_types[type];
		}
		else if(type in resolved) {
		    return resolved[type];
		}
		else {
		    var info = info_map[type];
		    console.log(type, info);
		    var fields = [];
		    for(var i in info.fields) {
			var field = info.fields[i];
			if(field.type.endsWith("[]")) {
			    var value_type = resolve(field.type.slice(0, -2));
			    fields.push({"type": new ArrayDataType(value_type), "name": field.name});
			}
			else {
			    fields.push({"type": resolve(field.type), "name": field.name});
			}
		    }
		    return resolved[type] = new RosMessageType(info.type, info.md5sum, fields);
		}
	    }
	    return resolve(definition.type);
	}
    };
    return RosSerialization;
})();
