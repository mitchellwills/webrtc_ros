var params = {};
if(window.location.search.length > 1) {
    var query_string = decodeURIComponent(window.location.search.substring(1));
    var parameter_key_values = query_string.split("&");
    for(var i in parameter_key_values) {
	if(parameter_key_values.hasOwnProperty(i)) {
	    var key_value = parameter_key_values[i];
	    if(key_value.length > 0) {
		var key_value_arr = key_value.split("=", 2);
		if(key_value_arr.length === 2) {
		    params[key_value_arr[0]] = key_value_arr[1];
		}
		else {
		    params[key_value_arr[0]] = "";
		}
	    }
	}
    }
}

var connection = WebrtcRos.createConnection();
connection.onConfigurationNeeded = function() {
    connection.addDataChannel({"target": "ros_subscriber:/diagnostics"}).then(function(event){
	console.log("Got channel: ", event);
	var message_definition = null;
	var previous_fragments = null;

	var handle_packet = function(packet) {
	    var packet_type = packet.getUint8(0);
	    console.group("start", packet_type, packet.byteLength);
	    var content = new DataView(packet.buffer, 1 + packet.byteOffset);

	    if(packet_type === 0) {
		var definition = JSON.parse(new TextDecoder("ascii").decode(content));
		message_definition = RosSerialization.parseMessageDefinition(definition);
	    }
	    else if(packet_type === 1) {
		var message_data_stream = RosSerialization.createStream(content);
		var message = message_definition.deserialize(message_data_stream);
		document.getElementById("data-box").innerHTML = JSON.stringify(message, null, '\t');
	    }
	    else if(packet_type === 2 || packet_type === 3) {
		var previous_length = previous_fragments ? previous_fragments.byteLength : 0;
		var fragments = new Uint8Array(previous_length + content.byteLength);
		if(previous_fragments) {
		    fragments.set(previous_fragments);
		}
		var content_bytes = new Uint8Array(content.buffer, content.byteOffset, content.byteLength);
		fragments.set(content_bytes, previous_length);
		if(packet_type === 2) {
		    previous_fragments = fragments;
		}
		else {
		    previous_fragments = null;
		    handle_packet(new DataView(fragments.buffer));
		}
	    }
	    else if(packet_type === 4) {
		var decompressed = pako.inflate(new Uint8Array(content.buffer, content.byteOffset, content.byteLength));
		handle_packet(new DataView(decompressed.buffer, decompressed.byteOffset, decompressed.byteLength));
	    }
	    else {
		console.warn("Unknown packet type", packet_type);
	    }
	    console.groupEnd();
	};
	event.channel.onmessage = function(msg) {
	    var packet = new DataView(msg.data);
	    handle_packet(packet);
	};
    });
    connection.sendConfigure();
};
connection.connect();
