window.WebrtcRos = (function() {
    var newId = function() {
	// Just need unique identifiers for streams
	return "webrtc_ros-"+Math.floor(Math.random()*1000000000).toString();
    };
    var WebrtcRosConnection = function(signalingServerPath) {
	this.signalingServerPath = signalingServerPath || "ws://"+window.location.host+"/webrtc";
	this.onConfigurationNeeded = undefined;
	this.signalingChannel = null;
	this.peerConnection = null;
	this.peerConnectionOptions = {
	    optional: [
		{DtlsSrtpKeyAgreement: true}
	    ]
	};

	this.lastConfigureActionPromise = Promise.resolve([]);

	this.addStreamCallbacks = {};
	this.removeStreamCallbacks = {};
	this.addDataChannelCallbacks = {}
    };

    WebrtcRosConnection.prototype.connect = function(){
	var self = this;
	this.close();
	this.signalingChannel = new WebSocket(this.signalingServerPath);
	this.signalingChannel.onmessage = function(e){ self.onSignalingMessage(e); };
	this.signalingChannel.onopen = function() {
	    console.log("WebRTC signaling open");
	    if(self.onConfigurationNeeded) {
		self.onConfigurationNeeded();
	    }
	};
	this.signalingChannel.onerror = function() {
	    console.log("WebRTC signaling error");
	};
	this.signalingChannel.onclose = function() {
	    console.log("WebRTC signaling close");
	};
	this.peerConnection = new RTCPeerConnection(null, this.peerConnectionOptions);
	this.peerConnection.onicecandidate = function(event) {
            if (event.candidate) {
                var candidate = {
                    sdp_mline_index: event.candidate.sdpMLineIndex,
                    sdp_mid: event.candidate.sdpMid,
                    candidate: event.candidate.candidate,
                    type: "ice_candidate"
                };
                self.signalingChannel.send(JSON.stringify(candidate));
            }
        };
	this.peerConnection.onaddstream = function(event) {
	    var stream = event.stream;
	    var callbackData = self.addStreamCallbacks[stream.id];
	    if(callbackData) {
		callbackData.resolve({
		    "stream": stream,
		    "remove": new Promise(function(resolve, reject) {
			self.removeStreamCallbacks[stream.id] = {
			    "resolve": resolve,
			    "reject": reject
			};
		    })
		});
	    }
	};
	this.peerConnection.onremovestream = function(event) {
	    var stream = event.stream;
	    var callbackData = self.removeStreamCallbacks[stream.id];
	    if(callbackData) {
		callbackData.resolve({
		    "stream": stream
		});
	    }
	};
	this.peerConnection.ondatachannel = function(event) {
	    console.log(event);
	    var channel = event.channel;
	    var callbackData = self.addDataChannelCallbacks[channel.label];
	    if(callbackData) {
		callbackData.resolve({
		    "channel": channel
		});
	    }
	};
    };
    WebrtcRosConnection.prototype.close = function(){
	if(this.peerConnection) {
	    this.peerConnection.close();
	    this.peerConnection = null;
	}
	if(this.signalingChannel) {
	    this.signalingChannel.close();
	    this.signalingChannel = null;
	}
    };
    WebrtcRosConnection.prototype.onSignalingMessage = function(e){
	var self = this;
	var dataJson = JSON.parse(e.data);
	console.log("WebRTC ROS Got Message: ", dataJson);
	if (dataJson.type === "offer") {
	    this.peerConnection.setRemoteDescription(new RTCSessionDescription(dataJson),
						     function(){
							 self.sendAnswer();
						     },
						     function(event) {
							 console.error("onRemoteSdpError", event);
						     });
	}
	else if(dataJson.type === "ice_candidate") {
	    var candidate = new RTCIceCandidate({sdpMLineIndex: dataJson.sdp_mline_index, candidate: dataJson.candidate});
	    this.peerConnection.addIceCandidate(candidate);
	}
	else {
	    console.warn("Unknown message type: ", dataJson.type);
	}
    };

    WebrtcRosConnection.prototype.sendAnswer = function() {
	var self = this;
	var mediaConstraints = {"optional": [
	    {"OfferToReceiveVideo": true},
	    {"OfferToReceiveVideo": true}
	]};
	this.peerConnection.createAnswer(function(sessionDescription) {
            self.peerConnection.setLocalDescription(sessionDescription);
            var data = JSON.stringify(sessionDescription);
            self.signalingChannel.send(data);
	}, function(error) {
            console.warn("Create answer error:", error);
	}, mediaConstraints);
    };

    WebrtcRosConnection.prototype.addRemoteStream = function(config) {
	var stream_id = newId();
	var self = this;

	this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(function(actions) {
	    actions.push({"type":"add_stream", "id": stream_id});
	    if(config.video) {
		actions.push({
		    "type":"add_video_track",
		    "stream_id": stream_id,
		    "id": stream_id + config.video.id,
		    "src": config.video.src
		});
	    }
	    if(config.audio) {
		actions.push({
		    "type":"add_audio_track",
		    "stream_id": stream_id,
		    "id": stream_id + config.audio.id,
		    "src": config.audio.src
		});
	    }
	    return actions;
	});

	return new Promise(function(resolve, reject) {
	    self.addStreamCallbacks[stream_id] = {
		"resolve": resolve,
		"reject": reject
	    };
	});
    };
    WebrtcRosConnection.prototype.removeRemoteStream = function(stream) {
	var self = this;
	this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(function(actions) {
	    actions.push({"type":"remove_stream", "id": stream.id});
	    return actions;
	});
    };
    WebrtcRosConnection.prototype.addDataChannel = function(config) {
	var self = this;
	var channel_id = newId();
	this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(function(actions) {
	    actions.push({"type":"add_data_channel", "id": channel_id, "target": config.target});
	    return actions;
	});
	return new Promise(function(resolve, reject) {
	    self.addDataChannelCallbacks[channel_id] = {
		"resolve": resolve,
		"reject": reject
	    };
	});
    };
    WebrtcRosConnection.prototype.addLocalStream = function(user_media_config, local_stream_config) {
	var self = this;
	return new Promise(function(resolve, reject) {
	    self.lastConfigureActionPromise = self.lastConfigureActionPromise.then(
		function(actions){
		    return navigator.mediaDevices.getUserMedia(user_media_config).then(function(stream){
			actions.push({"type":"expect_stream", "id": stream.id});
			if(local_stream_config.video) {
			    actions.push({
				"type":"expect_video_track",
				"stream_id": stream.id,
				"id": stream.getVideoTracks()[0].id,
				"dest":local_stream_config.video.dest
			    });
			}
			self.peerConnection.addStream(stream);
			resolve({
			    "stream": stream,
			    "remove": new Promise(function(resolve, reject) {
				self.removeStreamCallbacks[stream.id] = {
				    "resolve": resolve,
				    "reject": reject
				};
			    })
			});
			return actions;
		    });
		}
	    );

	});
    };
    WebrtcRosConnection.prototype.removeLocalStream = function(stream) {
	var self = this;
	this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(function(actions) {
	    console.log("Removing stream");
	    self.peerConnection.removeStream(stream);

	    var callbackData = self.removeStreamCallbacks[stream.id];
	    if(callbackData) {
		callbackData.resolve({
		    "stream": stream
		});
	    }
	    return actions;
	});
    };
    WebrtcRosConnection.prototype.sendConfigure = function() {
	var self = this;

	var currentLastConfigureActionPromise = this.lastConfigureActionPromise;
	this.lastConfigureActionPromise = Promise.resolve([]);

	currentLastConfigureActionPromise.then(function(actions) {
	    var configMessage = {"type": "configure", "actions": actions};
	    self.signalingChannel.send(JSON.stringify(configMessage));
	    console.log("WebRTC ROS Configure: ", actions);
	});
    };

    var WebrtcRos = {
	createConnection: function(signalingServerPath) {
	    return new WebrtcRosConnection(signalingServerPath);
	}
    };
    return WebrtcRos;
})();
