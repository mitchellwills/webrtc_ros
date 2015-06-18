#!/usr/bin/env python

import rospy
from webrtc_ros.msg import *
from webrtc_ros.srv import *

builtins = ['bool', 'char', 'byte','int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64', 'string', 'time', 'duration']

def get_message(type):
    (package, message_name) = type.split("/")
    package_msgs = __import__(package + ".msg").msg
    return getattr(package_msgs, message_name)

def get_message_info(type):
    if type in builtins:
        return None
    msg = get_message(type)
    info = MessageInfo(type=msg._type, md5sum=msg._md5sum, definition=msg._full_text)
    for i in xrange(len(msg.__slots__)):
        info.fields.append(MessageField(name=msg.__slots__[i], type=msg._slot_types[i]))
    return info

def get_message_info_rec(type):
    info = get_message_info(type)
    if info is None:
        return {}
    infos = {type: info}
    for field in info.fields:
        if field.type.endswith("[]"):
            infos.update(get_message_info_rec(field.type[:-2]))
        else:
            infos.update(get_message_info_rec(field.type))
    return infos

def fufill_message_info_request(req):
    return GetMessageInfoResponse(type=req.type, infos=get_message_info_rec(req.type).values())

if __name__=="__main__":
    rospy.init_node("message_info_node")
    service = rospy.Service("get_message_info", GetMessageInfo, fufill_message_info_request)

    rospy.spin();
