# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: robot_state.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x11robot_state.proto\x12\x04\x65nac\" \n\rno_args_func_\x12\x0f\n\x07nothing\x18\x01 \x01(\x05\"/\n\x08Position\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\r\n\x05theta\x18\x03 \x01(\x02\"a\n\x08SetState\x12\x16\n\x0eplate_position\x18\x01 \x01(\x05\x12\x14\n\x0cplate_number\x18\x02 \x01(\x05\x12\x13\n\x0b\x63\x65rise_drop\x18\x03 \x01(\x08\x12\x12\n\nclaw_state\x18\x04 \x01(\x05\"/\n\x05Speed\x12\n\n\x02vx\x18\x01 \x01(\x02\x12\n\n\x02vy\x18\x02 \x01(\x02\x12\x0e\n\x06vtheta\x18\x03 \x01(\x02\"&\n\x05Match\x12\x0e\n\x06status\x18\x01 \x01(\t\x12\r\n\x05score\x18\x02 \x01(\x05\"\x18\n\x06\x41\x63tion\x12\x0e\n\x06\x61\x63tion\x18\x01 \x01(\x05\x62\x06proto3')



_NO_ARGS_FUNC_ = DESCRIPTOR.message_types_by_name['no_args_func_']
_POSITION = DESCRIPTOR.message_types_by_name['Position']
_SETSTATE = DESCRIPTOR.message_types_by_name['SetState']
_SPEED = DESCRIPTOR.message_types_by_name['Speed']
_MATCH = DESCRIPTOR.message_types_by_name['Match']
_ACTION = DESCRIPTOR.message_types_by_name['Action']
no_args_func_ = _reflection.GeneratedProtocolMessageType('no_args_func_', (_message.Message,), {
  'DESCRIPTOR' : _NO_ARGS_FUNC_,
  '__module__' : 'robot_state_pb2'
  # @@protoc_insertion_point(class_scope:enac.no_args_func_)
  })
_sym_db.RegisterMessage(no_args_func_)

Position = _reflection.GeneratedProtocolMessageType('Position', (_message.Message,), {
  'DESCRIPTOR' : _POSITION,
  '__module__' : 'robot_state_pb2'
  # @@protoc_insertion_point(class_scope:enac.Position)
  })
_sym_db.RegisterMessage(Position)

SetState = _reflection.GeneratedProtocolMessageType('SetState', (_message.Message,), {
  'DESCRIPTOR' : _SETSTATE,
  '__module__' : 'robot_state_pb2'
  # @@protoc_insertion_point(class_scope:enac.SetState)
  })
_sym_db.RegisterMessage(SetState)

Speed = _reflection.GeneratedProtocolMessageType('Speed', (_message.Message,), {
  'DESCRIPTOR' : _SPEED,
  '__module__' : 'robot_state_pb2'
  # @@protoc_insertion_point(class_scope:enac.Speed)
  })
_sym_db.RegisterMessage(Speed)

Match = _reflection.GeneratedProtocolMessageType('Match', (_message.Message,), {
  'DESCRIPTOR' : _MATCH,
  '__module__' : 'robot_state_pb2'
  # @@protoc_insertion_point(class_scope:enac.Match)
  })
_sym_db.RegisterMessage(Match)

Action = _reflection.GeneratedProtocolMessageType('Action', (_message.Message,), {
  'DESCRIPTOR' : _ACTION,
  '__module__' : 'robot_state_pb2'
  # @@protoc_insertion_point(class_scope:enac.Action)
  })
_sym_db.RegisterMessage(Action)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _NO_ARGS_FUNC_._serialized_start=27
  _NO_ARGS_FUNC_._serialized_end=59
  _POSITION._serialized_start=61
  _POSITION._serialized_end=108
  _SETSTATE._serialized_start=110
  _SETSTATE._serialized_end=207
  _SPEED._serialized_start=209
  _SPEED._serialized_end=256
  _MATCH._serialized_start=258
  _MATCH._serialized_end=296
  _ACTION._serialized_start=298
  _ACTION._serialized_end=322
# @@protoc_insertion_point(module_scope)
