# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: robot_state.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='robot_state.proto',
  package='enac',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x11robot_state.proto\x12\x04\x65nac\" \n\rno_args_func_\x12\x0f\n\x07nothing\x18\x01 \x01(\x02\"/\n\x08Position\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\r\n\x05theta\x18\x03 \x01(\x02\"\xa6\x02\n\x08SetState\x12\x16\n\x0eplate_position\x18\x01 \x01(\x05\x12\x14\n\x0cplate_number\x18\x02 \x01(\x05\x12\x31\n\x0b\x63\x65rise_drop\x18\x03 \x01(\x0e\x32\x1c.enac.SetState.TobogganState\x12,\n\nclaw_state\x18\x04 \x01(\x0e\x32\x18.enac.SetState.ClawState\"F\n\tClawState\x12\n\n\x06NONE_G\x10\x00\x12\r\n\tCLAW_OPEN\x10o\x12\x0f\n\x0b\x43LAW_CLOSED\x10\x63\x12\r\n\tCLAW_GRAB\x10g\"C\n\rTobogganState\x12\n\n\x06NONE_T\x10\x00\x12\x11\n\rTOBOGGAN_OPEN\x10s\x12\x13\n\x0fTOBOGGAN_CLOSED\x10r\"/\n\x05Speed\x12\n\n\x02vx\x18\x01 \x01(\x02\x12\n\n\x02vy\x18\x02 \x01(\x02\x12\x0e\n\x06vtheta\x18\x03 \x01(\x02\"&\n\x05Match\x12\x0e\n\x06status\x18\x01 \x01(\t\x12\r\n\x05score\x18\x02 \x01(\x05\"\x18\n\x06\x41\x63tion\x12\x0e\n\x06\x61\x63tion\x18\x01 \x01(\x05\"\"\n\x04Side\x12\x1a\n\x05\x63olor\x18\x01 \x01(\x0e\x32\x0b.enac.Color*\x1c\n\x05\x43olor\x12\x08\n\x04\x42LUE\x10\x00\x12\t\n\x05GREEN\x10\x01\x62\x06proto3'
)

_COLOR = _descriptor.EnumDescriptor(
  name='Color',
  full_name='enac.Color',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='BLUE', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='GREEN', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=558,
  serialized_end=586,
)
_sym_db.RegisterEnumDescriptor(_COLOR)

Color = enum_type_wrapper.EnumTypeWrapper(_COLOR)
BLUE = 0
GREEN = 1


_SETSTATE_CLAWSTATE = _descriptor.EnumDescriptor(
  name='ClawState',
  full_name='enac.SetState.ClawState',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE_G', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CLAW_OPEN', index=1, number=111,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CLAW_CLOSED', index=2, number=99,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CLAW_GRAB', index=3, number=103,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=266,
  serialized_end=336,
)
_sym_db.RegisterEnumDescriptor(_SETSTATE_CLAWSTATE)

_SETSTATE_TOBOGGANSTATE = _descriptor.EnumDescriptor(
  name='TobogganState',
  full_name='enac.SetState.TobogganState',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE_T', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='TOBOGGAN_OPEN', index=1, number=115,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='TOBOGGAN_CLOSED', index=2, number=114,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=338,
  serialized_end=405,
)
_sym_db.RegisterEnumDescriptor(_SETSTATE_TOBOGGANSTATE)


_NO_ARGS_FUNC_ = _descriptor.Descriptor(
  name='no_args_func_',
  full_name='enac.no_args_func_',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='nothing', full_name='enac.no_args_func_.nothing', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=27,
  serialized_end=59,
)


_POSITION = _descriptor.Descriptor(
  name='Position',
  full_name='enac.Position',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='enac.Position.x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='enac.Position.y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='theta', full_name='enac.Position.theta', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=61,
  serialized_end=108,
)


_SETSTATE = _descriptor.Descriptor(
  name='SetState',
  full_name='enac.SetState',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='plate_position', full_name='enac.SetState.plate_position', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='plate_number', full_name='enac.SetState.plate_number', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='cerise_drop', full_name='enac.SetState.cerise_drop', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='claw_state', full_name='enac.SetState.claw_state', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _SETSTATE_CLAWSTATE,
    _SETSTATE_TOBOGGANSTATE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=111,
  serialized_end=405,
)


_SPEED = _descriptor.Descriptor(
  name='Speed',
  full_name='enac.Speed',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='vx', full_name='enac.Speed.vx', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vy', full_name='enac.Speed.vy', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vtheta', full_name='enac.Speed.vtheta', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=407,
  serialized_end=454,
)


_MATCH = _descriptor.Descriptor(
  name='Match',
  full_name='enac.Match',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='enac.Match.status', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='score', full_name='enac.Match.score', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=456,
  serialized_end=494,
)


_ACTION = _descriptor.Descriptor(
  name='Action',
  full_name='enac.Action',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='action', full_name='enac.Action.action', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=496,
  serialized_end=520,
)


_SIDE = _descriptor.Descriptor(
  name='Side',
  full_name='enac.Side',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='color', full_name='enac.Side.color', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=522,
  serialized_end=556,
)

_SETSTATE.fields_by_name['cerise_drop'].enum_type = _SETSTATE_TOBOGGANSTATE
_SETSTATE.fields_by_name['claw_state'].enum_type = _SETSTATE_CLAWSTATE
_SETSTATE_CLAWSTATE.containing_type = _SETSTATE
_SETSTATE_TOBOGGANSTATE.containing_type = _SETSTATE
_SIDE.fields_by_name['color'].enum_type = _COLOR
DESCRIPTOR.message_types_by_name['no_args_func_'] = _NO_ARGS_FUNC_
DESCRIPTOR.message_types_by_name['Position'] = _POSITION
DESCRIPTOR.message_types_by_name['SetState'] = _SETSTATE
DESCRIPTOR.message_types_by_name['Speed'] = _SPEED
DESCRIPTOR.message_types_by_name['Match'] = _MATCH
DESCRIPTOR.message_types_by_name['Action'] = _ACTION
DESCRIPTOR.message_types_by_name['Side'] = _SIDE
DESCRIPTOR.enum_types_by_name['Color'] = _COLOR
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

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

Side = _reflection.GeneratedProtocolMessageType('Side', (_message.Message,), {
  'DESCRIPTOR' : _SIDE,
  '__module__' : 'robot_state_pb2'
  # @@protoc_insertion_point(class_scope:enac.Side)
  })
_sym_db.RegisterMessage(Side)


# @@protoc_insertion_point(module_scope)
