# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: lidar_data.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='lidar_data.proto',
  package='enac',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=b'\n\x10lidar_data.proto\x12\x04\x65nac\"S\n\x05Lidar\x12\x0e\n\x06nb_pts\x18\x01 \x01(\x05\x12\x17\n\x0f\x61ngle_increment\x18\x02 \x01(\x02\x12\x0e\n\x06\x61ngles\x18\x03 \x03(\x02\x12\x11\n\tdistances\x18\x04 \x03(\x02\"C\n\tProximity\x12\x18\n\x10\x63losest_distance\x18\x01 \x01(\x02\x12\x1c\n\x06\x61\x63tion\x18\x02 \x01(\x0e\x32\x0c.enac.Action*\'\n\x06\x41\x63tion\x12\x06\n\x02OK\x10\x00\x12\x0b\n\x07WARNING\x10\x01\x12\x08\n\x04STOP\x10\x02\x62\x06proto3'
)

_ACTION = _descriptor.EnumDescriptor(
  name='Action',
  full_name='enac.Action',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OK', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WARNING', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STOP', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=180,
  serialized_end=219,
)
_sym_db.RegisterEnumDescriptor(_ACTION)

Action = enum_type_wrapper.EnumTypeWrapper(_ACTION)
OK = 0
WARNING = 1
STOP = 2



_LIDAR = _descriptor.Descriptor(
  name='Lidar',
  full_name='enac.Lidar',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='nb_pts', full_name='enac.Lidar.nb_pts', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angle_increment', full_name='enac.Lidar.angle_increment', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angles', full_name='enac.Lidar.angles', index=2,
      number=3, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distances', full_name='enac.Lidar.distances', index=3,
      number=4, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
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
  serialized_start=26,
  serialized_end=109,
)


_PROXIMITY = _descriptor.Descriptor(
  name='Proximity',
  full_name='enac.Proximity',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='closest_distance', full_name='enac.Proximity.closest_distance', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='action', full_name='enac.Proximity.action', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
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
  serialized_start=111,
  serialized_end=178,
)

_PROXIMITY.fields_by_name['action'].enum_type = _ACTION
DESCRIPTOR.message_types_by_name['Lidar'] = _LIDAR
DESCRIPTOR.message_types_by_name['Proximity'] = _PROXIMITY
DESCRIPTOR.enum_types_by_name['Action'] = _ACTION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Lidar = _reflection.GeneratedProtocolMessageType('Lidar', (_message.Message,), {
  'DESCRIPTOR' : _LIDAR,
  '__module__' : 'lidar_data_pb2'
  # @@protoc_insertion_point(class_scope:enac.Lidar)
  })
_sym_db.RegisterMessage(Lidar)

Proximity = _reflection.GeneratedProtocolMessageType('Proximity', (_message.Message,), {
  'DESCRIPTOR' : _PROXIMITY,
  '__module__' : 'lidar_data_pb2'
  # @@protoc_insertion_point(class_scope:enac.Proximity)
  })
_sym_db.RegisterMessage(Proximity)


# @@protoc_insertion_point(module_scope)