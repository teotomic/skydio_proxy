# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: vehicle.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='vehicle.proto',
  package='vehicle',
  syntax='proto3',
  serialized_pb=_b('\n\rvehicle.proto\x12\x07vehicle\"\x18\n\x08Vector3i\x12\x0c\n\x04\x64\x61ta\x18\x01 \x03(\x11\"\x18\n\x08Vector3f\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x02\"\x18\n\x08Vector3d\x12\x0c\n\x04\x64\x61ta\x18\x03 \x03(\x01\"\x1b\n\x0bQuaterniond\x12\x0c\n\x04xyzw\x18\x03 \x03(\x01\"\x1b\n\x0bQuaternionf\x12\x0c\n\x04xyzw\x18\x02 \x03(\x02\"f\n\x05Trans\x12\r\n\x05utime\x18\x01 \x01(\x03\x12)\n\x0borientation\x18\x02 \x01(\x0b\x32\x14.vehicle.Quaterniond\x12#\n\x08position\x18\x03 \x01(\x0b\x32\x11.vehicle.Vector3d\"\xca\x01\n\tPoseState\x12\r\n\x05utime\x18\x01 \x01(\x03\x12)\n\x0borientation\x18\x02 \x01(\x0b\x32\x14.vehicle.Quaterniond\x12#\n\x08position\x18\x03 \x01(\x0b\x32\x11.vehicle.Vector3d\x12\x32\n\x17\x61ngular_velocity_global\x18\x04 \x01(\x0b\x32\x11.vehicle.Vector3d\x12*\n\x0fvelocity_global\x18\x05 \x01(\x0b\x32\x11.vehicle.Vector3d\"`\n\x0bVoxelHeader\x12\x1f\n\x04\x64ims\x18\x01 \x01(\x0b\x32\x11.vehicle.Vector3i\x12\r\n\x05scale\x18\x02 \x01(\x02\x12!\n\x06origin\x18\x03 \x01(\x0b\x32\x11.vehicle.Vector3i\"c\n\x1eVoxelOccupancyRunLengthEncoded\x12\r\n\x05utime\x18\x01 \x01(\x12\x12$\n\x06header\x18\x02 \x01(\x0b\x32\x14.vehicle.VoxelHeader\x12\x0c\n\x04runs\x18\x03 \x03(\x11\"{\n\x12GimbalNavTransform\x12\r\n\x05utime\x18\x01 \x01(\x12\x12%\n\x1dhas_matching_navigation_image\x18\x02 \x01(\x08\x12/\n\x17nav_T_gimbal_camera_imu\x18\x03 \x01(\x0b\x32\x0e.vehicle.TransB*\n\x1a\x63om.skydio.pbtypes.vehicleB\x0cVehicleProtob\x06proto3')
)




_VECTOR3I = _descriptor.Descriptor(
  name='Vector3i',
  full_name='vehicle.Vector3i',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='vehicle.Vector3i.data', index=0,
      number=1, type=17, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=26,
  serialized_end=50,
)


_VECTOR3F = _descriptor.Descriptor(
  name='Vector3f',
  full_name='vehicle.Vector3f',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='vehicle.Vector3f.data', index=0,
      number=2, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=52,
  serialized_end=76,
)


_VECTOR3D = _descriptor.Descriptor(
  name='Vector3d',
  full_name='vehicle.Vector3d',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='vehicle.Vector3d.data', index=0,
      number=3, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=78,
  serialized_end=102,
)


_QUATERNIOND = _descriptor.Descriptor(
  name='Quaterniond',
  full_name='vehicle.Quaterniond',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='xyzw', full_name='vehicle.Quaterniond.xyzw', index=0,
      number=3, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=104,
  serialized_end=131,
)


_QUATERNIONF = _descriptor.Descriptor(
  name='Quaternionf',
  full_name='vehicle.Quaternionf',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='xyzw', full_name='vehicle.Quaternionf.xyzw', index=0,
      number=2, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=133,
  serialized_end=160,
)


_TRANS = _descriptor.Descriptor(
  name='Trans',
  full_name='vehicle.Trans',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='utime', full_name='vehicle.Trans.utime', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='orientation', full_name='vehicle.Trans.orientation', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='position', full_name='vehicle.Trans.position', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=162,
  serialized_end=264,
)


_POSESTATE = _descriptor.Descriptor(
  name='PoseState',
  full_name='vehicle.PoseState',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='utime', full_name='vehicle.PoseState.utime', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='orientation', full_name='vehicle.PoseState.orientation', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='position', full_name='vehicle.PoseState.position', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angular_velocity_global', full_name='vehicle.PoseState.angular_velocity_global', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='velocity_global', full_name='vehicle.PoseState.velocity_global', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=267,
  serialized_end=469,
)


_VOXELHEADER = _descriptor.Descriptor(
  name='VoxelHeader',
  full_name='vehicle.VoxelHeader',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='dims', full_name='vehicle.VoxelHeader.dims', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='scale', full_name='vehicle.VoxelHeader.scale', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='origin', full_name='vehicle.VoxelHeader.origin', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=471,
  serialized_end=567,
)


_VOXELOCCUPANCYRUNLENGTHENCODED = _descriptor.Descriptor(
  name='VoxelOccupancyRunLengthEncoded',
  full_name='vehicle.VoxelOccupancyRunLengthEncoded',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='utime', full_name='vehicle.VoxelOccupancyRunLengthEncoded.utime', index=0,
      number=1, type=18, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='header', full_name='vehicle.VoxelOccupancyRunLengthEncoded.header', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='runs', full_name='vehicle.VoxelOccupancyRunLengthEncoded.runs', index=2,
      number=3, type=17, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=569,
  serialized_end=668,
)


_GIMBALNAVTRANSFORM = _descriptor.Descriptor(
  name='GimbalNavTransform',
  full_name='vehicle.GimbalNavTransform',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='utime', full_name='vehicle.GimbalNavTransform.utime', index=0,
      number=1, type=18, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='has_matching_navigation_image', full_name='vehicle.GimbalNavTransform.has_matching_navigation_image', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='nav_T_gimbal_camera_imu', full_name='vehicle.GimbalNavTransform.nav_T_gimbal_camera_imu', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=670,
  serialized_end=793,
)

_TRANS.fields_by_name['orientation'].message_type = _QUATERNIOND
_TRANS.fields_by_name['position'].message_type = _VECTOR3D
_POSESTATE.fields_by_name['orientation'].message_type = _QUATERNIOND
_POSESTATE.fields_by_name['position'].message_type = _VECTOR3D
_POSESTATE.fields_by_name['angular_velocity_global'].message_type = _VECTOR3D
_POSESTATE.fields_by_name['velocity_global'].message_type = _VECTOR3D
_VOXELHEADER.fields_by_name['dims'].message_type = _VECTOR3I
_VOXELHEADER.fields_by_name['origin'].message_type = _VECTOR3I
_VOXELOCCUPANCYRUNLENGTHENCODED.fields_by_name['header'].message_type = _VOXELHEADER
_GIMBALNAVTRANSFORM.fields_by_name['nav_T_gimbal_camera_imu'].message_type = _TRANS
DESCRIPTOR.message_types_by_name['Vector3i'] = _VECTOR3I
DESCRIPTOR.message_types_by_name['Vector3f'] = _VECTOR3F
DESCRIPTOR.message_types_by_name['Vector3d'] = _VECTOR3D
DESCRIPTOR.message_types_by_name['Quaterniond'] = _QUATERNIOND
DESCRIPTOR.message_types_by_name['Quaternionf'] = _QUATERNIONF
DESCRIPTOR.message_types_by_name['Trans'] = _TRANS
DESCRIPTOR.message_types_by_name['PoseState'] = _POSESTATE
DESCRIPTOR.message_types_by_name['VoxelHeader'] = _VOXELHEADER
DESCRIPTOR.message_types_by_name['VoxelOccupancyRunLengthEncoded'] = _VOXELOCCUPANCYRUNLENGTHENCODED
DESCRIPTOR.message_types_by_name['GimbalNavTransform'] = _GIMBALNAVTRANSFORM
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Vector3i = _reflection.GeneratedProtocolMessageType('Vector3i', (_message.Message,), dict(
  DESCRIPTOR = _VECTOR3I,
  __module__ = 'vehicle_pb2'
  # @@protoc_insertion_point(class_scope:vehicle.Vector3i)
  ))
_sym_db.RegisterMessage(Vector3i)

Vector3f = _reflection.GeneratedProtocolMessageType('Vector3f', (_message.Message,), dict(
  DESCRIPTOR = _VECTOR3F,
  __module__ = 'vehicle_pb2'
  # @@protoc_insertion_point(class_scope:vehicle.Vector3f)
  ))
_sym_db.RegisterMessage(Vector3f)

Vector3d = _reflection.GeneratedProtocolMessageType('Vector3d', (_message.Message,), dict(
  DESCRIPTOR = _VECTOR3D,
  __module__ = 'vehicle_pb2'
  # @@protoc_insertion_point(class_scope:vehicle.Vector3d)
  ))
_sym_db.RegisterMessage(Vector3d)

Quaterniond = _reflection.GeneratedProtocolMessageType('Quaterniond', (_message.Message,), dict(
  DESCRIPTOR = _QUATERNIOND,
  __module__ = 'vehicle_pb2'
  # @@protoc_insertion_point(class_scope:vehicle.Quaterniond)
  ))
_sym_db.RegisterMessage(Quaterniond)

Quaternionf = _reflection.GeneratedProtocolMessageType('Quaternionf', (_message.Message,), dict(
  DESCRIPTOR = _QUATERNIONF,
  __module__ = 'vehicle_pb2'
  # @@protoc_insertion_point(class_scope:vehicle.Quaternionf)
  ))
_sym_db.RegisterMessage(Quaternionf)

Trans = _reflection.GeneratedProtocolMessageType('Trans', (_message.Message,), dict(
  DESCRIPTOR = _TRANS,
  __module__ = 'vehicle_pb2'
  # @@protoc_insertion_point(class_scope:vehicle.Trans)
  ))
_sym_db.RegisterMessage(Trans)

PoseState = _reflection.GeneratedProtocolMessageType('PoseState', (_message.Message,), dict(
  DESCRIPTOR = _POSESTATE,
  __module__ = 'vehicle_pb2'
  # @@protoc_insertion_point(class_scope:vehicle.PoseState)
  ))
_sym_db.RegisterMessage(PoseState)

VoxelHeader = _reflection.GeneratedProtocolMessageType('VoxelHeader', (_message.Message,), dict(
  DESCRIPTOR = _VOXELHEADER,
  __module__ = 'vehicle_pb2'
  # @@protoc_insertion_point(class_scope:vehicle.VoxelHeader)
  ))
_sym_db.RegisterMessage(VoxelHeader)

VoxelOccupancyRunLengthEncoded = _reflection.GeneratedProtocolMessageType('VoxelOccupancyRunLengthEncoded', (_message.Message,), dict(
  DESCRIPTOR = _VOXELOCCUPANCYRUNLENGTHENCODED,
  __module__ = 'vehicle_pb2'
  # @@protoc_insertion_point(class_scope:vehicle.VoxelOccupancyRunLengthEncoded)
  ))
_sym_db.RegisterMessage(VoxelOccupancyRunLengthEncoded)

GimbalNavTransform = _reflection.GeneratedProtocolMessageType('GimbalNavTransform', (_message.Message,), dict(
  DESCRIPTOR = _GIMBALNAVTRANSFORM,
  __module__ = 'vehicle_pb2'
  # @@protoc_insertion_point(class_scope:vehicle.GimbalNavTransform)
  ))
_sym_db.RegisterMessage(GimbalNavTransform)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\032com.skydio.pbtypes.vehicleB\014VehicleProto'))
# @@protoc_insertion_point(module_scope)