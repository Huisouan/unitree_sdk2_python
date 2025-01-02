"""
  Generated by Eclipse Cyclone DDS idlc Python Backend
  Cyclone DDS IDL version: v0.11.0
  Module: geometry_msgs.msg.dds_
  IDL file: Point32_.idl

"""

from enum import auto
from typing import TYPE_CHECKING, Optional
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

# root module import for resolving types
# import geometry_msgs


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Point32_(idl.IdlStruct, typename="geometry_msgs.msg.dds_.Point32_"):
    x: types.float32
    y: types.float32
    z: types.float32


