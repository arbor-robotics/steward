from google.protobuf import timestamp_pb2 as _timestamp_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import (
    ClassVar as _ClassVar,
    Mapping as _Mapping,
    Optional as _Optional,
    Union as _Union,
)

DESCRIPTOR: _descriptor.FileDescriptor

class Header(_message.Message):
    __slots__ = ("stamp", "frame_id")
    STAMP_FIELD_NUMBER: _ClassVar[int]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    stamp: _timestamp_pb2.Timestamp
    frame_id: str
    def __init__(
        self,
        stamp: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ...,
        frame_id: _Optional[str] = ...,
    ) -> None: ...
