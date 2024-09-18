python3 -m grpc_tools.protoc --proto_path=src/msg/protobuf_msgs/protobuf_msgs/protos \
  --python_out=src/msg/protobuf_msgs/protobuf_msgs --grpc_python_out=src/msg/protobuf_msgs/protobuf_msgs \
  src/msg/protobuf_msgs/protobuf_msgs/protos/**/*.proto
