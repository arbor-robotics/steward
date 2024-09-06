To compile from Steward root (e.g. `/home/main/steward'):

```bash
python3 -m grpc_tools.protoc -Isrc/msg/proto_msgs/proto --python_out=src/msg/proto_msgs/python/ --pyi_out=src/msg/proto_msgs/python/ --grpc_python_out=src/msg/proto_msgs/python/ src/msg/proto_msgs/proto/**/*proto
```