protoc -I=./src/ --go_out=../ ./src/msgs.proto
protoc --python_out=python/ --proto_path ./src  ./src/msgs.proto