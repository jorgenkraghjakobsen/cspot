#!/usr/bin/env bash

set -e
echo cocker 12
cd cpp-reflection
go build -o protoc-gen-cpprefl .
cd ..
mkdir -p protos
protoc --plugin=protoc-gen-cpprefl=cpp-reflection/protoc-gen-cpprefl --cpprefl_out protos/ protocol/*.proto --proto_path protocol/
