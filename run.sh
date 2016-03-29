#!/bin/bash

./src/april/april_runner &
./bin/runner

echo ""
trap 'kill $(jobs -p)' EXIT

while true
do
	sleep 1
done