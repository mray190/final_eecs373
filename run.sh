
echo "Starting rexarm driver"
./bin/rexarm_driver -d /dev/ttyUSB0 &

echo "Starting task1"
./bin/task1 < ./src/solutions/task1/red.in &

echo "Starting task2"
./bin/task2 & 

echo "Starting turn producer"
./bin/turnProducer

echo ""
trap 'kill $(jobs -p)' EXIT

while true
do
	sleep 5
done