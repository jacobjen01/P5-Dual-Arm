#!/bin/bash
echo "Remember to run docker compose up alone the first time"
docker compose up -d
xhost +local:root
sleep 0.1
docker exec -it p5-dual-arm-dual-arm-1 bash
