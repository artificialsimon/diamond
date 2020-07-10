#!/bin/bash
docker start lpzrobots
xhost local:root
docker exec -it lpzrobots bash
xhost -local:root
