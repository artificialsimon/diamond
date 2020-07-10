#!/bin/bash

xhost local:root
docker  run --rm -e DISPLAY=$DISPLAY --net=host -v ~/.cache/Xauthority -v /home/simon/Documents/artificial/projects/diamond/lpzrobots:/opt/lpzrobots -it --name lpzrobots lpzrobots_image /bin/bash
#docker run --rm -e DISPLAY=$DISPLAY --net=host -v ~/.cache/Xauthority                                                                                -it --name lpzrobots lpzrobots_image /bin/bash
xhost -local:root
