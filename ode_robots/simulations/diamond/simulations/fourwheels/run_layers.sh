#!/bin/bash
## on revision 
## looking for best learning rates and lambdas 

## linear with extended model
for layers in 1 2 3 4 5 
do
  for zsize in 0.0 0.25 0.50 0.75 1.0
  do
    for trial in {1..100}
    do
      echo TRIAL $trial/100
      echo ./start -log -terrain 1 -playground $zsize -simtime 10 -layers $layers -nographics -threads 1
      ./start -log -terrain 1 -playground $zsize -simtime 10 -layers $layers -nographics -threads 1
    done
  done
done
