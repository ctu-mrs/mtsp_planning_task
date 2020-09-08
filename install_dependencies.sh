#!/bin/bash

#install python dependencies
sudo apt-get install python-matplotlib python3-matplotlib python-sklearn python3-sklearn 
sudo pip install dubins

#install LKH solver
./mtsp_planner/scripts/solvers/lkh/install.sh


