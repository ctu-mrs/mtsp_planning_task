#!/bin/bash

# install python dependencies
sudo apt-get -y install python-matplotlib python3-matplotlib python-sklearn python3-sklearn
sudo -H pip install dubins sklearn

# install LKH solver
./mtsp_planner/scripts/solvers/lkh/install.sh
