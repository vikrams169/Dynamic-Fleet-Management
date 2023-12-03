#!/bin/bash
#
# A convenient script to create documentation
#
set -ue -o pipefail

###############################
# 0. check needed software
###############################
if (! which pandoc ); then
    echo "Please insall pandoc first. Try:"
    echo "  sudo apt install pandoc"
    exit 1
fi

###############################
# 1. Source the underlay
###############################
set +u                          # stop checking undefined variable  
source /opt/ros/humble/setup.bash
set -u                          # re-enable undefined variable check

###############################
# 2. run my_model's "docs" target
###############################
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select my_model \
       --cmake-target "docs"
##echo "open src/my_model/docs/html/index.html"

###############################
# 3. run my_controller's "docs" target
###############################
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select my_controller \
       --cmake-target "docs"
##echo "open src/my_controller/docs/html/index.html"

###############################
# 4. combine all docs
###############################
DOCS_DIR=src/docs/
mkdir -p $DOCS_DIR
cat << "EOF" > $DOCS_DIR/index.md
# ENPM808X Final Project Documentation 

Welcome to ENPM808X final project page.  

Here, you will find all the documentation for this project:

* [my_model](../my_model/docs/html/index.html)
* [my_controller](../my_controller/docs/html/index.html)

Have a nice day.
EOF
pandoc -f markdown $DOCS_DIR/index.md > $DOCS_DIR/index.html
open $DOCS_DIR/index.html || true

