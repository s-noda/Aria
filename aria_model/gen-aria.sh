#!/usr/bin/env sh

if [ "$1" ] ;
then
    PROJECT_DIR=$1;
else
    PROJECT_DIR=`pwd`;
fi;
echo "[gen-aria-model]";
echo " build in $PROJECT_DIR";

cd $PROJECT_DIR;

if [ -e aria.urdf ]; then echo " mv aria.urdf /tmp" ; mv aria.urdf /tmp ; fi ;
if [ -e aria.dae ]; then echo " mv aria.dae /tmp" ; mv aria.dae /tmp ; fi ;
if [ -e aria.l ]; then echo " mv aria.l /tmp" ; mv aria.l /tmp ; fi ;

roseus "(progn (load \"gen-aria.l\") (gen-urdf) (exit 1))" ;
rosrun collada_urdf urdf_to_collada aria.urdf aria.dae ;
rosrun euscollada collada2eus aria.dae  aria.yaml aria.l ;
if [ "$DEBUG" ]; then roseus "(progn (load \"aria.l\") (objects (list (setq a (aria)))))"; fi ;
