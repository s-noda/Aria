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
roseus "(progn (load \"gen-sasabody.l\") (gen-urdf) (exit 1))" ;
rosrun collada_urdf_jsk_patch urdf_to_collada sasabody.urdf sasabody.dae ;
rosrun euscollada collada2eus sasabody.dae  sasabody.yaml sasabody.l ;
## roseus "(progn (load \"sasabody.l\") (objects (list (setq a (sasabody)))))";
