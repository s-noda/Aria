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

if [ -e aria.urdf ]; then echo " mv aria.urdf /tmp" ; mv -f aria.urdf /tmp ; fi ;
if [ -e aria_for_eus.urdf ]; then echo " mv aria_for_eus.urdf /tmp" ; mv -f aria_for_eus.urdf /tmp ; fi ;
if [ -e aria.dae ]; then echo " mv aria.dae /tmp" ; mv -f aria.dae /tmp ; fi ;
if [ -e aria.l ]; then echo " mv aria.l /tmp" ; mv -f aria.l /tmp ; fi ;

roseus "(progn (load \"gen-aria.l\") (gen-urdf) (exit 1))" ;

URDF_BUF="aria_for_eus.urdf";
cp aria.urdf $URDF_BUF;
sed -i "s/^.\+aria_head_simple\.dae.\+scale.\+$/<mesh filename=\"package:\/\/aria_model\/dae\/aria_head_simple.dae\" scale=\"0.001 0.001 0.001\" \/>/g" $URDF_BUF ;
sed -i "s/^.\+aria_eye\.dae.\+scale.\+$/<mesh filename=\"package:\/\/aria_model\/dae\/aria_eye.dae\" scale=\"0.001 0.001 0.001\" \/>/g" $URDF_BUF ;

rosrun collada_urdf urdf_to_collada $URDF_BUF aria.dae ;
rosrun euscollada collada2eus aria.dae aria.yaml aria.l ;
if [ "$DEBUG" ]; then roseus "(progn (load \"aria.l\") (objects (list (setq a (aria)))))"; fi ;

