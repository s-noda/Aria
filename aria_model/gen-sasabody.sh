
roseus "(progn (load \"gen-sasabody.l\") (gen-urdf) (exit 1))" ;
rosrun collada_urdf_jsk_patch urdf_to_collada sasabody.urdf sasabody.dae ;
rosrun euscollada collada2eus sasabody.dae  sasabody.yaml sasabody.l ;
roseus "(progn (load \"sasabody.l\") (objects (list (setq a (sasabody)))))";
