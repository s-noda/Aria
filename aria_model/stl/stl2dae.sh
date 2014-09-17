ROOT="../dae" ;
mkdir $ROOT ;

for p in `ls | grep -e "stl$" | sed "s/\ /_/g"`;
do
    org=`echo $p | sed s/_/\ /g` ;
    name=`echo $org | sed "s/^\(.*\)\.stl$/\1/g"`;
    echo "convert \"$name.stl\" -> \"$ROOT/$name.dae\"";
    meshlabserver -i "$name.stl" -o "$ROOT/$name.dae" ;
done

# for p in `ls`;
# do
#     name=`echo $p | sed "s/^\(.*\)\.stl$/\1/g"`;
#     echo "convert $p -> $name.dae";
#     meshlabserver -i "$p" -o "$name.dae" ;
# done ;


## ls | grep -e "stl$" | sed "s/^\(.*\)$/\"\1\"/g"
## IFS="\n" && for p in `ls | grep -e "stl$"`; do echo $p; done
