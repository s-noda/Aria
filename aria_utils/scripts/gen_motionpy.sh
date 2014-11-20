#!/bin/bash
sgn=()
j=0
while read line
do
    sgn[j]="$line"
    j=$(($j + 1))
done < $(rospack find aria_utils)/settings/sgn.dat

offset=()
j=0
while read line
do
    offset[j]="$line"
    j=$(($j + 1))
done < $(rospack find aria_utils)/settings/offset.dat

gen_dir="$(rospack find aria_utils)/scripts/generated"
if [ ! -d "${gen_dir}" ]
then
    mkdir "${gen_dir}"
fi

fix_pyscript() {
    tab=$'\t'
    tab4=$'    '
    tab8=$'        '
    readtab='#|tab|#'
    sed "s/${tab}/${readtab}/g" $1 > tmp
    sed "s/${tab4}/${readtab}/g" tmp > tmp1 && mv tmp1 tmp
    sed "s/${tab8}/${readtab}/g" tmp > tmp1 && mv tmp1 tmp
    
    array=()
    i=0
    while read line
    do
	array[i]=$(echo "$line" | sed "s/${readtab}/${tab}/g")
	i=$(($i + 1))
    done < tmp

    body=''

    for e in "${array[@]}"
    do
	if [[ $e == *"set_positions"* ]]
	then
	    idx=0
	    line="${e##*[}"
	    line="${line:0:${#line}-2}"
	    replace=$(cut -d '[' -f 1 <<< "$e" )
	    replace="${replace}["
	    while IFS="," read -ra ARR; do
		for i in "${ARR[@]}"; do
		    val=$(echo "$i * ${sgn[${idx}]}" | bc | awk '{printf "%f", $0}')
		    val=$(echo "$val + ${offset[${idx}]}" | bc | awk '{printf "%f", $0}')
		    replace="${replace}${val},"
		    idx=$(($idx + 1))
		done
	    done <<< "$line"
	    replace="${replace:0:${#replace}-1}"
	    replace="${replace}])"
	    body="${body}${replace}\n"
	else
	    body="${body}${e}\n"
	fi
    done

    echo -e "$body" > $2
    chmod +x $2
}

for entry in $(rospack find aria_utils)/sample_motions/*.py
do
    tmp="${entry##*/}"
    echo "generating ${tmp}"
    fix_pyscript "${entry}" "${gen_dir}/${tmp}"
done

header='#!/usr/bin/env python\nimport aria\nfrom std_msgs.msg import String\n'
packages=''

#for entry in $(rospack find aria_utils)/sample_motions/*.py
for entry in ${gen_dir}/*.py
do
    tmp="${entry##*/}"
    packages="${packages}from ${tmp:0:${#tmp}-3} import *\n"
done

def='\naria.init_publisher()\n'
def="${def}motions = {"

#for entry in $(rospack find aria_utils)/sample_motions/*.py
for entry in ${gen_dir}/*.py
do
    tmp="${entry##*/}"
    def=$"${def}'${tmp:0:${#tmp}-3}': ${tmp:0:${#tmp}-3},"
done

def="${def:0:${#def}-1}"
def="${def}}\n"

body="$header$packages$def"
body="${body}\ndef callback(msg):\n\tmotions[msg.data]()\n\n"

main="if __name__ == '__main__':"
main="${main}\n\trospy.init_node('motion_runner',anonymous=True)\n\trospy.Subscriber('/aria/commandline',String,callback)\n\trospy.spin()"

body="$body$main"

echo -e $body > "${gen_dir}/sample_motion_publisher.py"
chmod +x "${gen_dir}/sample_motion_publisher.py"