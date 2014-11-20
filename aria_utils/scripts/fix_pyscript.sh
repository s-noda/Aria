#!/bin/bash
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

if [ -f "$2" ]
then
    echo "$2 exists"
else
    echo -e "$body" > $2
    chmod +x $2
fi
