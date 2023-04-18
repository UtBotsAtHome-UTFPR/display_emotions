#!/bin/bash

#for file in *; do echo "$file"; done

for filename in *.png
do
    convert "$filename" -crop 1280x720+0+130 "${filename%.png}".png
done