#!bin/bash
mkdir resized
for f in *.jpg
do
    convert $f -resize $1x$2! resized/$f
done
