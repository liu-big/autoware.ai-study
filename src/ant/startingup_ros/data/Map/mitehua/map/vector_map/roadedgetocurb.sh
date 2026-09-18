#!/bin/bash

cp roadedge.csv curb.csv
mv roadedge.csv roadedge.csv_bak

sed -i '1d' curb.csv
sed -i '1i\\ID,LID,Width,Color,type,LinkID' curb.csv

sed -i "s/,0/,0.1,0.1,1,0/g" curb.csv