#!/bin/bash
       echo "Script is running"
while :
    do
       sudo -u siddharth tail ~/output.txt -n 3 > output1.txt
       cp ./output1.txt /var/www/html
       sleep 1
    done

