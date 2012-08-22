#!/bin/bash
FILE=$1
SYMBOL=$2
echo "Find $SYMBOL in $FILE"
LINE=`nm $FILE | grep -n  $SYMBOL | grep -o "^[0-9]*"`;
UNMANGLED=`nm $FILE | c++filt | awk "NR==${LINE}"`
echo $UNMANGLED
