#!/bin/bash
SYMBOL=$1
echo $SYMBOL
LINE=`nm libedit_lasso.so | grep -n  $SYMBOL | grep -o "^[0-9]*"`;
UNMANGLED=`nm libedit_lasso.so | c++filt | awk "NR==${LINE}"`
echo $UNMANGLED
