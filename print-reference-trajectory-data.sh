#!/bin/bash

if [ $# -lt 1 ]; then
    echo "[Error] Useage ./print-reference-trajectory-data.sh <reference-trajectory-file-name.l>"
    exit
fi

roseus "(load \"reference-trajectory-generator.l\")" "(load \"$1\")" "(send *tg* :print-param)" 2>/dev/null
