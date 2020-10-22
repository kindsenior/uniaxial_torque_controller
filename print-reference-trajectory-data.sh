#!/bin/bash

roseus "(load \"reference-trajectory-generator.l\")" "(load \"$1\")" "(send *tg* :print-param)" 2>/dev/null
