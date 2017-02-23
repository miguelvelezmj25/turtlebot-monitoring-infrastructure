#/bin/bash

# Uses libavcodec video codec
# Uses MP3Lame audio codec

# $1 is vmax_b_frames
# $2 is vb_strategy
# $3 is cbp, mv0
# $4 is qns
# $5 is qpel
# $6 is dia
# $7 is predia
# $8 is cmp
# $9 is last_pred
# $10 is qprd
# $11 is vqcomp
# $12 is vlelim, vcelim
# $13 is output file

OUTPUT=${13}

mencoder -msglevel all=3 test1.m4v -o $OUTPUT -ovc lavc -oac mp3lame -lavcopts vcodec=mpeg4:vmax_b_frames=$1:vb_strategy=$2:cbp=$3:mv0=$3:qns=$4:qpel=$5:dia=$6:predia=$7:cmp=$8:last_pred=$9:qprd=${10}:vqcomp=${11}:vlelim=${12}:vcelim=${12}
