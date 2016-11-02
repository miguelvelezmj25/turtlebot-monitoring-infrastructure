#/bin/bash

# Uses Xvid video codec
# Uses MP3Lame audio codec

# $1 is vhq on/off
# $2 is bvhq on/off
# $3 is trellis on/off
# $4 is hq_ac on/off
# $5 is chroma_me on/off
# $6 is chroma_opt on/off
# $7 is lumi_mask on/off
# $8 is qpel on/off
# $9 is gmc on/off
# $10 is output file
# bitrate is 350

OUTPUT=${10}

mencoder -msglevel all=0 test1.m4v -o $OUTPUT -ovc xvid -oac mp3lame -xvidencopts bitrate=350:vhq=$1:bvhq=$2:trellis=$3:hq_ac=$4:chroma_me=$5:chroma_opt=$6:lumi_mask=$7:qpel=$8:gmc=$9

# echo vhq=$1 bvhq=$2 trellis=$3 hq_ac=$4 chroma_me=$5 chroma_opt=$6 lumi_mask=$7 qpel=$8 gmc=$9 >> time_log.txt

# Calls Mencoder on test1.m4v with options specified

