#!/usr/bin/perl 

use warnings; 
use strict;
use POSIX;

my $FH;
my $PORT="/dev/ttyUSB0";
my $LOGFILE = $ARGV[0] || "/dev/null";
open($FH, "+< $PORT") || die "Cannot open port: $!\n";;
system("stty -F $PORT 115200 raw");

my $FOUT;
open($FOUT,">> $LOGFILE") || die "Cannot open output file: $!\n";

select((select($FH),$|=1)[0]);
select((select($FOUT),$|=1)[0]);
$|=1;

print $FH "\n\n#L,W,3,E,0,1;\n\n";

my $var=0; 

while (defined(my $line=<$FH>)) {
   chomp $line;
   $var++; 
   my (undef, undef, $w, $v, $a, $wh) = split(/,/, $line);
   next unless $w;
   print "$w\n";
   print $FOUT strftime("%H:%M:%S", localtime())." $var $v $w $a $wh\n";
};
