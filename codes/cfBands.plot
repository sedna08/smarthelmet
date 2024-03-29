 set title 'Bands';
set xlabel 'time (s)';
set ylabel 'EEG values';
set ytics nomirror
#set yrange [0:50000000];
#xmin = 0;
#xmid = 1;
#xmax = 2;
#set xrange[xmin:xmax];
#set xtics (xmin,xmid,xmax);
set grid x,y;
show grid;
plot "downdata.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double" using 1:2 title 'Delta' with lines lw 1, \
    "downdata.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double" using 1:3 title 'Theta' with lines lw 1, \
    "downdata.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double" using 1:4 title 'LowAlpha' with lines lw 1, \
    "downdata.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double" using 1:5 title 'HighAlpha' with lines lw 1, \
    "downdata.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double" using 1:6 title 'LowBeta' with lines lw 1, \
    "downdata.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double" using 1:7 title 'HighBeta' with lines lw 1, \
    "downdata.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double" using 1:8 title 'LowGamma' with lines lw 1, \
    "downdata.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double" using 1:9 title 'MidGamma' with lines lw 1
    
while(1) {
    #set xrange [xmin+0.5:xmax+0.5];
    #xmin = xmin + 0.5;
    #xmax = xmax + 0.5;
    #xmid = xmid + 0.5;
    #set xtics (xmin,xmid,xmax);
    replot; 
    pause 3;
}
