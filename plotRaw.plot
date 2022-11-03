 set title 'Raw Values';
set xlabel 'time (s)';
set ylabel 'EEG values';
set ytics nomirror
set yrange [-2000:2000];
#xmin = 0;
#xmid = 1;
#xmax = 2;
#set xrange[xmin:xmax];
#set xtics (xmin,xmid,xmax);
set grid x,y;
show grid;
plot "data.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double%double%double%double" using 1:2 title 'Raw values' with lines lw 1, \
#	"data.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double%double%double%double" using 1:3 title 'Sum' with lines lw 1, \
count = 0;
window = 1;

while(1) {
    #set xrange [xmin+0.4:xmax+0.4];
    #xmin = xmin + 0.4;
    #xmax = xmax + 0.4;
    #xmid = xmid + 0.4;
    #set xtics (xmin,xmid,xmax);
    replot;
}
