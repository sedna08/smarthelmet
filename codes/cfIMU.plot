# set title 'IMU Values';
set xlabel 'time (s)';
set ylabel 'angle (degrees)';

set ytics nomirror
# set y2tics 0, 60


set yrange [-90:90];
# set y2range [0:6.283185307179586];
set grid x,y;
show grid;

plot "downdata.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double" using 1:10 title 'Roll' with lines lw 1, \
	"downdata.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double" using 1:11 title 'Pitch' with lines lw 1, \


while(1) {
    replot;
    pause 3;
}
