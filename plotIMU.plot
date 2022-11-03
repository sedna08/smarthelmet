# set title 'IMU Values';
set xlabel 'time (s)';
set ylabel 'angle (degrees)';

set ytics nomirror
# set y2tics 0, 60


set yrange [-180:180];
# set y2range [0:6.283185307179586];
set grid x,y;
show grid;

plot "data.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double%double%double%double" using 1:13 title 'Roll' with lines lw 1, \
	"data.bin" binary format="%double%double%double%double%double%double%double%double%double%double%double%double%double%double%double" using 1:14 title 'Pitch' with lines lw 1, \


while(1) {
    replot
    pause 0.5
}
