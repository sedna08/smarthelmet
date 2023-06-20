CC = g++
CCW = -Wall
CVLIB = $$(pkg-config --cflags --libs opencv4)
LDFLAGS = -lpigpio -pthread -lpthread

smarthelmet: main.cpp cv.cpp imu.cpp eeg.cpp Parser_Filter.cpp kalmanFilter.cpp
	$(CC) -o smarthelmet main.cpp eeg.cpp imu.cpp cv.cpp kalmanFilter.cpp Parser_Filter.cpp $(CVLIB) $(LDFLAGS)
	
smarthelmet1: main_mlvlclf.cpp cv.h imu.cpp eeg.cpp Parser_Filter.cpp kalmanFilter.cpp
	$(CC) -o smarthelmet1 main_mlvlclf.cpp eeg.cpp imu.cpp cv.cpp kalmanFilter.cpp Parser_Filter.cpp $(CVLIB) $(LDFLAGS)

smarthelmet2: main_slvlclf.cpp cv.h imu.cpp eeg.cpp Parser_Filter.cpp kalmanFilter.cpp
	$(CC) -o smarthelmet2 main_slvlclf.cpp eeg.cpp imu.cpp cv.cpp kalmanFilter.cpp Parser_Filter.cpp $(CVLIB) $(LDFLAGS)
	
temp: main_slvlclf_cpy.cpp cv.cpp imu.cpp eeg.cpp Parser_Filter.cpp kalmanFilter.cpp
	$(CC) -o temp main_slvlclf_cpy.cpp eeg.cpp imu.cpp cv.cpp kalmanFilter.cpp Parser_Filter.cpp $(CVLIB) $(LDFLAGS)

main.o: main.cpp
	$(CC) $(CCW) main.cpp -c

eeg.o: eeg.cpp
	$(CC) $(CCW) eeg.cpp -c

imu.o: imu.cpp
	$(CC) $(CCW) imu.cpp -c

Parser_Filter.o: Parser_Filter.cpp
	$(CC) $(CCW) Parser_Filter.cpp -c

kalmanFilter.o: kalmanFilter.cpp
	$(CC) $(CCW) kalmanFilter.cpp -c

clear: 
	rm -f *.o *.txt *.bin data smarthelmet smarthelmet1 smarthelmet2

decode:
	$(CC) $(CCW) -o decode decode.cpp

cdata:
	rm -f *.bin *.txt
	
cv.o: cv.cpp
	$(CC) $(CCW) cv.cpp -c
