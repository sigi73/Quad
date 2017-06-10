all : test

#CFLAGS = -fPIC -g -Wall `pkg-config --cflags opencv`
CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
INCLUDE = -I/usr/local/include/libfreenect -I/usr/include/libusb-1.0/
FREE_LIBS = -L/usr/local/lib -lfreenect -L/usr/local/lib -lusb-1.0 -pthread

test: test.cpp
	$(CXX) $(INCLUDE) $(CFLAGS) $? -o $@ $(LIBS) $(FREE_LIBS)

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o test
