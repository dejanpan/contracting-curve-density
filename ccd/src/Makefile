CC = g++
EXTRA = 
INCDIR =-I. -I./include -I/usr/include -I/usr/include/opencv -I/opt/ros/cturtle/ros/core/roscpp/include/ -I/opt/ros/cturtle/ros/core/rosconsole/
LDDIR = -L./lib -L/usr/lib64 -L/usr/lib
CFLAGS = -g3 -Wall -DLINUX $(EXTRA) $(INCDIR) $(LDDIR)
LDLIBS =
LDFLAGS = $(LDLIBS) -lm -lcv -lhighgui
OBJECTS=bspline.o 
$(OBJECTS): %.o: %.cpp
	$(CC) $(INCDIR) -c $(CFLAGS) $< -o $@
PROGRAMS=snake test ros_to_openCv
all:$(PROGRAMS)
clean:
	@rm -rf $(PROGRAMS) *.o
ros_to_openCv:ros_to_openCv.cpp
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $< $(LDFLAGS)
snake:snake.cpp
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $< $(LDFLAGS)
test:test.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
recompile:clean all

.PHONY: check-syntax
check-syntax:
	$(CC) $(INCDIR) $(CFLAGS) -Wall -Wextra -pedantic -fsyntax-only $(CHK_SOURCES)
