CC = g++
EXTRA = 
INCDIR =-I. -I./include -I/usr/include -I/usr/include/opencv -I/opt/ros/cturtle/ros/core/roscpp/include/
LDDIR = -L./lib -L/usr/lib64 -L/usr/lib
CFLAGS = -g3 -Wall -DLINUX $(EXTRA) $(INCDIR) $(LDDIR)
LDLIBS =
LDFLAGS = $(LDLIBS) -lm -lcv -lhighgui
OBJECTS=bspline.o ccd.o
$(OBJECTS): %.o: %.cpp
	$(CC) $(INCDIR) -c $(CFLAGS) $< -o $@
PROGRAMS=snake test
all:$(PROGRAMS)
clean:
	@rm -rf $(PROGRAMS) *.o
snake:snake.cpp
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $< $(LDFLAGS)
test:test.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
recompile:clean all

.PHONY: check-syntax
check-syntax:
	$(CC) $(INCDIR) $(CFLAGS) -Wall -Wextra -pedantic -fsyntax-only $(CHK_SOURCES)
