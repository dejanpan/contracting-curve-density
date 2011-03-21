CC = g++
GCC= gcc
EXTRA = 
INCDIR =-I. `pkg-config --cflags opencv` -I../include -I../../sift/include
LDDIR = -L./lib -L../../sift/lib
CFLAGS = -g3 -Wall -DLINUX $(EXTRA) $(LDDIR)
LDLIBS =
LDFLAGS = $(LDLIBS) -lm `pkg-config --libs opencv` ../../sift/lib/libfeat.a
OBJECTS=bspline.o ccd.o
$(OBJECTS): %.o: %.cpp
	$(CC) $(INCDIR) -c $(CFLAGS) $< -o $@
PROGRAMS=snake test ros_to_openCv ccd2 ccd_panin
all:$(PROGRAMS)
clean:
	@rm -rf $(PROGRAMS) *.o $(PROGRAMS) ../data/bspline*
ros_to_openCv:ros_to_openCv.cpp
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $< $(LDFLAGS)
snake:snake.cpp
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $< $(LDFLAGS)
test:test.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
sift:sift.c $(OBJECTS)
	$(GCC) $(INCDIR) $(CFLAGS)  -o $@ $@.c $(OBJECTS) $(LDFLAGS)
test1:test1.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
draw:draw.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
ccd:ccd.cpp ccd.h $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
ccd_panin:ccd_panin.cpp ccd_panin.h $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
ccd2:ccd2.cpp ccd2.h $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
ccd_circle:ccd_circle.cpp ccd.h $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
poly:poly.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
cov:cov.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
curve:curve.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
matrix:matrix.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)

recompile:clean all

.PHONY: check-syntax
check-syntax:
	$(CC) $(INCDIR) $(CFLAGS) -Wall -Wextra -pedantic -fsyntax-only $(CHK_SOURCES)
