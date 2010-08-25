CC = g++
EXTRA = 
INCDIR =-I. -I./include -I/home/dimilar/ros/external/shulei/ccd/include -I/opt/ros/cturtle/stacks/vision_opencv/opencv2/opencv/include -I/opt/ros/cturtle/stacks/vision_opencv/opencv2/opencv/include/opencv -I/opt/ros/cturtle/stacks/vision_opencv/cv_bridge/include -I/opt/ros/cturtle/stacks/image_common/image_transport/include -I/opt/ros/cturtle/stacks/common_msgs/sensor_msgs/include -I/opt/ros/cturtle/stacks/common_msgs/sensor_msgs/msg/cpp -I/opt/ros/cturtle/stacks/common_msgs/sensor_msgs/srv/cpp -I/opt/ros/cturtle/stacks/common_msgs/sensor_msgs/msg_gen/cpp/include -I/opt/ros/cturtle/stacks/common_msgs/sensor_msgs/srv_gen/cpp/include -I/opt/ros/cturtle/stacks/common_msgs/geometry_msgs/msg/cpp -I/opt/ros/cturtle/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include -I/opt/ros/cturtle/ros/tools/rosrecord/include -I/opt/ros/cturtle/ros/tools/rosbag/include -I/opt/ros/cturtle/ros/tools/topic_tools/include -I/opt/ros/cturtle/ros/tools/topic_tools/srv_gen/cpp/include -I/opt/ros/cturtle/ros/std_msgs/include -I/opt/ros/cturtle/ros/std_msgs/msg_gen/cpp/include -I/opt/ros/cturtle/stacks/common/pluginlib/include -I/opt/ros/cturtle/stacks/common/pluginlib -I/opt/ros/cturtle/stacks/common/tinyxml/include -I/opt/ros/cturtle/ros/core/message_filters/include -I/opt/ros/cturtle/ros/test/rostest/include -I/opt/ros/cturtle/ros/core/roscpp/include -I/opt/ros/cturtle/ros/core/roscpp/msg_gen/cpp/include -I/opt/ros/cturtle/ros/core/roscpp/srv_gen/cpp/include -I/opt/ros/cturtle/ros/3rdparty/xmlrpcpp/src -I/opt/ros/cturtle/ros/core/rosconsole/include -I/opt/ros/cturtle/ros/core/roslib/include -I/opt/ros/cturtle/ros/core/roslib/msg_gen/cpp/include -I/opt/ros/cturtle/ros/tools/rospack -I/opt/ros/cturtle/ros/tools/rospack/include -I/opt/ros/cturtle/ros/3rdparty/gtest/gtest/include
LDDIR = -L./lib -L/usr/lib64 -L/usr/lib  -rdynamic -L/opt/ros/cturtle/stacks/vision_opencv/opencv2/opencv/lib -L/opt/ros/cturtle/stacks/image_common/image_transport/lib -L/opt/ros/cturtle/stacks/common_msgs/sensor_msgs/lib -L/opt/ros/cturtle/ros/tools/rosrecord/lib -L/opt/ros/cturtle/ros/tools/rosbag/lib -L/opt/ros/cturtle/ros/tools/topic_tools/lib -L/opt/ros/cturtle/stacks/common/pluginlib/lib -L/opt/ros/cturtle/stacks/common/tinyxml -L/opt/ros/cturtle/ros/core/message_filters/lib -L/opt/ros/cturtle/ros/core/roscpp/lib -L/opt/ros/cturtle/ros/3rdparty/xmlrpcpp/lib -L/opt/ros/cturtle/ros/core/rosconsole/lib -L/opt/ros/cturtle/ros/core/roslib/lib -L/opt/ros/cturtle/ros/tools/rospack/lib -L/opt/ros/cturtle/ros/3rdparty/gtest/gtest/lib
CFLAGS = -g3 -Wall -DLINUX $(EXTRA) $(INCDIR) $(LDDIR)
LDLIBS =
LDFLAGS = $(LDLIBS) -lm -lcv -lhighgui  -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_objdetect -lopencv_features2d -lopencv_calib3d -lopencv_contrib -lopencv_legacy -limage_transport -lsensor_msgs -lrosrecorder -lrosbag -ltopic_tools -lpoco_lite -lboost_fs_wrapper -ltinyxml -lmessage_filters -lros -lboost_signals-mt -lXmlRpc -lrosconsole -lboost_thread-mt -llog4cxx -lroslib -lrospack -lrosstack -Wl,-rpath,/opt/ros/cturtle/stacks/vision_opencv/opencv2/opencv/lib:/opt/ros/cturtle/stacks/image_common/image_transport/lib:/opt/ros/cturtle/stacks/common_msgs/sensor_msgs/lib:/opt/ros/cturtle/ros/tools/rosrecord/lib:/opt/ros/cturtle/ros/tools/rosbag/lib:/opt/ros/cturtle/ros/tools/topic_tools/lib:/opt/ros/cturtle/stacks/common/pluginlib/lib:/opt/ros/cturtle/stacks/common/tinyxml:/opt/ros/cturtle/ros/core/message_filters/lib:/opt/ros/cturtle/ros/core/roscpp/lib:/opt/ros/cturtle/ros/3rdparty/xmlrpcpp/lib:/opt/ros/cturtle/ros/core/rosconsole/lib:/opt/ros/cturtle/ros/core/roslib/lib:/opt/ros/cturtle/ros/tools/rospack/lib:/opt/ros/cturtle/ros/3rdparty/gtest/gtest/lib
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
draw:draw.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
ccd:ccd.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
poly:poly.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)
cov:cov.cpp $(OBJECTS)
	$(CC) $(INCDIR) $(CFLAGS)  -o $@ $@.cpp $(OBJECTS) $(LDFLAGS)

recompile:clean all

.PHONY: check-syntax
check-syntax:
	$(CC) $(INCDIR) $(CFLAGS) -Wall -Wextra -pedantic -fsyntax-only $(CHK_SOURCES)
