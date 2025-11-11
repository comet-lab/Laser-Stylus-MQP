CC = g++

all: main.exe CalibrateRobot.exe


main: main.o PandaController.o FrankaSubscriber.o
	$(CC) -o main main.o PandaController.o FrankaSubscriber.o -L /home/nepacheco/libfranka/build/ \
	-lfranka -L /home/nepacheco/libfranka/build/examples -lexamples_common -L/opt/ros/noetic/lib \
	-Wl,-rpath,/opt/ros/noetic/lib -lroscpp -lrosconsole -lrostime \
	-lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib

CalibrateRobot: CalibrateRobot.o PandaController.o 
	$(CC) -o CalibrateRobot CalibrateRobot.o PandaController.o -L /home/nepacheco/libfranka/build/ \
	-lfranka -L /home/nepacheco/libfranka/build/examples -lexamples_common -L/opt/ros/noetic/lib \
	-Wl,-rpath,/opt/ros/noetic/lib -lroscpp -lrosconsole -lrostime \
	-lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib
	
main.o: main.cpp PandaController.h FrankaSubscriber.h UDPClient.h
	$(CC) -I ~/libfranka/include/ -I ~/libfranka/examples/ -I /usr/include/eigen3/ -I /opt/ros/noetic/include/ -c main.cpp

CalibrateRobot.o: CalibrateRobot.cpp PandaController.h
	$(CC) -I ~/libfranka/include/ -I ~/libfranka/examples/ -I /usr/include/eigen3/ -I /opt/ros/noetic/include/ -c CalibrateRobot.cpp

PandaController.o: PandaController.cpp PandaController.h
	$(CC) -I ~/Repositories/RL_laser_control/cpp_src/ -I ~/libfranka/include/ -I ~/libfranka/examples/ \
	-I /usr/include/eigen3/ -c  PandaController.cpp 

FrankaSubscriber.o: FrankaSubscriber.cpp FrankaSubscriber.h
	$(CC) -I ~/Repositories/RL_laser_control/cpp_src/ -I /opt/ros/noetic/include/ -c  FrankaSubscriber.cpp 
CameraAcquisition: CameraAcquisition.o
	$(CC) -o CameraAcquisition CameraAcquisition.o -L /opt/spinnaker/lib/ -lSpinnaker

CameraAcquisition.o: CameraAcquisition.cpp
	$(CC) -c CameraAcquisition.cpp -I /opt/spinnaker/include/ -I /opt/spinnaker/include/SpinGenApi/
