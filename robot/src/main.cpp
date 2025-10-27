#include "PandaController.h"
#include "FrankaSubscriber.h"
#include "UDPClient.h"
#include "UDPServer.h"
#include <thread>
#include <chrono>

void thread_function(PandaController*, double*, double orienTargetPtr[][3], double);
void parseMessage(char receiveBuffer[12*8 + 1], double * posTarget, double orienTarget[3][3], char* mode);

int main(int argc, char** argv) {
  double posTarget[3] = {0.34, 0.0, 0.6};
  double orienTargetPtr[3][3] = {{1,0,0},{0,-1,0},{0,0,-1}};
  double currState[] = {0,0,0,0,0,0,0};
  char mode = 0;
  char receiveBuffer[12*8 + 1];
  double timeDuration = 1000;
  UDPServer udpServer = UDPServer();
  PandaController * controller = new PandaController("172.16.0.2",mode,posTarget, orienTargetPtr,currState); 

  /* Get first Reading from python to set control mode */
  // controller->getPose();
  // std::cout << "Waiting for First Message" << std::endl;
  // udpServer.waitForRequest(receiveBuffer,currState,sizeof(currState));
  // parseMessage(receiveBuffer,posTarget,orienTargetPtr,&mode);
  // controller->setMode(mode);

  if (argc > 1){
    timeDuration = std::stod(argv[1]);
  }
  // Start Control thread
  std::thread thread_object(thread_function, controller, posTarget, orienTargetPtr, timeDuration);

  // The client requests a position setpoint and the message received from 
  // the server is 3 doubles which overrides the posTarget variable
  double elapsedTime = 0;
  auto startTime = std::chrono::high_resolution_clock::now();
  int count = 0;
  while (mode >=0) {
    udpServer.waitForRequest(receiveBuffer,currState,sizeof(currState));
    parseMessage(receiveBuffer,posTarget,orienTargetPtr,&mode);
    controller->setMode(mode);

    auto currentTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime-startTime);
    elapsedTime = duration.count()/1000.0; // convert milliseconds to seconds
    count ++;
  }
  std::cout << "Packets Sent: " << count << std::endl;
  std::cout << "Time Duration: " << elapsedTime << " s" << std::endl;
  std::cout << "UDP Rate: " << count/elapsedTime << " packets per second" << std::endl;
   

  // std::cout << "Subscriber Created" << std::endl;
  thread_object.join();
  // std::cout << "Thread Joined" << std::endl;
  return 0;
}

void thread_function(PandaController * controller, double* posTargetPtr,double orienTargetPtr[][3], double timeDuration){
  //controller->moveToStart(0.25, 0.00, 0.50); // This is needed to set desired Orientation
  // sleep(1);
  controller->runController(timeDuration);
  // std::cout << "Done with thread" << std::endl;
}

void parseMessage(char receiveBuffer[12*8 + 1], double * posTarget, double orienTarget[3][3], char* mode){
  double poseTarget[12];
  memcpy(poseTarget,receiveBuffer,sizeof(poseTarget));
  // std::cout << "Retrieving Pose" << std::endl;
  for (int i = 0; i < 12; i++){
      if (i < 9){
        orienTarget[i/3][i%3] = poseTarget[i];
      } else{
        posTarget[i-9] = poseTarget[i];
      }
    }
  // std::cout << "Retrieving Mode" << std::endl;
  *mode = receiveBuffer[12*8];

  // std::cout << "X: " << posTarget[0] << std::endl;
  // std::cout << "Y: " << posTarget[1] << std::endl;
  // std::cout << "Z: " << posTarget[2] << std::endl;
  // std::cout << "Mode: " << +(*mode) << std::endl;

}

  
// //Thread function from main branch 
// void thread_function(double* posTargetPtr, double timeDuration){
//   PandaController controller("172.16.0.2",posTargetPtr);
//   controller.moveToStart(0.366,0,0.41);
//   sleep(1);
//   controller.runController(timeDuration);
// }