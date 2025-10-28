#include "PandaController.h"
#include <thread>
#include <chrono>

void thread_function(PandaController*, double*, double orienTargetPtr[][3], double);
void parseMessage(char receiveBuffer[12*8 + 1], double * posTarget, double orienTarget[3][3], char* mode);

int main(int argc, char** argv) {
  double posTarget[3] = {0.34, 0.0, 0.6};
  double orienTargetPtr[3][3] = {{1,0,0},{0,-1,0},{0,0,-1}};
  double currState[] = {0,0,0,0,0,0,0};
  char mode = 1;
  char receiveBuffer[12*8 + 1];
  double timeDuration = 1000;
  PandaController * controller = new PandaController("172.16.0.2",mode,posTarget, orienTargetPtr,currState); 
  controller->moveToStart(0.25, -0.15, 0.45);
  controller->pivot();
  controller->moveToStart(0.25, -0.15, 0.45);
  controller->RotateAxis(0);
  controller->RotateAxis(1);
  controller->RotateAxis(2);
  // controller->RotateJoint(4);
  // controller->RotateJoint(5);
  // controller->RotateJoint(6);
  }