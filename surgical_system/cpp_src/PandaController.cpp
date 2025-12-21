#include "PandaController.h" 

PandaController::PandaController(std::string ipAddress,char mode, double *posTargetPtr, double orienTargetPtr[][3], double currState[13]){
  this->ipAddress = ipAddress;
  this->posTargetPtr = posTargetPtr;
  this->orienTargetPtr = orienTargetPtr;
  this->currState = currState;
  this->robot = new franka::Robot(this->ipAddress, franka::RealtimeConfig::kEnforce, 50UL);
  this->setMode(mode);

  double mass    = 0.8;                       // [kg]
  std::array<double,3> com = {0.0, 0.0, 0.03}; // [m]
  std::array<double,9> inertia = {
    0.00001, 0.0,     0.0,
    0.0,     0.00001, 0.0,
    0.0,     0.0,     0.00001}; // guess               
  robot->setLoad(mass, com, inertia); //
}

void PandaController::setBehavior(){
  // Set additional parameters always before the control loop, NEVER in the control loop!
  // Set the joint impedance.
  this->robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  // Set the collision behavior
  std::array<double, 7> lower_torque_thresholds_nominal{
      {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
  std::array<double, 7> upper_torque_thresholds_nominal{
      {37.0, 37.0, 33.0, 30.0, 30.0, 28.0, 25.0}};
  std::array<double, 7> lower_torque_thresholds_acceleration{
      {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
  std::array<double, 7> upper_torque_thresholds_acceleration{
      {37.0, 37.0, 33.0, 30.0, 30.0, 28.0, 25.0}};
  std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
  std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
  std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
  std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
  this->robot->setCollisionBehavior(
      lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
      lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
      lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
      lower_force_thresholds_nominal, upper_force_thresholds_nominal);
  return;
}

void PandaController::moveToStart(double x, double y, double z){
  if (mode <= 0)
  {
    return; // if we aren't in control mode then don't control the robot->
  }
  // // only a motion generator
  // // First move the robot to a suitable joint configuration
  double joint_array[7]; 
  double pos_array[3] = {x, y, z};
  PandaController::geomIkin(pos_array, joint_array);
  // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, 0}};
  std::array<double, 7> q_goal = {{joint_array[0], joint_array[1], joint_array[2], joint_array[3], \
                                  joint_array[4], joint_array[5], joint_array[6]}};
  MotionGenerator motion_generator(0.5, q_goal);
  std::cout << "Going to First Position\n";
  this->robot->control(motion_generator);
  return;
}

void PandaController::runController(double maxTime /*=10*/){
  // Eigen::Vector3d theFirst = Eigen::Vector3d::Zero();
  
  while (mode >= 0)
  { // while we haven't received the stop command
    if (mode == 0)
    {
      this->getPose();
    } 
    else if (mode == 1){
      franka::Model model = this->robot->loadModel();
      modelPtr = &model;
      setDefaultBehavior(*robot);
      this->setBehavior();
      std::cout << "Running Controller" << std::endl;
      double time = 0;
      this->robot->control([=,&time](const franka::RobotState& robotState,
                                    franka::Duration timeStep) -> franka::CartesianVelocities {
        
        //LINEAR VELOCITY VARIABLES 
        static Eigen::Vector3d linVel = {0.0,0.0,0.0};
        Eigen::Vector3d pastLinVel = linVel;
        double maxVel = 0.30; // [m/s]
        double maxAccel = 2.4/1000.0; // [m/s per ms]
        Eigen::Vector3d posTarget(this->posTargetPtr[0], this->posTargetPtr[1], this->posTargetPtr[2]);
        time += timeStep.toSec();  // Update time at the beginning of the callback.
        // based on the current height, and desired height, change robot velocity.
        this->convertPose(robotState.O_T_EE.data(), this->currState);
        Eigen::Map<const Eigen::Matrix<double,4,4>> O_T_EE(robotState.O_T_EE.data());
        Eigen::Vector3d currentPos(O_T_EE(0,3),O_T_EE(1,3),O_T_EE(2,3));
        Eigen::Vector3d changeInPos;

        //ANGULAR VELOCITY VARIABLES
        static Eigen::Vector3d angVel(0.0,0.0,0.0);
        Eigen::Vector3d omega;
        Eigen::Matrix<double,3,3> orienTarget;
        orienTarget << this->orienTargetPtr[0][0], this->orienTargetPtr[0][1], this->orienTargetPtr[0][2],
                       this->orienTargetPtr[1][0], this->orienTargetPtr[1][1], this->orienTargetPtr[1][2],
                       this->orienTargetPtr[2][0], this->orienTargetPtr[2][1], this->orienTargetPtr[2][2];
        Eigen::Vector3d changeInOrien(0.0,0.0,0.0);
        double maxAngVel = 1.0; //[rad/sec] 
        double maxAngAccel = 2.5/1000.0; //[rad/s^2]*[s] essentially how large of a step we can take in our velocity
        double theta = this->getRotationVelocity(robotState,orienTarget,omega);

        // Calculate if joint acceleration is too high
        // Jacobian for preventing excessive joint velocities
        std::array<double, 42> jacobian_array = modelPtr->zeroJacobian(franka::Frame::kFlange, robotState);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

        // We can get the robot's current joint velocities targets
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> jointVel_d(robotState.dq_d.data());
        Eigen::Matrix<double, 6, 1> cartVel_d; // panda desired cartesian velocity
        // current cartesian velocity based on jacobian
        cartVel_d = jacobian*jointVel_d;

        // set current cartesian linear and angular acceleration based on desired joint velocities
        linVel(0) = cartVel_d(0); linVel(1) = cartVel_d(1); linVel(2) = cartVel_d(2); 
        angVel(0) = cartVel_d(3); angVel(1) = cartVel_d(4); angVel(2) = cartVel_d(5);

        this->currState[7] = cartVel_d(0);
        this->currState[8] = cartVel_d(1);
        this->currState[9] = cartVel_d(2);
        this->currState[10] = cartVel_d(3);
        this->currState[11] = cartVel_d(4);
        this->currState[12] = cartVel_d(5);
        
        // calculate necessary change in position (aka velocity vector)
        changeInPos = posTarget - currentPos;

        // if (std::abs(changeInPos.x()) < 1e-4) changeInPos.x() = 1e-7;
        // if (std::abs(changeInPos.y()) < 1e-4) changeInPos.y() = 1e-7;
        // if (std::abs(changeInPos.z()) < 1e-4) changeInPos.z() = 1e-7;

        // std::cout << "Current Position: " << currentPos.transpose() << std::endl;
        // std::cout << "Target Position: " << posTarget.transpose() << std::endl;
        // std::cout << "Change in Position: " << changeInPos.transpose() << std::endl;

        // calculate necessary change in orientation (aka angular velocity vector)
        changeInOrien = theta*omega;

        Eigen::Vector3d desiredLinAccel;
        Eigen::Vector3d desiredAngAccel;
        // use our necessary change in position and current velocity to determine new velocity setpoint
        setAcceleration(linVel,changeInPos,desiredLinAccel,maxVel,maxAccel);
        // use our desired change in rotation about the desired axis to determine new ang velocity setpoint
        setAcceleration(angVel,changeInOrien,desiredAngAccel,maxAngVel,maxAngAccel);
        // update our linear and angular velocities
        linVel += desiredLinAccel;
        angVel += desiredAngAccel;
 
        franka::CartesianVelocities output = {{linVel(0), linVel(1), linVel(2), angVel(0), angVel(1), angVel(2)}};
        
        //debug statements
        // double printFrequency = 100; //[Hz]
        // if (abs(round(time * printFrequency) - time*printFrequency) < 0.0005){     // only print every second   
        //   std::cout << "\nC++ Debug at time: " << time  << " seconds" << std::endl;
        //   // std::cout << "Current Joint Velocities: " << jointVel.transpose() << std::endl;
        //   // std::cout << "Desired Joint Velocities: " << jointVel_d.transpose() << std::endl;
        //   // std::cout << "Commanded Joint Velocities: " << jointVel_c.transpose() << std::endl;
        //   std::cout << "Current Cartesian Velocity: " << linVel << std::endl;
        //   std::cout << "Desired Cartesian Velocities: " << cartVel_d.transpose() << std::endl;
        //   // std::cout << "Commanded Cartesian Velocity: " << cartVel_c.transpose() << std::endl;
        //   // std::cout << "Desired Linear Acceleration: " << desiredLinAccel.transpose() << std::endl;
        //   // std::cout << "Orientation Target: \n" << orienTarget << std::endl;
        //   // std::cout << "Full transform: \n" << O_T_EE << std::endl;
        // //   for (int i = 0; i < 3; i ++){
        // //     std::cout << orienTarget[i][0] <<" "<< orienTarget[i][1] <<" "<< orienTarget[i][2] << std::endl;
        // //   }
        // //   std::cout << "\n";
        // //   for (int i = 0; i < 3; i ++){
        // //     std::cout << robotState.O_T_EE.data()[i] <<" "<< robotState.O_T_EE.data()[i+4] <<" "<< robotState.O_T_EE.data()[i + 8] << std::endl;
        // //   }
        // } 

        if ((time >= maxTime) || (this->mode != 1)) 
        {
          // Return MotionFinished at the end of the trajectory.
          std::cout << "Control Finished" << std::endl;
          return franka::MotionFinished(output);
        }
        return output;
      });
    }
    else if (mode == 2){
      this->velocityMode(maxTime);
    }
  }
}


void PandaController::velocityMode(double maxTime){

  std::cout << "Velocity Controller: " << std::endl;
  double maxVel = 0.30; // [m/s]
  double maxAccel = 1.0/1000.0; // [m/s per ms]
  double maxAngVel = 1.0; //[rad/sec] 
  double maxAngAccel = 2.5/1000.0; //[rad/s^2]*[s] essentially how large of a step we can take in our velocity

  auto clampNorm = [](const Eigen::Vector3d& v, double max_n) {
    double n = v.norm();
    if (n <= max_n) return v;
    return (v * (max_n / n)).eval();
  };

  auto clamp = [](double x, double hi) {
    return std::min(hi, x);
  };

  franka::Model model = this->robot->loadModel();
  modelPtr = &model;
  setDefaultBehavior(*robot);
  this->setBehavior();
  std::cout << "Running Controller" << std::endl;
  double time = 0;

  this->robot->control([=,&time](const franka::RobotState& robotState,
                                franka::Duration timeStep) -> franka::CartesianVelocities {
    
    double dt = timeStep.toSec();  // Update time at the beginning of the callback.
    time += dt; // sec

    Eigen::Vector3d linVelTarget(this->posTargetPtr[0], this->posTargetPtr[1], this->posTargetPtr[2]);
    Eigen::Vector3d omegaVelTarget(this->orienTargetPtr[0][0], this->orienTargetPtr[0][1], this->orienTargetPtr[0][2]);

    linVelTarget = clampNorm(linVelTarget, maxVel);
    omegaVelTarget = clampNorm(omegaVelTarget, maxAngVel);   
    
    
    // // Calculate if joint acceleration is too high
    // // Jacobian for preventing excessive joint velocities
    std::array<double, 42> jacobian_array = modelPtr->zeroJacobian(franka::Frame::kFlange, robotState);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    // // We can get the robot's current joint velocities targets
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> jointVel_d(robotState.dq_d.data());
    Eigen::Matrix<double, 6, 1> cartVel_d; // panda desired cartesian velocity
    // // current cartesian velocity based on jacobian
    cartVel_d = jacobian*jointVel_d;

    static Eigen::Vector3d currentCartVel = Eigen::Vector3d::Zero();
    static Eigen::Vector3d currentOmegaVel = Eigen::Vector3d::Zero();

    currentCartVel(0) = cartVel_d(0); currentCartVel(1) = cartVel_d(1); currentCartVel(2) = cartVel_d(2); 
    currentOmegaVel(0) = cartVel_d(3); currentOmegaVel(1) = cartVel_d(4); currentOmegaVel(2) = cartVel_d(5);

    this->convertPose(robotState.O_T_EE.data(), this->currState);

    this->currState[7] = cartVel_d(0);
    this->currState[8] = cartVel_d(1);
    this->currState[9] = cartVel_d(2);
    this->currState[10] = cartVel_d(3);
    this->currState[11] = cartVel_d(4);
    this->currState[12] = cartVel_d(5);
    
    Eigen::Vector3d linVelOutput = Eigen::Vector3d::Zero();
    Eigen::Vector3d omegaVelOutput  = Eigen::Vector3d::Zero();

    // Eigen::Vector3d omega;
    // double theta = this->getRotationVelocity(robotState,orienTarget,omega);

    Eigen::Vector3d desiredLinAccel = Eigen::Vector3d::Zero();
    Eigen::Vector3d desiredAngAccel = Eigen::Vector3d::Zero();
    // use our necessary change in position and current velocity to determine new velocity setpoint
    setAcceleration(currentCartVel,linVelTarget,desiredLinAccel,maxVel,maxAccel);
    setAcceleration(currentOmegaVel,omegaVelTarget,desiredAngAccel,maxAngVel,maxAngAccel);
    // use our desired change in rotation about the desired axis to determine new ang velocity setpoint
    // if(sqrt(omegaVelTarget.transpose()*omegaVelTarget) > 0.0000001){
    // }

    // update our linear and angular velocities
    linVelOutput = currentCartVel + desiredLinAccel;
    omegaVelOutput = currentOmegaVel + desiredAngAccel;

    // for(int i = 0; i < 3; i++){
    //   linVelOutput(i) = clamp(linVelOutput(i), linVelTarget(i));
    //   omegaVelOutput(i) = clamp(omegaVelOutput(i), omegaVelTarget(i)); 
    // }

      
    
    franka::CartesianVelocities output = {{linVelOutput(0), linVelOutput(1), linVelOutput(2), omegaVelOutput(0), omegaVelOutput(1), omegaVelOutput(2)}};
    // franka::CartesianVelocities output = {{0, 0, 0, 0, 0, 0}};
    
    
    //debug statements
    double printFrequency = 100; //[Hz]
    if (abs(round(time * printFrequency) - time*printFrequency) < 0.0005){     // only print every second   
      // std::cout << "\nC++ Debug at time: " << time  << " seconds" << std::endl;
      // std::cout << "Current Cartesian Velocity: " << currentCartVel.transpose() << std::endl;
      // std::cout << "Current target Velocity: " << linVelOutput.transpose() << std::endl;
      // std::cout << "Desired Cartesian Velocities: " << linVelTarget.transpose() << std::endl << std::endl;

      // std::cout << "Current Omega Velocity: " << currentOmegaVel.transpose() << std::endl;
      // std::cout << "Current target Velocity: " << omegaVelOutput.transpose() << std::endl;
      // std::cout << "Desired Omega Velocities: " << omegaVelTarget.transpose() << std::endl;
      // std::cout << "Desired Linear Acceleration: " << desiredLinAccel.transpose() << std::endl;
      // std::cout << "Orientation Target: \n" << orienTarget << std::endl;
      // std::cout << "Commanded Cartesian Velocity: " << cartVel_c.transpose() << std::endl;
      // std::cout << "Full transform: \n" << O_T_EE << std::endl;
    //   for (int i = 0; i < 3; i ++){
    //     std::cout << orienTarget[i][0] <<" "<< orienTarget[i][1] <<" "<< orienTarget[i][2] << std::endl;
    //   }
    //   std::cout << "\n";
    //   for (int i = 0; i < 3; i ++){
    //     std::cout << robotState.O_T_EE.data()[i] <<" "<< robotState.O_T_EE.data()[i+4] <<" "<< robotState.O_T_EE.data()[i + 8] << std::endl;
    //   }
    } 

    if ((time >= maxTime) || (this->mode != 2)) 
    {
      // Return MotionFinished at the end of the trajectory.
      std::cout << "Control Finished" << std::endl;
      return franka::MotionFinished(output);
    }
    return output;
  });
}


void PandaController::getPose(){
  /*
  Returns the robot state as an array.
  The first 3 elements are the robots position
  The final 4 elements are the robots orientation as a quaternion
  */
  franka::RobotState robotState = this->robot->readOnce();
  const double * T = robotState.O_T_EE.data();
  this->convertPose(T,this->currState);
}

void PandaController::convertPose(const double T[16], double currState[13]){
  currState[0] = T[12]; currState[1] = T[13]; currState[2] = T[14];
  // for (int j = 0; j < 4; j++){
  //   std::cout << "[";
  //   for (int i=0; i < 4; i++){
  //     std::cout << " " << T[i*4 + j];
  //   }
  //   std::cout << "]" << std::endl;
  // }
  Eigen::Matrix3d rotationMatrix;
  rotationMatrix << T[0], T[4], T[8], T[1], T[5], T[9], T[2], T[6], T[10];
  // std::cout << rotationMatrix << std::endl;
  Eigen::Quaterniond q(rotationMatrix);
  // std::cout << q.vec() << std::endl;
  // std::cout << q.w() << std::endl;
  currState[3] = q.vec()[0]; currState[4] = q.vec()[1]; currState[5] = q.vec()[2]; currState[6] = q.w();
}

void PandaController::geomIkin(double* pos, double* joint_array){
  /*
  pos should be a pointer to a double[3] and joint_array should be a pointer to a double[7]
  */
  double xd = sqrt(pow(pos[0],2) + pow(pos[1],2));
  double zd = pos[2];
  double l1 = 0.3330;
  double l2 = 0.3160;
  double l3 = 0.0825;
  double l4 = 0.3840;
  double l5 = 0.1070;
  double l6 = 0.0880;

  double xd_p = xd - l6;
  double zd_p = zd + l5;
  double d1 = sqrt(pow(l2,2) + pow(l3,2));
  double d2 = sqrt(pow(l3,2) + pow(l4,2));
  double d3 = sqrt(pow(xd_p,2) + pow((zd_p - l1),2));
  double alpha_3 = acos((pow(d2,2) + pow(d1,2) - pow(d3,2))/(2*d1*d2));
  double beta_1 = acos((pow(l3,2) + pow(d2,2) - pow(l4,2))/(2*l3*d2));
  double beta_2 = acos((pow(l3,2) + pow(d1,2) - pow(l2,2)) / (2*l3*d1));
  double theta4 = -(2*M_PI - (beta_1 + beta_2 + alpha_3));

  double alpha_2 = acos((pow(d1,2) + pow(d3,2) - pow(d2,2))/(2*d1*d3));
  double gamma_1 = atan2(zd_p - l1, xd_p);
  double gamma_2 = acos((pow(l2,2) + pow(d1,2) - pow(l3,2))/(2*l2*d1));
  double theta2 = -(gamma_2 + alpha_2 - (M_PI/2 - gamma_1));

  double theta6 = theta2 - theta4;

  joint_array[0] = atan2(pos[1], pos[0]);
  joint_array[1] = theta2;
  joint_array[2] = 0;
  joint_array[3] = theta4;
  joint_array[4] = 0;
  joint_array[5] = theta6;
  joint_array[6] = -M_PI_2; // This is not 0 to make it easier to attach the laser fiber
}

double PandaController::getRotationVelocity(const franka::RobotState& robotState,const Eigen::Matrix<double,3,3> &finalRotationMat, Eigen::Matrix<double,3,1> &omega){
  /* robotState comes from libfranka
     finalRotation is a [3][3] double with the final rotation we want to achieve.
  */

  Eigen::Matrix3d invCurrentRotation;
  invCurrentRotation << robotState.O_T_EE.data()[0], robotState.O_T_EE.data()[1], robotState.O_T_EE.data()[2],
                        robotState.O_T_EE.data()[4], robotState.O_T_EE.data()[5], robotState.O_T_EE.data()[6],
                        robotState.O_T_EE.data()[8], robotState.O_T_EE.data()[9], robotState.O_T_EE.data()[10];

  Eigen::Matrix3d currentRotation = invCurrentRotation.transpose();

  Eigen::Matrix3d desiredRotation = invCurrentRotation * finalRotationMat;
  Eigen::AngleAxisd angleAxis(desiredRotation);

  bool flag = false;
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      if(finalRotationMat(i,j) == NAN){
        std::cout << "NAN Value in finalRotation" << std::endl;
        angleAxis.angle() = 0;
        flag = true;
        break;
      }
    }
    if(flag){
      break;
    }
  }
 
  double theta = angleAxis.angle();

  // std::cout << "Theta: " << theta << std::endl;

  // Convert the axis back into the world reference frame
  omega = currentRotation * angleAxis.axis();
  // std::cout << "Omega x: " << omega[0] << std::endl;
  // std::cout << "Omega y: " << omega[1] << std::endl;
  // std::cout << "Omega z: " << omega[2] << std::endl;
  // std::cout << "\n";
  return theta;
}

void PandaController::multiplyMat(double arr1[3][3], double arr2[3][3], double arr3[3][3]){
  /* Multiplies two 3x3 matrices and stores result in arr3
  */
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            arr3[i][j] = 0;
            for(int k = 0; k < 3; k++){
                arr3[i][j] += arr1[i][k] * arr2[k][j];
            }
        }
    }
}

void PandaController::setAcceleration(Eigen::Vector3d & currVel, Eigen::Vector3d & desVel, Eigen::Vector3d & outputAccel, double maxVel, double maxAccel){
  Eigen::Vector3d velDir_d = {0.0,0.0,0.0};
  double velMag_d = sqrt(desVel.transpose()*desVel);
  if (velMag_d > 0.00001) {  // if magnitude of the desired velocity is 0, we can't get a proper direction.
    velDir_d = desVel/velMag_d;
  }

  if (velMag_d > maxVel){ // cap our velocity we want to hit
    velMag_d = maxVel;
    // std::cout << "Bigger " << std::endl;
  } 
  else if (this->mode == 1) {
    // scale the velocity by a sine wave. Makes it converge faster;
    // sin(pi/2*x) > x for all values where 0:x:1. In our case, x is a percentage of the maximum speed we want to go.
    velMag_d = maxVel*sin(M_PI_2*velMag_d/maxVel); 
  }

  // This is the actual desired velocity we want after we have scaled the magnitude
  Eigen::Vector3d vel_d = velMag_d*velDir_d;
  // based on our current velocity and desired velocity, determine how we need to accelerate
  Eigen::Vector3d accel_d = vel_d - currVel;

  // std::cout << "Accel D" << accel_d << std::endl;

  // Get acceleration direction and magnitude
  double accelMag = sqrt(accel_d.transpose()*accel_d);
  if(accelMag < 0.0001){
    return;
  }
  Eigen::Vector3d accelDir = accel_d/accelMag;
  // set maximum acceleration
  if (accelMag > maxAccel){
    accelMag = maxAccel;
  }
  // Set desired acceleration based on maximum acceleration  
  outputAccel = accelDir*accelMag;
  // std::cout << "Velocity: " << velMag_d << " Acceleration: " << accelMag << std::endl;
  return;
}

void PandaController::setMode(int mode){
  this->mode = mode;
}

void PandaController::pivot(){
  setDefaultBehavior(*robot);
  this->setBehavior();
  std::cout << "Running Controller" << std::endl;
  double time = 0;
  double maxTime = 20;
  double numLoops = 4;
  this->robot->control([=,&time](const franka::RobotState& robotState,
                                    franka::Duration timeStep) -> franka::CartesianVelocities {
    time += timeStep.toSec();
    double static angVel[3] = {0.0,0.0,0.0};
    
    double tau = maxTime/numLoops;
    angVel[0] = 0.2*((2*M_PI/tau)*(-cos(time*(2*M_PI)/(maxTime))+ 1)*cos(time*(2*M_PI)/tau) + (2*M_PI/(maxTime))*(-sin(time*(2*M_PI)/(2*maxTime)))*sin(time*2*M_PI/tau));
    angVel[1] = 0.2*((2*M_PI/tau)*(-cos(time*(2*M_PI)/(maxTime)) + 1)*sin(time*(2*M_PI)/tau) + (2*M_PI/(maxTime))*(-sin(time*(2*M_PI)/(maxTime)))*cos(time*2*M_PI/tau));
    angVel[2] = 0.0;

    franka::CartesianVelocities output = {{0.0,0.0,0.0, angVel[0], angVel[1], angVel[2]}};
      if ((time >= maxTime)) 
      {
        // Return MotionFinished at the end of the trajectory.
        std::cout << "Control Finished" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
  });
}

void PandaController::RotateAxis(int axis){
  setDefaultBehavior(*robot);
  this->setBehavior();
  std::cout << "Running Controller" << std::endl;
  double time = 0;
  double maxTime = 20;
  double numLoops = 4;
  this->robot->control([=,&time](const franka::RobotState& robotState,
                                    franka::Duration timeStep) -> franka::CartesianVelocities {
    time += timeStep.toSec();
    double static angVel[3] = {0.0,0.0,0.0};
    
    double tau = maxTime/numLoops;
    angVel[axis] = 0.2*((2*M_PI/tau)*(-cos(time*(2*M_PI)/(maxTime))+ 1)*cos(time*(2*M_PI)/tau) + (2*M_PI/(maxTime))*(-sin(time*(2*M_PI)/(2*maxTime)))*sin(time*2*M_PI/tau));

    franka::CartesianVelocities output = {{0.0,0.0,0.0, angVel[0], angVel[1], angVel[2]}};
      if ((time >= maxTime)) 
      {
        // Return MotionFinished at the end of the trajectory.
        std::cout << "Control Finished" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
  });
}

void PandaController::RotateJoint(int joint){
  setDefaultBehavior(*robot);
  this->setBehavior();
  std::cout << "Running Controller" << std::endl;
  double time = 0;
  double maxTime = 20;
  double numLoops = 4;
  this->robot->control([=,&time](const franka::RobotState& robotState,
                                    franka::Duration timeStep) -> franka::JointVelocities {
    time += timeStep.toSec();
    double static jointVel[7] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    
    double tau = maxTime/numLoops;
    jointVel[joint] = 0.2*((2*M_PI/tau)*(-cos(time*(2*M_PI)/(maxTime))+ 1)*cos(time*(2*M_PI)/tau) + (2*M_PI/(maxTime))*(-sin(time*(2*M_PI)/(2*maxTime)))*sin(time*2*M_PI/tau));

    franka::JointVelocities output = {{jointVel[0], jointVel[1], jointVel[2], jointVel[3], jointVel[4], jointVel[5], jointVel[6]}};
      if ((time >= maxTime)) 
      {
        // Return MotionFinished at the end of the trajectory.
        std::cout << "Control Finished" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
  });
}