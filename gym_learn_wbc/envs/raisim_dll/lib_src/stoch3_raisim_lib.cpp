/*
Shared library implementing functions
to be called from python ie gym env
for simulation 
*/

#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include "stoch3_test.cpp"
using Vector3d = Eigen::Matrix<double,3,1>;
raisim::World world;
//enum ControlMode = {PD_PLUS_FEEDFORWARD_TORQUE,FORCE_AND_TORQUE}
auto ground = world.addGround();
auto stoch = world.addArticulatedSystem(raisim::loadResource("Stoch3/urdf/stoch3_urdf.urdf"));
void setupCallback() {
  auto vis = raisim::OgreVis::get();

  /// light
  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(-3, -3, -0.5);
  lightdir.normalise();
  vis->getLightNode()->setDirection({lightdir});

  /// load  textures
  vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
  vis->loadMaterialFile("checkerboard.material");

  /// shdow setting
  vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  vis->getSceneManager()->setShadowTextureSettings(2048, 3);

  /// scale related settings!! Please adapt it depending on your map size
  // beyond this distance, shadow disappears
  vis->getSceneManager()->setShadowFarDistance(30);
  // size of contact points and contact forces
  vis->setContactVisObjectSize(0.06, .6);
  // speed of camera motion in freelook mode
  vis->getCameraMan()->setTopSpeed(5);
}

extern "C"{
void _init_ViSsetup(bool gravity)
{


 if(!gravity)
 world.setGravity({0,0,0}); // by default gravity is set to {0,0,g}
 world.setTimeStep(0.0025);

  auto vis = raisim::OgreVis::get();

  /// these method must be called before initApp
  vis->setWorld(&world);
  vis->setWindowSize(2600, 1200);
  vis->setImguiSetupCallback(imguiSetupCallback);
  vis->setImguiRenderCallback(imguiRenderCallBack);
  vis->setKeyboardCallback(raisimKeyboardCallback);
  vis->setSetUpCallback(setupCallback);
  vis->setAntiAliasing(2);
  vis->setDesiredFPS(25);

  //simulation is automatically stepped, if is false
  raisim::gui::manualStepping = true; 
  //raisim::gui::Collisionbodies = true; 
  /// starts visualizer thread
  vis->initApp();

  
  auto groundVis = vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");
  auto stochVis = vis->createGraphicalObject(stoch, "stoch");

  vis->select(groundVis->at(0));
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4), 2);
  auto names = stoch->getMovableJointNames();


}
}

extern "C"{
void _init_stoch(float base_initial_pos[3],int ControlMode)
{

  stoch->setGeneralizedCoordinate({     base_initial_pos[0], base_initial_pos[1], base_initial_pos[2], //base co ordinates 
                                        1, 0, 0, 0,  //orientation 
                                        0.0, 0.0, 0.0, //leg 1
                                        0.0, 0.0, 0.0, //leg 2
                                        0.0, 0.0, 0.0, //leg 3
                                        0.0, 0.0, 0.0}); //leg 4


Eigen::VectorXd jointPgain(stoch->getDOF()), 
				jointDgain(stoch->getDOF()),
				jointVelocityTarget(stoch->getDOF());
jointPgain.setZero();
jointDgain.setZero();
jointVelocityTarget.setZero();
  
 // P and D gains for the leg actuators alone
jointPgain.tail(12).setConstant(400.0);
jointDgain.tail(12).setConstant(50.0);

  stoch->setGeneralizedForce(Eigen::VectorXd::Zero(stoch->getDOF()));
 if(ControlMode == 0)
  stoch->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
 else
  stoch->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

 stoch->setPdGains(jointPgain, jointDgain);
  stoch->setName("stoch");
  

  stoch->printOutBodyNamesInOrder();
}}



void ToEulerAngles(double euler[3],double q[4]) {
   // roll (x-axis rotation)
    double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (std::abs(sinp) >= 1)
        euler[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);

    
}


extern "C"{
void _sim(double*state,long int no_of_steps,double omega,double radius,bool render,float velocity_target[2])
{
auto vis = raisim::OgreVis::get();

double time = 0;
  float realTimeFactor_ = 1.0;
  /// inverse of visualization frequency
  int desiredFPS = 25;
  double visTime = 1. / desiredFPS;
 // double omega = -3;
 // double radius = 0.06;
  double ctime = 0;
  long int step_no = 0;
  //while (!vis->getRoot()->endRenderingQueued()) 
double avg_vel_values[10] = {
                             0.0,0.0,    // avg vx,vy
                             0.0,0.0,    // avg evx,evy
                             0.0,0.0,0.0,// avg rpy
                             0.0,0.0,0.0 // avg rdot,pdot,ydot
                            } ;
while(step_no <no_of_steps)
  {
    step_no ++ ;

    //std::cout<<"\nStep_no:"<<step_no<<std::endl;
    int takeNSteps_ = desiredFPS;
    bool paused_ = vis->getPaused();
    while (time < 0 && (!(takeNSteps_ == 0 && paused_) || !paused_)) 
    {
      trot_test(stoch, omega, radius, ctime);
      world.integrate1(); 
      world.integrate2();
      time += world.getTimeStep();
      ctime = world.getWorldTime();
      
      /// count the allowed number of sim steps
      if (takeNSteps_ > 0) takeNSteps_--;
    }
      raisim::VecDyn GenCo(stoch->getGeneralizedCoordinate());
      raisim::VecDyn GenV(stoch->getGeneralizedVelocity());
      raisim::VecDyn GenT(stoch->getGeneralizedForce());
      //std::cout <<"\n\nGenCo:\n"<<GenCo;
      //std::cout <<"\nGenV:\n"<<GenV;
      //std::cout <<"\nGenT:\n"<<GenT<<"\n";

      //running avg values:

      //avg vx and vy
      avg_vel_values [0] = ((step_no - 1)*avg_vel_values[0] + GenV.v[0])/step_no;
      avg_vel_values [1] = ((step_no - 1)*avg_vel_values[1] + GenV.v[1])/step_no;

      //avg of error in vx and vy wrt to target
      float error_vx = GenV.v[0] - velocity_target [0];
      float error_vy = GenV.v[1] - velocity_target [1];
      avg_vel_values [2] = ((step_no - 1)*avg_vel_values[2] + error_vx)/step_no;
      avg_vel_values [3] = ((step_no - 1)*avg_vel_values[3] + error_vy)/step_no;
      
      //avg of rpy and ang veloicities
      double euler[3];
      double q[4];// = {0.7073883,0.7068252, 0, 0};
      /*
      for(int i=0;i<4;i++)
        q[i] = GenCo.v[3+i];
      */
      ToEulerAngles(euler,q);

      for(int i=0;i<3;i++)
      {
        
      avg_vel_values [4+i] = ((step_no - 1)*avg_vel_values[4+i] + euler[i])/step_no;
      avg_vel_values [7+i] = ((step_no - 1)*avg_vel_values[7+i] + GenV.v[3+i])/step_no;
      }


    /// compute how much sim is ahead
    if (time > -visTime * realTimeFactor_ * .1)
      time -= visTime * realTimeFactor_;

    /// do actual rendering
    if(render)
    vis->renderOneFrame();
  }

//std::cout<<"\nAvg_vel_values:\n";

for(int i=0;i<10;i++)
{  //std::cout<<avg_vel_values[i]<<"\t";
  *(state+i) = avg_vel_values[i];}

}}

extern "C"{
void _reset(float base_initial_pos[3])
{

  stoch->setGeneralizedCoordinate({     base_initial_pos[0], base_initial_pos[1], base_initial_pos[2], //base co ordinates 
                                        1, 0, 0, 0,  //orientation 
                                        0.0, 0.0, 0.0, //leg 1
                                        0.0, 0.0, 0.0, //leg 2
                                        0.0, 0.0, 0.0, //leg 3
                                        0.0, 0.0, 0.0}); //leg 4
  stoch->setGeneralizedForce(Eigen::VectorXd::Zero(stoch->getDOF()));
  // world.integrate1();
  // world.integrate2();
}}

extern "C"{
  
void _close()

{
  auto vis = raisim::OgreVis::get();
  vis->closeApp();

}

}