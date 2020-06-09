//
// Created by Jemin Hwangbo on 10/15/27.
// MIT License
//
// Copyright (c) 2027-2027 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include <fstream>
#include <iostream>
#include <string>

// #include "leg.cpp"
#include "stoch3_test.cpp"
using Vector3d = Eigen::Matrix<double,3,1>;
// #include <OgreApplicationContext.h>
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

int main(int argc, char **argv) 
{
  /// create raisim world
  raisim::World world;
  
  //world.setGravity({0,0,0}); // by default gravity is set to {0,0,g}
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

  /// starts visualizer thread
  vis->initApp();

  /// create raisim objects
  auto ground = world.addGround();
  

  /// create visualizer objects
  vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");

  /// stoch joint PD controller
  /*jointNominalConfig
   first 3 - base co-ordinates
   next 4 - base orientation
   last 20 - 5 joints per leg*/

  Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointForce(18), jointPgain(18), jointDgain(18);
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  
  //P and D gains for the leg actuators alone
  jointPgain.tail(12).setConstant(400.0);
  jointDgain.tail(12).setConstant(50.0);

  
  //no of steps per call
  const size_t N = 100;

  auto stoch = world.addArticulatedSystem(raisim::loadResource("Stoch3/urdf/stoch3_urdf.urdf"));
  auto stochVis = vis->createGraphicalObject(stoch, "stoch");
  auto names = stoch->getMovableJointNames();
  // for(auto x:names)
  // {
  //   std::cout<<x<<std::endl;
  // }
  // std::cout<<stoch->getDOF()<<std::endl;
  stoch->setGeneralizedCoordinate({     0, 0, 0.6, //base co ordinates 
                                        1, 0, 0, 0,  //orientation 
                                        0.0, 0.0, 0.0, //leg 1
                                        0.0, 0.0, 0.0, //leg 2
                                        0.0, 0.0, 0.0, //leg 3
                                        0.0, 0.0, 0.0}); //leg 4

  stoch->setGeneralizedForce(Eigen::VectorXd::Zero(stoch->getDOF()));
  stoch->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  stoch->setPdGains(jointPgain, jointDgain);
  //stoch->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
  stoch->setName("stoch");
  
//   //to take random samples
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.25);
  std::srand(std::time(nullptr));
  stoch->printOutFrameNamesInOrder();
  /// set camera
  vis->select(stochVis->at(0));
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0.), Ogre::Radian(-1.), 3);

  /// run the app
  double time = 0;
  float realTimeFactor_ = 1.0;
  /// inverse of visualization frequency
  int desiredFPS = 25;
  double visTime = 1. / desiredFPS;
  double omega = -3;
  double radius = 0.06;
  double ctime = 0;
  while (!vis->getRoot()->endRenderingQueued()) 
  {
    // std::cout<<"time: "<<time<<std::endl;
    int takeNSteps_ = desiredFPS;
    bool paused_ = vis->getPaused();
    while (time < 0 && (!(takeNSteps_ == 0 && paused_) || !paused_)) 
    {

      // control functions
      // Vector3d torque_FL;
      // Vector3d force_FL(0.001, 0.00, 0.00);
      // Vector3d theta_FL;
      // auto thetas = stoch->getGeneralizedCoordinate();
      // theta_FL[0]=thetas[7];
      // theta_FL[1]=thetas[8];
      // theta_FL[2]=thetas[9];
      // Eigen::VectorXd torques(18);
      // torques.setZero();
      // inverse_dynamics_FL(torque_FL, force_FL, theta_FL);
      // torques[6]=torque_FL[0];
      // torques[7]=torque_FL[1];
      // torques[8]=torque_FL[2];
      // // std::cout<<torques[7]<<","<<torques[8]<<","<<torques[9]<<std::endl;
      // stoch->setGeneralizedForce(torques);

      //Simulation functions
      trot_test(stoch, omega, radius, ctime);
      world.integrate1(); 
      world.integrate2();
      time += world.getTimeStep();
      ctime = world.getWorldTime();
      /// count the allowed number of sim steps
      if (takeNSteps_ > 0) takeNSteps_--;
    }

    /// compute how much sim is ahead
    if (time > -visTime * realTimeFactor_ * .1)
      time -= visTime * realTimeFactor_;

    /// do actual rendering
    vis->renderOneFrame();
  }

  vis->closeApp();
  /// terminate

  return 0;
}
