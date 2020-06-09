#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include "leg.cpp"
#include "iostream"
#include <string>
using Vector3d = Eigen::Matrix<double,3,1>;

float magnitude(Vector3d &vec)
{
  return sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
}

void fwd_inv_kinematics_test(raisim::ArticulatedSystem *stoch, double &omega, double &radius, double &ctime)
{
    // control functions
      // std::cout<<"inside"<<std::endl;
      Vector3d end_effector_pos_FL(radius*cos(omega*ctime*2*PI),0.0,radius*sin(omega*ctime*2*PI)-0.4);
      Vector3d end_effector_pos_FR(radius*cos(omega*ctime*2*PI+PI),-0.02,radius*sin(omega*ctime*2*PI+PI)-0.4);
      Vector3d end_effector_pos_BL(radius*cos(omega*ctime*2*PI+PI),0.02,radius*sin(omega*ctime*2*PI+PI)-0.4);
      Vector3d end_effector_pos_BR(radius*cos(omega*ctime*2*PI),-0.02,radius*sin(omega*ctime*2*PI)-0.4);
      Vector3d fwd_kin_FL;
      Vector3d fwd_kin_FR;
      Vector3d fwd_kin_BL;
      Vector3d fwd_kin_BR;
      Vector3d thetas_FL;
      Vector3d thetas_FR;
      Vector3d thetas_BL;
      Vector3d thetas_BR;
      Eigen::VectorXd pos_target(19), vel_target(18);
      vel_target.setZero();
      inverse_kinematics(thetas_FL, end_effector_pos_FL,0);
      inverse_kinematics(thetas_FR, end_effector_pos_FR,1);
      inverse_kinematics(thetas_BL, end_effector_pos_BL,2);
      inverse_kinematics(thetas_BR, end_effector_pos_BR,3);
      
      forward_kinematics(fwd_kin_FL,thetas_FL,0);
      forward_kinematics(fwd_kin_FR,thetas_FR,1);
      forward_kinematics(fwd_kin_BL,thetas_BL,2);
      forward_kinematics(fwd_kin_BR,thetas_BR,3);
      pos_target<<0, 0, 0, //base co ordinates 
                  0, 0, 0,0,  //orientation 
                  thetas_FL[0], thetas_FL[1], thetas_FL[2], //leg 1
                  thetas_FR[0], thetas_FR[1], thetas_FR[2], //leg 2
                  thetas_BL[0], thetas_BL[1], thetas_BL[2], //leg 3
                  thetas_BR[0], thetas_BR[1], thetas_BR[2];
      fwd_kin_FL -=  end_effector_pos_FL; 
      fwd_kin_FR -=  end_effector_pos_FR;        
      fwd_kin_BL -=  end_effector_pos_BL;        
      fwd_kin_BR -=  end_effector_pos_BR;        
      if(magnitude(fwd_kin_FL )<0.001 && magnitude(fwd_kin_FR )<0.001 && magnitude(fwd_kin_BL )<0.001 && magnitude(fwd_kin_BR )<0.001)
      {
        std::cout<<"correct"<<std::endl;
      }  
      else
      {
        std::cout<<"not-correct"<<std::endl;
      }
          
      stoch->setGeneralizedCoordinate({ 1, 1, 0.6, //base co ordinates 
                                        1, 0, 0, 0,  //orientation 
                                        thetas_FL[0], thetas_FL[1], thetas_FL[2], //leg 1
                                        thetas_FR[0], thetas_FR[1], thetas_FR[2], //leg 2
                                        thetas_BL[0], thetas_BL[1], thetas_BL[2], //leg 3
                                        thetas_BR[0], thetas_BR[1], thetas_BR[2]}); //leg 4
}


void trot_test(raisim::ArticulatedSystem *stoch, double &omega, double &radius, double &ctime)
{
    Vector3d end_effector_pos_FL(radius*cos(omega*ctime*2*PI),0.02,radius*sin(omega*ctime*2*PI)-0.4);
    Vector3d end_effector_pos_FR(radius*cos(omega*ctime*2*PI+PI),-0.02,radius*sin(omega*ctime*2*PI+PI)-0.4);
    Vector3d end_effector_pos_BL(radius*cos(omega*ctime*2*PI+PI),0.02,radius*sin(omega*ctime*2*PI+PI)-0.4);
    Vector3d end_effector_pos_BR(radius*cos(omega*ctime*2*PI),-0.02,radius*sin(omega*ctime*2*PI)-0.4);
    Vector3d thetas_FL;
    Vector3d thetas_FR;
    Vector3d thetas_BL;
    Vector3d thetas_BR;
    Eigen::VectorXd pos_target(19), vel_target(18);
    vel_target.setZero();
    inverse_kinematics(thetas_FL, end_effector_pos_FL,0);
    inverse_kinematics(thetas_FR, end_effector_pos_FR,1);
    inverse_kinematics(thetas_BL, end_effector_pos_BL,2);
    inverse_kinematics(thetas_BR, end_effector_pos_BR,3);

    pos_target<<0, 0, 0, //base co ordinates 
                0, 0, 0,0,  //orientation 
                thetas_FL[0], thetas_FL[1], thetas_FL[2], //leg 1
                thetas_FR[0], thetas_FR[1], thetas_FR[2], //leg 2
                thetas_BL[0], thetas_BL[1], thetas_BL[2], //leg 3
                thetas_BR[0], thetas_BR[1], thetas_BR[2];

    // stoch->setGeneralizedCoordinate({ 1, 1, 0.6, //base co ordinates 
    //                                   1, 0, 0, 0,  //orientation 
    //                                   thetas_FL[0], thetas_FL[1], thetas_FL[2], //leg 1
    //                                   thetas_FR[0], thetas_FR[1], thetas_FR[2], //leg 2
    //                                   thetas_BL[0], thetas_BL[1], thetas_BL[2], //leg 3
    //                                   thetas_BR[0], thetas_BR[1], thetas_BR[2]}); //leg 4
    stoch->setPdTarget(pos_target, vel_target);
    //Simulation functions
}

void log_torque_data(raisim::ArticulatedSystem *stoch, std::string &fileName )
{
  std::ofstream myfile;
  auto force = stoch->getGeneralizedForce();
  myfile.open(fileName,  std::ios_base::app);
  myfile<<force[6]<<","<<force[7]<<","<<force[8]<<",";
  myfile<<force[9]<<","<<force[10]<<","<<force[11]<<",";
  myfile<<force[12]<<","<<force[13]<<","<<force[14]<<",";
  myfile<<force[15]<<","<<force[16]<<","<<force[17]<<"\n";
  myfile.close();
}