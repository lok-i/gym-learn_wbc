#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#define PI  3.141592
//LEG LINK LENGTHS
#define L1 0.12
#define L2 0.303
#define L3 0.303

#define FL 0
#define FR 1
#define BL 2
#define BR 3
#define MASS 11.168
/*inertia  = 0.178402 -0.0458692 0.0180358 
-0.0458692 0.470056 -8.95236e-08 
0.0180358 -8.95236e-08 0.446392 
*/
//LEG-FL

int bound_end_eff_pos(double &x, double &y, double &z)
{
    if(x<= 0.4 && x>= - 0.4)
    {
        if(y <= 0.22 && y>= 0)
        {
            if(z<= -0.15 && z>= -0.4)
            {
                return 1;
            }
        }
    }
    return 0;
}
void _forward_kinematics_FL(Eigen::Vector3d &end_effector_pos, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    end_effector_pos[0] = (l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]));
    end_effector_pos[1] = l1*cos(thetas[0])+(l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*sin(thetas[0]);
    end_effector_pos[2] = l1*sin(thetas[0])-((l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*cos(thetas[0]));
}

void _inverse_kinematics_FL(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos, float l1 = L1, float l2 = L2, float l3 = L3)
{
    double x = end_effector_pos[0];
    double y = end_effector_pos[1];
    double z = end_effector_pos[2];
    int isValid = bound_end_eff_pos(x,y,z);
    if(!isValid)
    {
        std::cout<<"invalid point: "<<x<<","<<y<<","<<z<<std::endl;
    }
    double r, th23, th1, th2, th3,t;
    r = sqrt(y*y + z*z -l1*l1);
    th1 = atan2(y*r + z*l1, y*l1 - z*r);
    t = (2*l2*x + sqrt(4*pow(l2,2)*pow(r,2) - pow(r,4) + 4*pow(l2,2)*pow(x,2) - 2*pow(r,2)*pow(x,2) - pow(x,4)))/(2*l2*r + pow(r,2) + pow(x,2));
    th23 = atan2(2*t, 1-t*t);
    th2 = atan2(x - l2*sin(th23), r - l2*cos(th23));
    th3 = th23 - th2;
    thetas[0]=th1;
    thetas[1]=th2;
    thetas[2]=th3;
}

void _jacobian_FL(Eigen::Matrix<double, 3, 3> &jacob, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    jacob << 0,l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]),l3*cos(thetas[1] + thetas[2]),
           (l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*cos(thetas[0]) - l1*sin(thetas[0]),(-l2*sin(thetas[1]) - l3*sin(thetas[1] + thetas[2]))*sin(thetas[0]),-l3*sin(thetas[1] + thetas[2])*sin(thetas[0]),
           ((l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*sin(thetas[0])) +l1*cos(thetas[0]),(l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]))*cos(thetas[0]),l3*sin(thetas[1] + thetas[2])*cos(thetas[0]);

}

void _inverse_dynamics_FL(Eigen::Vector3d &torques, Eigen::Vector3d &force, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Finds Joint torques for desired force at end effector
    torques =  [abduction, hip, knee]
    force = [x,y,z]
    thetas = [abduction, hip, knee] angles in radians
    l1, l2, l3 = link lengths
    */
    Eigen::Matrix<double,3,3> jacob;
    _jacobian_FL(jacob, thetas);
    torques = jacob.transpose()*force; 
}  


//LEG-FR
void _forward_kinematics_FR(Eigen::Vector3d &end_effector_pos, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    end_effector_pos[0] = (l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]));
    end_effector_pos[1] = -1*(l1*cos(thetas[0])+(l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*sin(thetas[0]));
    end_effector_pos[2] = l1*sin(thetas[0])-((l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*cos(thetas[0]));
}

void _inverse_kinematics_FR(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos, float l1 = L1, float l2 = L2, float l3 = L3)
{
    double x = end_effector_pos[0];
    double y = -1*end_effector_pos[1];
    double z = end_effector_pos[2];
    int isValid = bound_end_eff_pos(x,y,z);
    if(!isValid)
    {
        std::cout<<"invalid point: "<<x<<","<<y<<","<<z<<std::endl;
    }
    double r, th23, th1, th2, th3,t;
    r = sqrt(y*y + z*z -l1*l1);
    th1 = atan2(y*r + z*l1, y*l1 - z*r);
    t = (2*l2*x + sqrt(4*pow(l2,2)*pow(r,2) - pow(r,4) + 4*pow(l2,2)*pow(x,2) - 2*pow(r,2)*pow(x,2) - pow(x,4)))/(2*l2*r + pow(r,2) + pow(x,2));
    th23 = atan2(2*t, 1-t*t);
    th2 = atan2(x - l2*sin(th23), r - l2*cos(th23));
    th3 = th23 - th2;
    thetas[0]=th1;
    thetas[1]=th2;
    thetas[2]=th3;
}

void _jacobian_FR(Eigen::Matrix<double, 3, 3> &jacob, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
   jacob << 0,l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]),l3*cos(thetas[1] + thetas[2]),
           (l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*cos(thetas[0]) - l1*sin(thetas[0]),(l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]))*sin(thetas[0]),l3*sin(thetas[1] + thetas[2])*sin(thetas[0]),
           ((l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*sin(thetas[0])) +l1*cos(thetas[0]),(l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]))*cos(thetas[0]),l3*sin(thetas[1] + thetas[2])*cos(thetas[0]);
}

void _inverse_dynamics_FR(Eigen::Vector3d &torques, Eigen::Vector3d &force, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Finds Joint torques for desired force at end effector
    torques =  [abduction, hip, knee]
    force = [x,y,z]
    thetas = [abduction, hip, knee] angles in radians
    l1, l2, l3 = link lengths
    */
    Eigen::Matrix<double,3,3> jacob;
    _jacobian_FR(jacob, thetas);
    torques = jacob.transpose()*force; 
    torques[0]=-1*torques[0];
} 

//LEG-BL
void _forward_kinematics_BL(Eigen::Vector3d &end_effector_pos, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    end_effector_pos[0] = (l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]));
    end_effector_pos[1] = l1*cos(thetas[0])+(l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*sin(thetas[0]);
    end_effector_pos[2] = l1*sin(thetas[0])-((l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*cos(thetas[0]));
}

void _inverse_kinematics_BL(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos, float l1 = L1, float l2 = L2, float l3 = L3)
{
    double x = end_effector_pos[0];
    double y = end_effector_pos[1];
    double z = end_effector_pos[2];
    int isValid = bound_end_eff_pos(x,y,z);
    if(!isValid)
    {
        std::cout<<"invalid point: "<<x<<","<<y<<","<<z<<std::endl;
    }
    double r, th23, th1, th2, th3,t;
    r = sqrt(y*y + z*z -l1*l1);
    th1 = atan2(y*r + z*l1, y*l1 - z*r);
    t = (2*l2*x + sqrt(4*pow(l2,2)*pow(r,2) - pow(r,4) + 4*pow(l2,2)*pow(x,2) - 2*pow(r,2)*pow(x,2) - pow(x,4)))/(2*l2*r + pow(r,2) + pow(x,2));
    th23 = atan2(2*t, 1-t*t);
    th2 = atan2(x - l2*sin(th23), r - l2*cos(th23));
    th3 = th23 - th2;
    thetas[0]=th1;
    thetas[1]=th2;
    thetas[2]=th3;
}

void _jacobian_BL(Eigen::Matrix<double, 3, 3> &jacob, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
   jacob << 0,l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]),l3*cos(thetas[1] + thetas[2]),
           (l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*cos(thetas[0]) - l1*sin(thetas[0]),(-l2*sin(thetas[1]) - l3*sin(thetas[1] + thetas[2]))*sin(thetas[0]),-l3*sin(thetas[1] + thetas[2])*sin(thetas[0]),
           ((l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*sin(thetas[0])) +l1*cos(thetas[0]),(l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]))*cos(thetas[0]),l3*sin(thetas[1] + thetas[2])*cos(thetas[0]);

}

void _inverse_dynamics_BL(Eigen::Vector3d &torques, Eigen::Vector3d &force, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Finds Joint torques for desired force at end effector
    torques =  [abduction, hip, knee]
    force = [x,y,z]
    thetas = [abduction, hip, knee] angles in radians
    l1, l2, l3 = link lengths
    */
    Eigen::Matrix<double,3,3> jacob;
    _jacobian_BL(jacob, thetas);
    torques = jacob.transpose()*force; 
} 

//Leg - BR
void _forward_kinematics_BR(Eigen::Vector3d &end_effector_pos, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    end_effector_pos[0] = (l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]));
    end_effector_pos[1] = -1*(l1*cos(thetas[0])+(l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*sin(thetas[0]));
    end_effector_pos[2] = l1*sin(thetas[0])-((l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*cos(thetas[0]));
}

void _inverse_kinematics_BR(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos, float l1 = L1, float l2 = L2, float l3 = L3)
{
    double x = end_effector_pos[0];
    double y = -1*end_effector_pos[1];
    double z = end_effector_pos[2];
    int isValid = bound_end_eff_pos(x,y,z);
    if(!isValid)
    {
        std::cout<<"invalid point: "<<x<<","<<y<<","<<z<<std::endl;
    }
    double r, th23, th1, th2, th3,t;
    r = sqrt(y*y + z*z -l1*l1);
    th1 = atan2(y*r + z*l1, y*l1 - z*r);
    t = (2*l2*x + sqrt(4*pow(l2,2)*pow(r,2) - pow(r,4) + 4*pow(l2,2)*pow(x,2) - 2*pow(r,2)*pow(x,2) - pow(x,4)))/(2*l2*r + pow(r,2) + pow(x,2));
    th23 = atan2(2*t, 1-t*t);
    th2 = atan2(x - l2*sin(th23), r - l2*cos(th23));
    th3 = th23 - th2;
    thetas[0]=th1;
    thetas[1]=th2;
    thetas[2]=th3;
}

void _jacobian_BR(Eigen::Matrix<double, 3, 3> &jacob, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
   jacob << 0,l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]),l3*cos(thetas[1] + thetas[2]),
           (l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*cos(thetas[0]) - l1*sin(thetas[0]),(l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]))*sin(thetas[0]),l3*sin(thetas[1] + thetas[2])*sin(thetas[0]),
           ((l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*sin(thetas[0])) +l1*cos(thetas[0]),(l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]))*cos(thetas[0]),l3*sin(thetas[1] + thetas[2])*cos(thetas[0]);


}

void _inverse_dynamics_BR(Eigen::Vector3d &torques, Eigen::Vector3d &force, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Finds Joint torques for desired force at end effector
    torques =  [abduction, hip, knee]
    force = [x,y,z]
    thetas = [abduction, hip, knee] angles in radians
    l1, l2, l3 = link lengths
    */
    Eigen::Matrix<double,3,3> jacob;
    _jacobian_BR(jacob, thetas);
    torques = jacob.transpose()*force;
    torques[0]=-1*torques[0];
} 




void forward_kinematics(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos, int leg, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Performs forward kinematics for a 3R spatial manipulator
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    FL=0
    FR=1
    BL=2
    BR=3
    */
   if(leg == FL)
   {
       _forward_kinematics_FL(thetas, end_effector_pos);
   }
   else if(leg == FR)
   {
       _forward_kinematics_FR(thetas, end_effector_pos);
   }
   else if(leg == BL)
   {
       _forward_kinematics_BL(thetas, end_effector_pos);     
   }
   else if(leg == BR)
   {
       _forward_kinematics_BR(thetas, end_effector_pos);          
   }
   else
   {
       std::cout<<"Invalid leg number, forward kinematics"<<std::endl;
   }
   
}

void inverse_kinematics(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos, int leg, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Performs forward kinematics for a 3R spatial manipulator
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    FL=0
    FR=1
    BL=2
    BR=3
    */
   if(leg == FL)
   {
       _inverse_kinematics_FL(thetas, end_effector_pos);
   }
   else if(leg == FR)
   {
       _inverse_kinematics_FR(thetas, end_effector_pos);
   }
   else if(leg == BL)
   {
       _inverse_kinematics_BL(thetas, end_effector_pos);     
   }
   else if(leg == BR)
   {
       _inverse_kinematics_BR(thetas, end_effector_pos);          
   }
   else
   {
       std::cout<<"Invalid leg number, inverse kinematics"<<std::endl;
   }
   
}






// int main()
// {

//     // Eigen::Vector3d end_eff(0.15, 0.15, 0.2);
//     // Eigen::Vector3d fwd_kin(0.0, 0.0, 0.0);
//     // Eigen::Vector3d jpos(0,0,0);
//     // Eigen::Matrix<double,3,3> jaco;
//     // // _inverse_kinematics_BR(jpos, end_eff);
//     // _forward_kinematics_FL(fwd_kin, jpos);
//     // _jacobian_FL(jaco, jpos);
//     // std::cout<<jaco<<std::endl;
//     Eigen::Vector3d thetas(1.2, 0.3, 0.66);
//     Eigen::Vector3d fwd_kin(0.0, 0.0, 0.0);
//     Eigen::Vector3d inv_kin(0,0,0);
//     Eigen::Vector3d fwd_kin2(0,0,0);
//     _forward_kinematics_FL(fwd_kin, thetas);
//     _inverse_kinematics_FL(inv_kin, fwd_kin);
//     _forward_kinematics_FL(fwd_kin2, inv_kin);
//     std::cout<<fwd_kin[0]<<","<<fwd_kin[1]<<","<<fwd_kin[2]<<std::endl;
//     std::cout<<fwd_kin2[0]<<","<<fwd_kin2[1]<<","<<fwd_kin2[2]<<std::endl;

//     return 0;
// }