#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/kdl.hpp>
#include <stdio.h>
#include <iostream>
 
using namespace KDL;
 
 
int main( int argc, char** argv )
{
    //Definition of a kinematic chain & add segments to the chain
    KDL::Chain chain;
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,1))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,1))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,1))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,1))));
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0))));

    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 
    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
 
    // Assign some values to the joint positions
    jointpositions(0)=0;
    jointpositions(1)=0;
    jointpositions(2)=0;
    jointpositions(3)=0;
    jointpositions(4)=1.57079632679;
    jointpositions(5)=1.57079632679;

    // Create the frame that will contain the results
    KDL::Frame cartpos;    
 
    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

    //Creation of the solvers:
    ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
    ChainIkSolverPos_NR iksolver1(chain,fksolver,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

    //Creation of jntarrays:
    JntArray q(chain.getNrOfJoints());
    JntArray q_init(chain.getNrOfJoints());

    //Set destination frame
    Frame F_dest=Frame(Rotation::RPY(0, 3.14159265359, 0),Vector(1.0, 0,4));

    int ret = iksolver1.CartToJnt(q_init,F_dest,q);

    for (int i = 0; i < q.rows(); i++)
    {
        std::cout << q(i) << std::endl;
        jointpositions(i) = q(i);
    }

    // Calculate forward position kinematics
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

}


// pi   = 3.14159265359
// pi/2 = 1.57079632679
// pi/4 = 0.78539816339