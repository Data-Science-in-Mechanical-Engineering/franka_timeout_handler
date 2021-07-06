#include "../include/franka_real_time/robot.h"
#include <iostream>
#include<math.h>
#define PI 3.14159265
int _main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./hold IP" << std::endl;
        return 1;
    }

    franka_real_time::Robot robot(argv[1]);
    robot.start();
    robot.move_to_reference();

    //for(int i=0;i<1000;i++){
    //    robot.receive_and_send();
    //}
    //robot.receive();
    //Eigen::Matrix<double, 7, 1> position_joint = robot.get_joint_positions();
    //std::cout<<position_joint<<std::endl;
    double error=0;
    if(true){
    //std::cout<<Next_target<<std::endl;
    robot.receive();
    double r=0.15;
    double ellipse_factor=1.25;
    Eigen::Matrix<double, 3, 1> center=robot.get_position();
    //center(1)-=ellipse_factor*r;
    //robot.set_target_position(Next_target);
    //Next_target=robot.get_position();
    //std::cout<<Next_target<<std::endl;
    //robot.send();
    Eigen::Matrix<double, 3, 1> Next_target =center;
    Eigen:: Matrix<double,3,1> reference = Next_target;
    reference(1)+=ellipse_factor*r;
    //robot.set_target_position(Next_target);
    int traj_time=1000;
    double time_factor = 1.0/traj_time;
    for(int i=0;i<traj_time;i++){
        Next_target = center + i*time_factor*(reference-center);
        robot.set_target_position(Next_target);
        robot.receive_and_send();
    }
    Eigen::Matrix<double, 3, 3> stiffness=robot.get_translation_stiffness();
    double factor=0.6;
    stiffness=factor*stiffness;
    Eigen::Matrix<double,3,3> damping=robot.get_translation_damping();
    damping=sqrt(factor)*damping;
    robot.set_translation_damping(damping);
    robot.set_translation_stiffness(stiffness);
    int i=0;
    
    double w=PI/2000;
    
    Eigen::Matrix<double,3,1> position=robot.get_position();
    Eigen::Matrix<double,3,1> current_target=robot.get_target_position();
    Eigen::Matrix<double,3,1> difference; 
    difference.setZero();
    while (true)
    {   
        
        double sin_x=sin (w*i+PI);
        double cos_x=cos (w*i);
        Next_target(0)=center(0)+r*sin_x;
        Next_target(1)=center(1)+ellipse_factor*r*cos_x;
        //std::cout<<Next_target<<std::endl;
        robot.receive();
        current_target=robot.get_target_position();
        robot.set_target_position(Next_target);
        position=robot.get_position();
        robot.send();
        difference=position-current_target;
        double norm=difference.norm();
        if (error<norm) {
            error=norm;
            //std::cout<<error<<std::endl;
            }
        //std::cout << (robot.get_late() ? "Late" : "Not late") << std::endl;
        i+=1;
        
    }
    }
    else{
        while (true)
        {   
    
            robot.receive();
            robot.send();
            std::cout << (robot.get_late() ? "Late" : "Not late") << std::endl;
            
        }
    }
    
    return 0;
}

int main(int argc, char **argv)
{
    return _main(argc, argv);
}