#include "robotic_arm_controllers/robotic_arm_hwinterface.hpp"


namespace robotic_arm_hw
{

    RoboticArmHWInterface::RoboticArmHWInterface(){

    }

    // Destructor
    RoboticArmHWInterface::~RoboticArmHWInterface(){
        // Check if 

        if (arduino_port.IsOpen()){
            try
            {
                arduino_port.
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
        }
    }

}