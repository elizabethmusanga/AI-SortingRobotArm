// Include the necessary headers
#include <iostream>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <urdf/model.h>

// Define the robot parameters
#define NUM_JOINTS 5 // Number of joints in the robot
#define BASE_LINK "base_link" // Name of the base link of the robot
#define TIP_LINK "gripper_finger_1" // Name of the tip link of the robot
#define URDF_FILE "/home/newton/ROS2/ai_sorting_robotic_arm/src/AI-SortingRobotArm/robotic_arm_description/urdf/robotic_arm_urdf.urdf"
" // Name of the URDF file

// Declare global variables
KDL::JntArray joint_seed, joint_solution; // Joint arrays for the seed and solution
KDL::Frame desired_pose; // Frame for the desired pose
TRAC_IK::TRAC_IK *tracik_solver; // Pointer to the TRAC IK solver object

// Function to load the URDF model from a file
bool load_urdf_model(const std::string& file_name, urdf::Model& model)
{
  // Open the file stream
  std::ifstream ifs(file_name);
  if (!ifs.is_open()) // Check if the file is open
  {
    std::cerr << "Failed to open file: " << file_name << std::endl;
    return false;
  }

  // Read the file content into a string
  std::string xml_string;
  xml_string.assign((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // Close the file stream
  ifs.close();

  // Parse the string into a URDF model
  if (!model.initString(xml_string)) // Check if the parsing is successful
  {
    std::cerr << "Failed to parse URDF: " << file_name << std::endl;
    return false;
  }

  // Return true if everything is OK
  return true;
}

int main(int argc, char** argv)
{
  // Create a URDF model object
  urdf::Model model;

  // Load the URDF model from the file
  bool valid = load_urdf_model(URDF_FILE, model);
  if (!valid) // Check if the model is valid
  {
    std::cerr << "Invalid model" << std::endl;
    return -1;
  }

  // Create the TRAC IK solver object with the robot parameters and the URDF model
  tracik_solver = new TRAC_IK::TRAC_IK(BASE_LINK, TIP_LINK, model, 0.005, 1e-5, TRAC_IK::Speed);

  // Get the KDL chain from the solver
  KDL::Chain chain;
  valid = tracik_solver->getKDLChain(chain);
  if (!valid) // Check if the chain is valid
  {
    std::cerr << "Invalid chain" << std::endl;
    return -1;
  }

  // Resize the joint arrays according to the number of joints
  joint_seed.resize(NUM_JOINTS);
  joint_solution.resize(NUM_JOINTS);

  // Set the desired pose as a KDL frame
  // You can modify this according to your needs
  desired_pose = KDL::Frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0.5, 0.5, 0.5));

  // Solve the inverse kinematics problem using the TRAC IK solver
  int rc = tracik_solver->CartToJnt(joint_seed, desired_pose, joint_solution);
  if (rc >= 0) // Check if the solver succeeded
  {
    std::cout << "IK solution found" << std::endl;
    // Print the joint solution
    for (int i = 0; i < NUM_JOINTS; i++)
    {
      std::cout << "Joint " << i + 1 << ": " << joint_solution(i) << std::endl;
    }
  }
  else // The solver failed
  {
    std::cerr << "IK solution not found" << std::endl;
  }

  // Delete the TRAC IK solver object
  delete tracik_solver;

  return 0;
}
