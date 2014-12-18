#include <iostream>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#define DEBUG 0

int main(int argc, char *argv[])
{
    using namespace KDL;

    std::cout << "kdl tester" << std::endl;

    ros::init(argc, argv, "kdl_parser_node");
    ros::NodeHandle node;
    std::string urdf;
    node.param("wam/robot_description", urdf, std::string());
#if DEBUG
    std::cout << "urdf: \n" << urdf << std::endl;
#endif

    // get tree from urdf string
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromString(urdf, my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return -1;
    }
    std::cout << "NumJoints in Tree = " << my_tree.getNrOfJoints() << std::endl;

    // kdl::tree -> kdl::chain
    KDL::Chain my_chain;
    std::string rootLink = "wam/base_link";
    std::string tipLink = "wam/cutter_tip_link";
    if (!my_tree.getChain(rootLink, tipLink, my_chain))
    {
        ROS_ERROR("Failed to get chain from kdl tree, check root/rip link");
        return -1;
    }
    std::cout << "NumJoints in Chain = " << my_chain.getNrOfJoints() << std::endl;

    // FK Solver
    ChainFkSolverPos_recursive fkSolver = ChainFkSolverPos_recursive(my_chain);
    unsigned int numJnts = my_chain.getNrOfJoints();
    KDL::JntArray jnt_q = JntArray(numJnts);
    std::cout << "jnt_q = " << jnt_q.data.transpose() << std::endl;

    Frame tipFrame;
    fkSolver.JntToCart(jnt_q, tipFrame);
    std::cout << "tipFrame = \n" << tipFrame << std::endl;

    return 0;
}
