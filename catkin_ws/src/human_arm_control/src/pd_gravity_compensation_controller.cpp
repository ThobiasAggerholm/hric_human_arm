/*
    Credits to:
        https://github.com/modulabs/arm-control/blob/master/arm_controllers/src/gravity_comp_controller.cpp
        https://www.youtube.com/watch?v=7BLc18lOFJw

*/
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <vector>
#include <memory>

class pd_grav_control : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
    public:
    //Inherited from controller_interface::Controller
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        //Get joint key values from parameter server.
        std::vector<std::string> param_joints;
        if(!n.getParam("joint", param_joints))
        {
            ROS_ERROR("No joints were retrieved from parameter server under /joint.");
            return false;
        }
        //Get the handles to each joint resource from hardware layer
        n_joints_ = param_joints.size();
        joint_resource_handles_.resize(n_joints_);
        qd_.resize(n_joints_);
        for(int i = 0; i < n_joints_; ++i)
        {
            hardware_interface::JointHandle joint_resource_handle = hw->getHandle(param_joints[i]); //If it fails to return handle it throws exception.
            joint_resource_handles_[i] = joint_resource_handle;
            qd_(i) = joint_resource_handle.getPosition();
        }

        //Get proportional gains
        for(int i = 0; i < n_joints_; ++i)
        {
            if(!n.getParam("pid/" + param_joints[i] + "/p", kps_[i]))
            {
                ROS_ERROR("Joint kp values could not be found on parameter server under /pid/p.");
                return false;
            }
            if(!n.getParam("pid/" + param_joints[i] + "/d", kds_[i]))
            {
                ROS_ERROR("Joint kp values could not be found on parameter server under /pid/p.");
                return false;
            } 
        }

        //Get KDL tree
        if(!kdl_parser::treeFromParam("robot_description", human_arm_tree_))
        {
            ROS_ERROR("Could not read robot_description from parameter server to KDL tree.");
            return false;
        }
        
        //Get KDL chain from tree
        if(!human_arm_tree_.getChain("base_link", "Hand", human_arm_chain_))
        {
            ROS_ERROR("Could not get chain from tree.");
            return false;
        }

        G_.resize(n_joints_);
        gravity_ = KDL::Vector::Zero();
        gravity_(2) = -9.81;
        id_solver_.reset(new KDL::ChainDynParam(human_arm_chain_, gravity_));

        sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &pd_grav_control::setCommandCB, this);


        return true;
    }

    //Inherited from controller_interface::ControllerBase
    void update (const ros::Time &time, const ros::Duration &period)
    {

        KDL::JntArray q(n_joints_);
        for(int i = 0; i < n_joints_; ++i)
        {
            q(i) = joint_resource_handles_[i].getPosition();;
        }
        id_solver_->JntToGravity(q, G_);
        for(int i = 0; i < n_joints_; ++i)
        {
            double q_error = qd_(i) - joint_resource_handles_[i].getPosition();
            double effort = G_(i) + kps_[i] * q_error - kds_[i] * joint_resource_handles_[i].getVelocity();
            joint_resource_handles_[i].setCommand(effort);

        }
    }

    void setCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {

        for(int i = 0; i < n_joints_; ++i)
        {
            qd_(i) = msg->data[i];
        }
        
    }

    //Inherited from controller_interface::ControllerBase
    void starting (const ros::Time &)
    {

    }

    //Inherited from controller_interface::ControllerBase
    void stopping (const ros::Time &)
    {

    }
    private:
    std::vector<hardware_interface::JointHandle> joint_resource_handles_;
    std::vector<double> kps_;
    std::vector<double> kds_;
    KDL::JntArray qd_;
    ros::Subscriber sub_command_;
    int n_joints_;
    //Gravity constant gazebo world
    KDL::Vector gravity_;
    KDL::JntArray G_;

    KDL::Tree human_arm_tree_;
    KDL::Chain human_arm_chain_;
    std::unique_ptr<KDL::ChainDynParam> id_solver_;

};

//Make controller available to Controller Manager
PLUGINLIB_EXPORT_CLASS(pd_grav_control,controller_interface::ControllerBase);