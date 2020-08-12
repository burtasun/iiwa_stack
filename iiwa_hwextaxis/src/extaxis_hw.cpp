
#include "extaxis_hw.hpp"


#define DEG2RAD(deg) deg*M_PI/180
#define RAD2DEG(rad) rad*180/M_PI


namespace iiwa_hwextaxis
{


    bool ExtAxisRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO_STREAM("iiwa_hwextaxis::ExtAxisRobotHW::ini");
        /*conexion y home*/
        bool res=true;
        this->tth_ = TurnTableHandler();
        res &= this->tth_.Connect();
        res &= this->tth_.gotoHome();
        if(!res){
            ROS_ERROR_STREAM("ExtAxisRobotHW: no se pudo conectar e ir a home en el plato externo");
            throw std::runtime_error("no se pudo conectar al plato externo");
        }
            
        /*Inicializacion variables*/
        joint_name_ = "JointExtAxis";

        joint_position_ = 0.0;
        joint_position_command_ = 0.0;

        joint_last_position_command_ = 0.0;
        joint_velocity_ = 0.0;
        joint_effort_ = 0.0;

        /*registro de interfaces / ligar variables de posicion y consigna interfaz*/
        //    joint state
        hardware_interface::JointStateHandle state_handle(
            joint_name_, &joint_position_, &joint_velocity_, &joint_effort_);
        state_interface_.registerHandle(state_handle);
        this->registerInterface(&state_interface_);
        //    joint command
        hardware_interface::JointHandle position_joint_handle(
            state_interface_.getHandle(joint_name_), &joint_position_command_);
        position_interface_.registerHandle(position_joint_handle);
        this->registerInterface(&position_interface_);

        return true;
    }

    void ExtAxisRobotHW::read(const ros::Time& time, const ros::Duration& period)
    {
        double currentPosDeg = tth_.getPos();
        this->joint_position_ = DEG2RAD(currentPosDeg);
    }

    void ExtAxisRobotHW::write(const ros::Time& time, const ros::Duration& period)
    {
        if(joint_position_command_ == joint_last_position_command_)//nuevas consignas unicamente
            return;

        bool res=true;
        joint_last_position_command_ = joint_position_command_;
        if(!tth_.MovePos(joint_position_command_))
            ROS_ERROR_STREAM("extaxis_hw  error while commanding the " << this->joint_name_ << " joint!");

    }

    bool ExtAxisRobotHW::prepareSwitch(
        const std::list<hardware_interface::ControllerInfo>& ,
        const std::list<hardware_interface::ControllerInfo>& )
    {
        return true;
    }
    void ExtAxisRobotHW::doSwitch(
        const std::list<hardware_interface::ControllerInfo>& ,
        const std::list<hardware_interface::ControllerInfo>& )
    {
        return;
    }

}  // namespace iiwa_hwextaxis

PLUGINLIB_EXPORT_CLASS(iiwa_hwextaxis::ExtAxisRobotHW, hardware_interface::RobotHW)
