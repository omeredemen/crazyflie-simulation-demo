/** 
 *  ...........       ____  _ __
 *  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 *  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 *  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *  
 * MIT License
 * 
 * Copyright (c) 2022 Bitcraze
 * 
 * 
 * @file CrazyflieController.cc
 * A simple control plugin for Ignition Gazebo for controlling the motors
 * 
 */

#include "CrazyflieController.hh"

#include <ignition/plugin/Register.hh>

#include <ignition/msgs/actuators.pb.h>
#include "ignition/gazebo/components/Actuators.hh"


using namespace crazyflie_controller;

IGNITION_ADD_PLUGIN(
    crazyflie_controller::CrazyflieController,
    ignition::gazebo::System,
    crazyflie_controller::CrazyflieController::ISystemConfigure,
    crazyflie_controller::CrazyflieController::ISystemPreUpdate)

using namespace crazyflie_controller;

CrazyflieController::CrazyflieController()
{
}

CrazyflieController::~CrazyflieController()
{

}

void CrazyflieController::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &/*_eventMgr*/)
{
    this->model = ignition::gazebo::Model(_entity);
      this->motorCommands.mutable_velocity()->Resize(
      4, 0);

    _ecm.CreateComponent(this->model.Entity(),
                      ignition::gazebo::components::Actuators(this->motorCommands));
    this->node.Subscribe(topic, &CrazyflieController::cb, this);
    // if (!this->node.Subscribe(topic, &CrazyflieController::cb))
    // {
    //     std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
    // }
    
}

void CrazyflieController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{

    this->motorCommands.set_velocity(0, commands);
    this->motorCommands.set_velocity(1, commands);
    this->motorCommands.set_velocity(2, commands);
    this->motorCommands.set_velocity(3, commands);

    auto motorCommandsComponent =
        _ecm.Component<ignition::gazebo::components::Actuators>(this->model.Entity());

    motorCommandsComponent->SetData(this->motorCommands, 
                        [](const ignition::msgs::Actuators &, const ignition::msgs::Actuators &){return false;});

    _ecm.SetChanged(this->model.Entity(), ignition::gazebo::components::Actuators::typeId, ignition::gazebo::ComponentState::PeriodicChange);

   // ignmsg << "set motors" << std::endl;
}

void CrazyflieController::cb(const ignition::msgs::Float &_msg){
    std::lock_guard<std::mutex> lock(this->recvdFloatMsgMutex);
    commands = _msg.data();
    ignmsg << "fdfsadsadsa" << std::endl;
}



