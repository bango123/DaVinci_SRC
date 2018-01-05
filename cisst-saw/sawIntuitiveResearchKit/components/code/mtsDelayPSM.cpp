// system include
#include <iostream>
#include <time.h>

// cisst

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawIntuitiveResearchKit/mtsDelayPSM.h>
#include <ros/ros.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDelayPSM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);


mtsDelayPSM::mtsDelayPSM(const std::string & componentName, const double periodInSeconds):
mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

mtsDelayPSM::mtsDelayPSM(const mtsTaskPeriodicConstructorArg & arg):
mtsTaskPeriodic(arg)
{
    Init();
}

mtsDelayPSM::~mtsDelayPSM()
{
}

void mtsDelayPSM::Init(void)
{
    mDelay   = 0.0;
    mRosOnly = false;

	//Master side "delayed" feedback about where the slave arms are
    this->StateTable.AddData(CartesianCurrent, "CartesianPosition");
    this->StateTable.AddData(StateJointCurrent, "StateJoint");

	//Master side target position from the master arms
    this->StateTable.AddData(cartesian_target_master, "CartesianTarget_master");
    this->StateTable.AddData(jaw_target_master,       "JawTarget_master");

	//Slave side target position for the slave arms
    this->StateTable.AddData(cartesian_target_slave, "CartesianTarget_slave");
    this->StateTable.AddData(jaw_target_slave,       "JawTarget_slave");

    //Set up the interfraces required by tele-op PSM
    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("TeleOp");


   if (interfaceProvided) {
	//These 2 are for the master side getting the "delayed" information
     interfaceProvided->AddCommandReadState(this->StateTable, CartesianCurrent,  "GetPositionCartesian");
     interfaceProvided->AddCommandReadState(this->StateTable, StateJointCurrent, "GetStateJoint");

     interfaceProvided->AddCommandReadState(this->StateTable, cartesian_target_master, "GetTargetCartesian_master");
     interfaceProvided->AddCommandReadState(this->StateTable, jaw_target_master,       "GetJawTarget_master");

     interfaceProvided->AddCommandReadState(this->StateTable, cartesian_target_slave,  "GetTargetCartesian_slave");
     interfaceProvided->AddCommandReadState(this->StateTable, jaw_target_slave,        "GetJawTarget_slave");

     interfaceProvided->AddCommandRead( &mtsDelayPSM::GetRobotControlState,      this, "GetRobotControlState", std::string(""));
     interfaceProvided->AddCommandWrite(&mtsDelayPSM::SetRobotControlState,      this, "SetRobotControlState", std::string(""));
     interfaceProvided->AddCommandWrite(&mtsDelayPSM::SetPositionCartesian_tele, this, "SetPositionCartesian");
     interfaceProvided->AddCommandWrite(&mtsDelayPSM::SetJawPosition_tele,       this, "SetJawPosition");
     interfaceProvided->AddCommandWrite(&mtsDelayPSM::SetDelay,                  this, "SetDelay");

     interfaceProvided->AddEventWrite(MessageEvents.Error,      "Error",      std::string(""));
  
     interfaceProvided->AddCommandWrite(&mtsDelayPSM::SetRosOnly, this, "SetRosOnly", false);

     interfaceProvided->AddCommandWrite(&mtsDelayPSM::SetPositionCartesian,     this, "SetPositionCartesian_ROS");
     interfaceProvided->AddCommandWrite(&mtsDelayPSM::SetJawPosition,           this, "SetJawPosition_ROS");


 }

//   interfaceProvided = AddInterfaceProvided("ROS");


    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("PSM");


  if (interfaceRequired) {

    interfaceRequired->AddFunction("GetPositionCartesian", GetPositionCartesian_psm);
    interfaceRequired->AddFunction("SetPositionCartesian", SetPositionCartesian_psm);
    interfaceRequired->AddFunction("SetJawPosition",       SetJawPosition_psm);
    interfaceRequired->AddFunction("GetRobotControlState", GetRobotControlState_psm);
    interfaceRequired->AddFunction("SetRobotControlState", SetRobotControlState_psm);
    interfaceRequired->AddFunction("GetStateJoint",        GetStateJoint_psm);

    interfaceRequired->AddEventHandlerWrite(&mtsDelayPSM::PSMErrorEventHandler,
                                            this, "Error");
  }
}

void mtsDelayPSM::Run(void){
    //Update the cartesian position on the master side of the world
    if (mDelay == 0.0 || m_GetPositionCoordinates_buffer.empty()){
        GetPositionCartesian_psm(CartesianCurrent);
    }

    if (mDelay != 0.0){
       GetPositionCartesian_psm(CartesianGetParam);
       m_GetPositionCoordinates_buffer.push_back(CartesianGetParam);
       m_GetPosition_TimeStamps_buffer.push_back(ros::Time::now());

       if(ros::Time::now() - m_GetPosition_TimeStamps_buffer.front() > ros::Duration(mDelay/1000.0)){
         while(ros::Time::now() - m_GetPosition_TimeStamps_buffer.front() > ros::Duration(mDelay/1000.0) ){
            //std::cout << "Added Delay   : " << ros::Time::now() - m_GetPosition_TimeStamps_buffer.front() << std::endl;
            m_GetPositionCoordinates_buffer.pop_front();
            m_GetPosition_TimeStamps_buffer.pop_front();
            if( m_GetPositionCoordinates_buffer.empty()){
                break;
              }
          }

         if( !m_GetPositionCoordinates_buffer.empty()){
            CartesianCurrent = m_GetPositionCoordinates_buffer.front();
         }

       }
    }

    //Update the joint angles on the master side of the world
    if (mDelay == 0.0 || m_GetPositionCoordinates_buffer.empty()){
        GetStateJoint_psm(StateJointCurrent);
    }
    if (mDelay != 0.0){
        GetStateJoint_psm(StateJointGetParam);
        m_GetJoint_buffer.push_back(StateJointGetParam);
        m_GetJoint_TimeStamps_buffer.push_back(ros::Time::now());
        
        if( ros::Time::now() - m_GetJoint_TimeStamps_buffer.front() > ros::Duration(mDelay/1000.0) ){
          while(ros::Time::now() - m_GetJoint_TimeStamps_buffer.front() > ros::Duration(mDelay/1000.0)){
              //std::cout << "Added Delay   : " << ros::Time::now() - m_GetJoint_TimeStamps_buffer.front() << std::endl;
              m_GetJoint_buffer.pop_front();
              m_GetJoint_TimeStamps_buffer.pop_front();
              if( m_GetJoint_buffer.empty()){
                break;
              }
            }

            if( !m_GetJoint_buffer.empty()){
              StateJointCurrent = m_GetJoint_buffer.front();
            }
        }
    }

    ProcessQueuedEvents();
    ProcessQueuedCommands();
}

void mtsDelayPSM::Startup(void){

}

void mtsDelayPSM::Cleanup(void){
  //Hopefully improve stability for the MTM...
  ClearBuffers();
  GetPositionCartesian_psm(CartesianCurrent);
  GetStateJoint_psm(StateJointCurrent);

}

void mtsDelayPSM::Configure(std::string const & filename){

}

void mtsDelayPSM::GetRobotControlState(std::string & state) const
{
    GetRobotControlState_psm(state);
}

void mtsDelayPSM::SetRobotControlState(const std::string & state)
{
    //Improve stability for the MTM
    ClearBuffers();
    GetPositionCartesian_psm(CartesianCurrent);
    GetStateJoint_psm(StateJointCurrent);

    SetRobotControlState_psm(state);
}

//This function sends command to the PSM
void mtsDelayPSM::SetPositionCartesian(const prmPositionCartesianSet & newPosition){

  //Send the first one accross because otherwise it crashes
  if(m_SetPositionCoordinates_buffer.empty() || mDelay == 0.0){
      cartesian_target_slave = newPosition;
      if(!mRosOnly){
           SetPositionCartesian_psm(newPosition);
          }
   }

    if(mDelay != 0.0){
      m_SetPositionCoordinates_buffer.push_back(newPosition);
      m_SetPosition_TimeStamps_buffer.push_back(ros::Time::now());

      if( ros::Time::now() - m_SetPosition_TimeStamps_buffer.front() > ros::Duration(mDelay/1000.0) ){
          //Pop off all of the stuff that is greater than the time delay
          while(ros::Time::now() - m_SetPosition_TimeStamps_buffer.front() > ros::Duration(mDelay/1000.0)){
              m_SetPosition_TimeStamps_buffer.pop_front();
              m_SetPositionCoordinates_buffer.pop_front();

              //Saftey... Hopefully not needed!!!
              if(m_SetPositionCoordinates_buffer.empty()){
                break;
              }
          } //while

          //Only want to send one from the buffer, otherwise stuff gets overloaded
          if(!m_SetPositionCoordinates_buffer.empty()){
              cartesian_target_slave = m_SetPositionCoordinates_buffer.front();
              if(!mRosOnly){
                SetPositionCartesian_psm(m_SetPositionCoordinates_buffer.front());
               }
          }
        }
    }//if (mDelay!= 0.0)
}

//These fucntions send the commands to PSM
void mtsDelayPSM::SetJawPosition(const double & openAngle){
  if(m_SetJaw_buffer.empty() || mDelay == 0.0){
      jaw_target_slave = openAngle;   
      if(!mRosOnly){
     	 SetJawPosition_psm(openAngle);
      }
   }

  if(mDelay != 0.0){
    m_SetJaw_buffer.push_back(openAngle);
    m_SetJaw_TimeStamps_buffer.push_back(ros::Time::now());

    if(ros::Time::now() - m_SetJaw_TimeStamps_buffer.front() > ros::Duration(mDelay/1000.0)){
        while(ros::Time::now() - m_SetJaw_TimeStamps_buffer.front() > ros::Duration(mDelay/1000.0)){
            //Pop the front from timestamps and buffer :D
            m_SetJaw_TimeStamps_buffer.pop_front();
            m_SetJaw_buffer.pop_front();

            if(m_SetJaw_buffer.empty()){
              break;
            }

          }

          if( !m_SetJaw_buffer.empty()){
              jaw_target_slave = m_SetJaw_buffer.front();
              if(!mRosOnly){
                SetJawPosition_psm(m_SetJaw_buffer.front());
              }
          }
     }
  } //if (mDelay!= 0.0)

}

void mtsDelayPSM::SetDelay(const double & delay){
      ClearBuffers();
      mDelay = delay;
}

void mtsDelayPSM::SetRosOnly(const bool & rosOnly){
	//std::cout << rosOnly << std::endl; 
     ClearBuffers();
     mRosOnly = rosOnly;
}

void mtsDelayPSM::PSMErrorEventHandler(const std::string & message)
{
    MessageEvents.Error(message);
}


void mtsDelayPSM::SetPositionCartesian_tele(const prmPositionCartesianSet & newPosition)
{
    cartesian_target_master = newPosition;
    if( !mRosOnly || m_firstTime){
        m_firstTime = false;
        SetPositionCartesian(newPosition);
    }
}

void mtsDelayPSM::SetJawPosition_tele(const double & openAngle)
{
   jaw_target_master = openAngle;
   if( !mRosOnly){
       SetJawPosition(openAngle);
   }
}

void mtsDelayPSM::ClearBuffers(){
  m_firstTime = true;

  m_GetJoint_buffer.clear();
  m_GetJoint_TimeStamps_buffer.clear();
  m_GetPositionCoordinates_buffer.clear();
  m_GetPosition_TimeStamps_buffer.clear();

  m_SetPositionCoordinates_buffer.clear();
  m_SetPosition_TimeStamps_buffer.clear();
  m_SetJaw_buffer.clear();
  m_SetJaw_TimeStamps_buffer.clear();

}
