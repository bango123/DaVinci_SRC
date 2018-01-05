#ifndef MTSINTUITIVERESEARCHKITDELAYPSM_H
#define MTSINTUITIVERESEARCHKITDELAYPSM_H

#include <sawIntuitiveResearchKit/mtsStateMachine.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmStateJoint.h>

#include <ros/ros.h>
#include <boost/circular_buffer.hpp>

// always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>


class CISST_EXPORT mtsDelayPSM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsDelayPSM(const std::string & componentName, const double periodInSeconds);
    mtsDelayPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsDelayPSM();

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);


protected:
    double mDelay;
    bool   mRosOnly;
    bool m_firstTime = true;

	//Buffers to delay everything
    boost::circular_buffer<prmStateJoint> m_GetJoint_buffer                          = boost::circular_buffer<prmStateJoint>(1000);
    boost::circular_buffer<ros::Time> m_GetJoint_TimeStamps_buffer                   = boost::circular_buffer<ros::Time>(1000);
	
    boost::circular_buffer<prmPositionCartesianGet> m_GetPositionCoordinates_buffer  = boost::circular_buffer<prmPositionCartesianGet>(1000);
    boost::circular_buffer<ros::Time> m_GetPosition_TimeStamps_buffer                = boost::circular_buffer<ros::Time>(1000);

    boost::circular_buffer<prmPositionCartesianSet> m_SetPositionCoordinates_buffer  = boost::circular_buffer<prmPositionCartesianSet>(1000);
    boost::circular_buffer<ros::Time> m_SetPosition_TimeStamps_buffer                = boost::circular_buffer<ros::Time>(1000);

    boost::circular_buffer<double> m_SetJaw_buffer                                   = boost::circular_buffer<double>(1000);
    boost::circular_buffer<ros::Time> m_SetJaw_TimeStamps_buffer                     = boost::circular_buffer<ros::Time>(1000);

    void Init(void);
    void ClearBuffers();
	
//PSM facing functions/direct control
    void SetRobotControlState(const std::string & state);
    void GetRobotControlState(std::string & state) const;

    void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    void GetPositionCartesian(prmPositionCartesianGet & currentPosition) const;
    void SetJawPosition(const double & openAngle);
    void SetDelay(const double & delay);
    void SetRosOnly(const bool & rosOnly);
    void PSMErrorEventHandler(const std::string & message);

//Tele-op facing functions only
    void SetPositionCartesian_tele(const prmPositionCartesianSet & newPosition);
    void SetJawPosition_tele      (const double & openAngle);

   

// Functions used by the delayPSM from the PSM arm
     mtsFunctionRead  GetStateJoint_psm;
     mtsFunctionRead  GetPositionCartesian_psm;
     mtsFunctionWrite SetPositionCartesian_psm;
     mtsFunctionWrite SetJawPosition_psm;
 
     mtsFunctionRead  GetRobotControlState_psm;
     mtsFunctionWrite SetRobotControlState_psm;
     mtsFunctionWrite SetDelay_psm;

	//Used about the masters known state of the PSM position
    prmPositionCartesianGet CartesianCurrent;
    prmStateJoint           StateJointCurrent;

    // Functions for events
    struct {
        mtsFunctionWrite Status;
        mtsFunctionWrite Warning;
        mtsFunctionWrite Error;
        mtsFunctionWrite RobotState;
    } MessageEvents;

    //mtsIntuitiveResearchKitArmTypes::RobotStateType RobotState;

	//Used as temp variables for buffering!!
    prmPositionCartesianGet CartesianGetParam;
    prmStateJoint           StateJointGetParam;

	//Used as states in the state table to hold input info
    prmPositionCartesianSet cartesian_target_master;
    double jaw_target_master;
	//Used as states in the state table to hold output info
    prmPositionCartesianSet cartesian_target_slave;
    double jaw_target_slave;
  
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDelayPSM);


#endif // MTSINTUITIVERESEARCHKITDELAYPSM_H
