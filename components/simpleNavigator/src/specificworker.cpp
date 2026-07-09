/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();



    //initializeCODE
    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 
}



void SpecificWorker::compute()
{
    if(not follow_robot)
        return;

    // targetPose is a relative offset in robot frame, refreshed continuously by the VR component.
    // We map it directly to speeds — no odometry, no feedback loop.
    // The VR keeps resending the offset (carrot), so the robot always chases it.

    constexpr float MAX_V  = 500.f;   // mm/s
    constexpr float MAX_W  = 1.0f;    // rad/s
    // Scale: distance (mm) or angle (rad) at which the robot reaches ~76% of max speed.
    constexpr float SCALE_V = 600.f;  // mm
    constexpr float SCALE_W = 1.2f;   // rad

    float vx = MAX_V * tanh(targetPose[0] / SCALE_V);
    float vy = MAX_V * tanh(targetPose[1] / SCALE_V);

    // Suppress rotation when angle is near ±π (ambiguous wrap-around region)
    constexpr float DEAD_W = 0.1f;
    float r = targetPose[2];
    float w = (std::abs(r) < DEAD_W || std::abs(r) > M_PI - DEAD_W)
              ? 0.f
              : MAX_W * tanh(r / SCALE_W);

    std::cout << "target: " << targetPose[0] << " " << targetPose[1] << " " << targetPose[2]
              << "  speeds: " << vx << " " << vy << " " << w << std::endl;

    try
    {
        omnirobot_proxy->setSpeedBase(-vy, vx, w);
    }
    catch(const Ice::Exception &e)
    {
        std::cout << "Error writing Omnirobot: " << e << std::endl;
    }
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

RoboCompNavigator::LayoutData SpecificWorker::Navigator_getLayout()
{
	RoboCompNavigator::LayoutData ret{};
	//implementCODE

	return ret;
}

RoboCompNavigator::Result SpecificWorker::Navigator_getPath(RoboCompNavigator::TPoint source, RoboCompNavigator::TPoint target, float safety)
{
	RoboCompNavigator::Result ret{};
	//implementCODE

	return ret;
}

RoboCompNavigator::TPose SpecificWorker::Navigator_getRobotPose()
{
	return {0.f, 0.f, 0.f};
}

RoboCompNavigator::NavigationStatus SpecificWorker::Navigator_getStatus()
{
	RoboCompNavigator::NavigationStatus ret{};
	//implementCODE

	return ret;
}

RoboCompNavigator::TPoint SpecificWorker::Navigator_gotoObject(std::string object)
{
	RoboCompNavigator::TPoint ret{};
	//implementCODE

	return ret;
}

RoboCompNavigator::TPoint SpecificWorker::Navigator_gotoPoint(RoboCompNavigator::TPoint target)
{
	RoboCompNavigator::TPoint ret{};
	//implementCODE

	return ret;
}

RoboCompNavigator::TPose SpecificWorker::Navigator_gotoPose(RoboCompNavigator::TPose pose)
{
	// pose.x/y are relative offsets in robot frame (mm), pose.r is relative rotation (rad).
	// Mapping: Navigator.y → OmniRobot advz (forward), Navigator.x → OmniRobot advx (lateral).
	targetPose[0] = pose.y;
	targetPose[1] = pose.x;
	targetPose[2] = pose.r + M_PI;
	if(std::abs(targetPose[2]) > M_PI)
		targetPose[2] -= 2*M_PI;
	follow_robot = true;
	return pose;
}

void SpecificWorker::Navigator_resume()
{
	//implementCODE

}

void SpecificWorker::Navigator_stop()
{
	follow_robot = false;
	try { omnirobot_proxy->stopBase(); }
	catch(const Ice::Exception &e)
	{ std::cout << "Error stopping Omnirobot: " << e << std::endl; }
}



/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompNavigator you can use this types:
// RoboCompNavigator::TPoint
// RoboCompNavigator::Result
// RoboCompNavigator::Pose
// RoboCompNavigator::NavigationStatus
// RoboCompNavigator::TObject
// RoboCompNavigator::LayoutData
// RoboCompNavigator::TPose

