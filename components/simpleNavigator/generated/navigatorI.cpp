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
#include "navigatorI.h"

NavigatorI::NavigatorI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getLayoutHandlers = {
		[this]() -> RoboCompNavigator::LayoutData {if (worker != nullptr) return worker->Navigator_getLayout(); else throw std::runtime_error("Worker is null");}
	};

	getPathHandlers = {
		[this](auto &a, auto &b, auto &c) -> RoboCompNavigator::Result {if (worker != nullptr) return worker->Navigator_getPath(a, b, c); else throw std::runtime_error("Worker is null");}
	};

	getRobotPoseHandlers = {
		[this]() -> RoboCompNavigator::TPose {if (worker != nullptr) return worker->Navigator_getRobotPose(); else throw std::runtime_error("Worker is null");}
	};

	getStatusHandlers = {
		[this]() -> RoboCompNavigator::NavigationStatus {if (worker != nullptr) return worker->Navigator_getStatus(); else throw std::runtime_error("Worker is null");}
	};

	gotoObjectHandlers = {
		[this](auto &a) -> RoboCompNavigator::TPoint {if (worker != nullptr) return worker->Navigator_gotoObject(a); else throw std::runtime_error("Worker is null");}
	};

	gotoPointHandlers = {
		[this](auto &a) -> RoboCompNavigator::TPoint {if (worker != nullptr) return worker->Navigator_gotoPoint(a); else throw std::runtime_error("Worker is null");}
	};

	gotoPoseHandlers = {
		[this](auto &a) -> RoboCompNavigator::TPose {if (worker != nullptr) return worker->Navigator_gotoPose(a); else throw std::runtime_error("Worker is null");}
	};

	resumeHandlers = {
		[this]() {if (worker != nullptr) worker->Navigator_resume(); else throw std::runtime_error("Worker is null");}
	};

	stopHandlers = {
		[this]() {if (worker != nullptr) worker->Navigator_stop(); else throw std::runtime_error("Worker is null");}
	};

}

NavigatorI::~NavigatorI()
{
}

RoboCompNavigator::LayoutData NavigatorI::getLayout(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getLayoutHandlers.at(id)();
}

RoboCompNavigator::Result NavigatorI::getPath(RoboCompNavigator::TPoint source, RoboCompNavigator::TPoint target, float safety, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getPathHandlers.at(id)(source, target, safety);
}

RoboCompNavigator::TPose NavigatorI::getRobotPose(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getRobotPoseHandlers.at(id)();
}

RoboCompNavigator::NavigationStatus NavigatorI::getStatus(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getStatusHandlers.at(id)();
}

RoboCompNavigator::TPoint NavigatorI::gotoObject(std::string object, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return gotoObjectHandlers.at(id)(object);
}

RoboCompNavigator::TPoint NavigatorI::gotoPoint(RoboCompNavigator::TPoint target, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return gotoPointHandlers.at(id)(target);
}

RoboCompNavigator::TPose NavigatorI::gotoPose(RoboCompNavigator::TPose pose, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return gotoPoseHandlers.at(id)(pose);
}

void NavigatorI::resume(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	resumeHandlers.at(id)();
}

void NavigatorI::stop(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	stopHandlers.at(id)();
}