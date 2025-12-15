/*
 *    Copyright (C) 2025 by YOUR NAME HERE
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
#include <iostream>
#include <limits>
#include <chrono>
#include <random>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
  this->startup_check_flag = startup_check;
  if (this->startup_check_flag) {
    this->startup_check();
  } else {
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
    if (error.length() > 0) {
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
  numPoints = configLoader.get<int>("NumPoints");
  cloudData.X.resize(numPoints);
  cloudData.Y.resize(numPoints);
  cloudData.Z.resize(numPoints);
  cloudData.R.resize(numPoints);
  cloudData.G.resize(numPoints);
  cloudData.B.resize(numPoints);
}

void SpecificWorker::compute()
{
  std::cout << "Compute worker" << std::endl;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<short> distPos(
      -5000, 5000); // Random position between -5000 and 5000 mm
  std::uniform_int_distribution<unsigned short> distColor(0,
                                                          255); // Random color

  for (int i = 0; i < numPoints; i++) {
    cloudData.X[i] = distPos(gen);
    cloudData.Y[i] = distPos(gen);
    cloudData.Z[i] = distPos(gen);
    cloudData.R[i] = static_cast<unsigned char>(distColor(gen));
    cloudData.G[i] = static_cast<unsigned char>(distColor(gen));
    cloudData.B[i] = static_cast<unsigned char>(distColor(gen));
  }
  cloudData.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
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

RoboCompLidar3D::TColorCloudData SpecificWorker::Lidar3D_getColorCloudData() {
  return cloudData;
}

RoboCompLidar3D::TData
SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len,
                                     int decimationDegreeFactor) {
  RoboCompLidar3D::TData ret{};
  // implementCODE

  return ret;
}

RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{
  RoboCompLidar3D::TDataImage ret{};
	//implementCODE

  return ret;
}

RoboCompLidar3D::TDataCategory SpecificWorker::Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp)
{
  RoboCompLidar3D::TDataCategory ret{};
	//implementCODE

  return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataProyectedInImage(std::string name)
{
  RoboCompLidar3D::TData ret{};
	//implementCODE

  return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor)
{
  RoboCompLidar3D::TData ret{};
	//implementCODE

  return ret;
}



/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

