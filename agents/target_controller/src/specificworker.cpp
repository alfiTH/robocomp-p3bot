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
	//G->write_to_json_file("./"+agent_name+".json");
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

	/***
	Custom Widget
	In addition to the predefined viewers, Graph Viewer allows you to add various widgets designed by the developer.
	The add_custom_widget_to_dock method is used. This widget can be defined like any other Qt widget,
	either with a QtDesigner or directly from scratch in a class of its own.
	The add_custom_widget_to_dock method receives a name for the widget and a reference to the class instance.
	***/

	graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);

	
	targetSelector_ui.setupUi(&targetSelector_widget);
	connect(
        targetSelector_ui.pushTarget,  // Objeto botón 
        &QPushButton::clicked,  // Señal
        this,  // Objeto receptor
        &SpecificWorker::on_pushTarget_clicked  // Slot
    );

	graph_viewer->add_custom_widget_to_dock("TargetSelector", &targetSelector_widget);



}

void SpecificWorker::on_pushTarget_clicked() {
    // 1. Obtener la selección del combobox
    QString selection = targetSelector_ui.selectorTarget->currentText();
    qDebug() << "Change to:" << selection;

    // 2. Obtener los nodos del grafo
    auto robot_node = G->get_node("robot");
    auto target_node = G->get_node(selection.toStdString()); // Convertir QString a std::string

    if (robot_node.has_value() && (target_node.has_value() || selection == "Without Target")) {
        // 3. Primero eliminar cualquier edge target existente del robot
        auto hasTarget = G->get_edges_by_type("TARGET");
        for (auto &&target : hasTarget) {
            if (G->get_name_from_id(target.from()).value()=="robot") {
                G->delete_edge(target.from(), target.to(), "TARGET");
            }
        }

		if (selection != "Without Target"){
			// 4. Crear y añadir el nuevo edge target
			DSR::Edge newTarget;
			newTarget.from(robot_node.value().id());
			newTarget.to(target_node.value().id());
			newTarget.type("TARGET");
			
			// // Añadir atributos si son necesarios
			// std::map<std::string, std::variant<ValType>> attrs;
			// attrs["pos_x"] = 0.0f;
			// attrs["pos_y"] = 0.0f;
			// attrs["pos_z"] = 0.0f;
			// newTarget.attrs(attrs);
			
			G->insert_or_assign_edge(newTarget);
			qDebug() << "Nuevo target establecido:" << selection;
		}  
    } else {
        qDebug() << "Error: Nodos no encontrados";
    }
}



void SpecificWorker::compute()
{
    std::cout << "Compute worker" << std::endl;
	//computeCODE
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
    //    if (img.empty())
    //        emit goToEmergency()
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
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



