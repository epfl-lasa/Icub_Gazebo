/*
 * Copyright (C) 2013-2015 Istituto Italiano di Tecnologia RBCS & ADVR & iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "LoadURDF.hh"


#include "eigen3/Eigen/Dense"
#include "sg_filter.h"

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <fstream>

#include <iDynTree/ModelIO/URDFModelImport.h>
#include <iDynTree/KinDynComputations.h>

#include <iCub/iDynTree/DynTree.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <iCub/iDynTree/yarp_iDynTree.h>

//Loops from KDL_CoDyCo
#include <kdl_codyco/position_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/jacobian_loops.hpp>
#include <kdl_codyco/com_loops.hpp>
#include <kdl_codyco/crba_loops.hpp>
#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/regressor_utils.hpp>
#include <kdl_codyco/generalizedjntpositions.hpp>

#include "iDynTree/Core/Transform.h"
#include "iDynTree/Core/Wrench.h"
#include "iDynTree/Core/SpatialForceVector.h"

#include <kdl_codyco/regressor_utils.hpp>
#include <kdl_codyco/regressors/dirl_utils.hpp>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>



#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>


using namespace std;
using namespace iDynTree ;

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN (LoadURDF)

		LoadURDF::LoadURDF()
{
	yarp::os::Network::init();
}

LoadURDF::~LoadURDF()
{
	gazebo::event::Events::DisconnectWorldUpdateBegin(this->m_updateConnection);
	yarp::os::Network::fini();
}

void LoadURDF::UpdateChild()
{

	gazebo::physics::Joint_V joints = m_myModel-> GetJoints();
	gazebo::physics::Link_V links = m_myModel->GetLinks();

	yarp::sig::Vector q(joints.size());
	yarp::sig::Vector dq(joints.size());
/*
	for(unsigned int i = 0; i < links.size(); ++i)
	{


		gazebo::physics::JointPtr joint = joints[i];
		gazebo::math::Angle joint_ang  = joint->GetAngle(i);

		//cout << "AAAAAAAAAAAAAa" << endl;
		double joint_angle = joint_ang.Radian();
		q(i) = joint_angle ;
		dq(i)  = joint->GetVelocity(i);


	//		cout << i << "-----" <<joint_angle << "====" << dq(i) << endl ;

	}*/
//cout << links.size() << endl;

	//ICUB_ROBOT_TREE.setAng(q);
	//ICUB_ROBOT_TREE.setDAng(dq);


	//Vel_data_icubtree.open( "TREE_Vel_data.txt", std::ofstream::out | std::ofstream::app );

	yarp::sig::Vector robot_mm_acc(6) ;//= yarp::math::zeros(int 6) ;
	yarp::sig::Vector link_vel(6);
	yarp::sig::Vector link_acc(6);
	yarp::sig::Vector link_mm(6);
	yarp::sig::Vector robot_ang_vel(6);

	double m_acc=0.0 ;

	for(unsigned int i = 0; i < ICUB_ROBOT_TREE.getNrOfLinks(); ++i)
	{
		KDL::RigidBodyInertia Handle = ICUB_ROBOT_TREE.getKDLUndirectedTree().getLink(i)->getInertia();
		double Mass_i = Handle.getMass();

		//link_vel = ICUB_ROBOT_TREE.getVel(i,false)   ;
		//link_acc = ICUB_ROBOT_TREE.getAcc(i,false) ;

		//link_mm = yarp::math::operator* (link_vel,Mass_i);
		//yarp::sig::Vector link_mm = link_vel * Mass_i ;

		robot_mm_acc = yarp::math::operator+=(robot_mm_acc,link_mm);

	//	robot_ang_vel = yarp::math::operator +=(robot_ang_vel,link_vel);

		//robot_mm_acc += link_mm ;
		m_acc += Handle.getMass();

	}

		//yarp::sig::Vector robot_vel_com = yarp::math::operator/(robot_mm_acc, m_acc) ;
	//cout << "Robot COM Vel " << " x_dot = "<<robot_vel_tot(0) <<  " y_dot = "<< robot_vel_tot(1) <<  " z_dot = "<< robot_vel_tot(2) << endl ;

	//Vel_data_icubtree << robot_vel(0) <<"----y = " << robot_vel(1) <<"------z = " <<robot_vel(2) << endl;
	yarp::sig::Vector COM_ICUBTREE(3);

	COM_ICUBTREE = ICUB_ROBOT_TREE.getCOM(-1);


	if (m_count ==1.0){

		cout << "FROM URDF FILE:    "<<"x= "<<COM_ICUBTREE(0) <<"----y = " << COM_ICUBTREE(1) <<"------z = " <<COM_ICUBTREE(2) << endl;
		cout << "MASS from URDF file = "<< m_acc << endl ;

		m_count = 2.0 ;

		}

	bot.addDouble(robot_ang_vel(3));
	bot.addDouble(robot_ang_vel(4));
	bot.addDouble(robot_ang_vel(5));

	m_outputport.write(bot);


	ofstream angular_vel_urdf ;
	angular_vel_urdf.open( "Angular_vel_urdf.txt", std::ofstream::out | std::ofstream::app );
	angular_vel_urdf << "Robot Angular Velocity    " << robot_ang_vel(3) << "    "<<  robot_ang_vel(4) << "    "<<  robot_ang_vel(5) << endl;


}


void LoadURDF::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

m_count = 1.0 ;

	// Check if yarp network is active;
	if (!yarp::os::Network::checkNetwork()) {
		yError("ERROR Yarp Network was not found active");
		return;
	}

	m_outputport.open("/AngularVel_URDF");
	ofstream com_data_icub ;
	double totalMass = 0.0 ;

		com_data_icub.open( "LINK_COM_URDF.txt", std::ofstream::out | std::ofstream::app );


	bool ok = ICUB_ROBOT_TREE.loadURDFModel("/home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelzmp/src/icub_dummy.urdf");


	iDynTree::KinDynComputations urdf_kindyncomp;

	iDynTree::HighLevel::DynamicsComputations urdf_dyncomp;

	//bool okk = urdf_dyncomp.loadRobotModelFromFile("/home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelzmp/src/icub_with_hands.urdf");

	//bool okkk = urdf_kindyncomp.loadRobotModelFromFile("/home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelzmp/src/iCubHeidelberg01.urdf");

	//const iDynTree::Model & model = urdf_kindyncomp.getRobotModel();


	//cout << "x_com   "<< urdf_dyncomp.getCenterOfMass()(0)<<  "   y_com"<<urdf_dyncomp.getCenterOfMass()(1) <<"   z_com " <<urdf_dyncomp.getCenterOfMass()(2) << endl ;

	//urdf_dyncomp.getr

	/*for(size_t l=0; l < model.getNrOfLinks(); l++)
	    {
	        totalMass += model.getLink(l)->getInertia().getMass();
	}
	std::cout << "The total mass of model is " << totalMass << " Kg." << std::endl;
*/
/*	for (int i=0; i< ICUB_ROBOT_TREE.getNrOfLinks(); ++i)
	{
	    Position linkComWrtLink = urdf_kindyncomp.getRobotModel().getLink(i)->getInertia().getCenterOfMass();

	  //  Position linkComWrtFrame = frame_H_link*linkComWrtLink;

	    std::cout << "The COM of link  with respect to link  is " << linkComWrtLink.toString() << std::endl;
	}*/
	//iDynTree::Position dofs = urdf_dyncomp.getCenterOfMass();

	//cout << "COM ========"<< dofs(1) << endl;

	//const Traversal & traversal;
	//KDL::CoDyCo::LinkMap::const_iterator link_b = traversal.getNrOfVisitedLinks()

	//	int nnnn = traversal.getNrOfVisitedLinks();

	//	cout << "Nr of visited link "<< nnnn << endl;

//	KDL::CoDyCo::UndirectedTree undirectedTree = ICUB_ROBOT_TREE.getKDLUndirectedTree();

	//KDL::CoDyCO::LinkMap::const_iterator getOrderedLink(int order_number);



	for(unsigned int i = 0; i < ICUB_ROBOT_TREE.getNrOfLinks(); ++i)
		{
			KDL::RigidBodyInertia HHandle = ICUB_ROBOT_TREE.getKDLUndirectedTree().getLink(i)->getInertia();

			com_data_icub << i << "-   "<<ICUB_ROBOT_TREE.getKDLUndirectedTree().getLink(i)->getName()<< "----------Link Mass = "<< HHandle.getMass();
			com_data_icub <<"------Link COG =  "<<HHandle.getCOG().data[0]<<"   "<< HHandle.getCOG().data[1]<<"   "<< HHandle.getCOG().data[2]<<endl;
		}

	com_data_icub.close();

	//iDynTree::SpatialInertia HHHHHH = ICUB_ROBOT_TREE

	if( !ok )
	{
		std::cerr << "Loading urdf file  failed, exiting" << std::endl;

	}
	else{std::cout << "Loading urdf file  SUCCESS" << std::endl;}


	int DoF(ICUB_ROBOT_TREE.getNrOfDOFs());

	cout << ICUB_ROBOT_TREE.getNrOfDOFs() << endl;

	cout << ICUB_ROBOT_TREE.getNrOfLinks() << endl;
//iDynTree::SpatialInertiaRaw

	//iDynTree::SpatialInertia DynamicsComputations::getLinkInertia()
/*
	bool okk = dynComp.loadRobotModelFromFile("/home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelzmp/src/icub_with_hands.urdf");

	iDynTree::Position vv ;
	iDynTree::SpatialInertia m_II ;
	double *Handle;

	m_II =dynComp.getLinkInertia(10);
	Handle=m_II.asMatrix().data();

	cout<<"Handle "<<endl;
	cout<<Handle[24]<<endl;*/




	/* vv = dynComp.getCenterOfMass() ;

	 cout << "AAAAAAAAA" << vv(1) << endl;

	//KDL::Vector COM_ICUBTREE_KDL = ICUB_ROBOT_TREE.getCOMKDL(10);

	//	cout << "AAAAAAAAAAAAAAAAAAAAAAAAa"<< COM_ICUBTREE_KDL(0) << endl;
	//	SpatialInertia

//	cout << ICUB_NEW_MODEL.getNrOfLinks() << endl;

		for(unsigned int i = 0; i < ICUB_ROBOT_TREE.getNrOfLinks(); ++i)
				{
					cout << "LINK :" << ICUB_ROBOT_TREE.getLinkName(i, link_l) << std::endl ;
				}*/



	// Copy the pointer to the model to access later from UpdateChild
		this->m_myModel = _model;
		bool configuration_loaded = false;
	// Listen to the update event. This event is broadcast every
	// simulation iteration.

	this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &LoadURDF::UpdateChild, this ) );

}


}








