/*
 * Copyright (C) 2013-2015 Istituto Italiano di Tecnologia RBCS & ADVR & iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ShowModelZMP.hh"


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

#include <yarp/os/LogStream.h>
#include <kdl/tree.hpp>
#include <sstream>
#include <algorithm>
#include <map>
#include <stack>
#include <iostream>
#include <cassert>

#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/utils.hpp>
#include <vector>

using namespace std;
using namespace iDynTree ;

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN (ShowModelZMP)

		ShowModelZMP::ShowModelZMP()
: m_zmpOutputPort(0)
{
	yarp::os::Network::init();
}

ShowModelZMP::~ShowModelZMP()
{
	if (m_zmpOutputPort) {
		m_zmpOutputPort->interrupt();
		m_zmpOutputPort->close();
		delete m_zmpOutputPort;
		m_zmpOutputPort = 0;
	}
	gazebo::event::Events::DisconnectWorldUpdateBegin(this->m_updateConnection);
	yarp::os::Network::fini();
}

void ShowModelZMP::UpdateChild()
{

	/*static double time_init = yarp::os::Time::now() ;



		if(open_file){
		double time_init = yarp::os::Time::now() ;
		}
		open_file = false ;

		double time_current = yarp::os::Time::now();
		//cout << "time_current" << time_current << endl;

		double time_diff = time_current-time_init ;
		//	cout <<"DIFF = " <<diff << endl ;*/

	gazebo::physics::Link_V links = m_myModel->GetLinks();
	gazebo::physics::Joint_V joints = m_myModel-> GetJoints();

	yarp::sig::Vector q(joints.size());
	yarp::sig::Vector dq(joints.size());

	//cout << links.size() << endl;
	//cout << joints.size() << endl;
	/*yarp::sig::Vector q(joints.size());
	yarp::sig::Vector dq(joints.size());*/

	double mass_acc = 0.0;
	gazebo::math::Vector3 weighted_position_acc = gazebo::math::Pose::Zero.pos;
	gazebo::math::Vector3 angular_velocity_R = gazebo::math::Pose::Zero.pos;
	gazebo::math::Vector3 angular_mom_R = gazebo::math::Pose::Zero.pos;
	gazebo::math::Vector3 angular_acc_R = gazebo::math::Pose::Zero.pos;

	gazebo::math::Vector3 linea_vel_R = gazebo::math::Pose::Zero.pos;

	gazebo::math::Vector3 H_d_COM2 = gazebo::math::Pose::Zero.pos;


	ofstream link_com_icub ;
	link_com_icub.open( "Link_COM_data.txt", std::ofstream::out | std::ofstream::app );

	//cout << "Links size ====== "<<links.size() << endl;

	for(unsigned int i = 0; i < links.size(); ++i)
	{
		gazebo::physics::LinkPtr link = links[i];
		gazebo::math::Vector3 worldCoG_i = link->GetWorldCoGPose().pos;
		gazebo::math::Vector3 world_AngVel_i = link->GetWorldAngularVel();
		gazebo::math::Vector3 world_LinVel_i = link->GetWorldLinearVel();

		gazebo::math::Vector3 world_ang_mom_i = link->GetWorldAngularMomentum();
		gazebo::math::Vector3 world_ang_acc_i = link->GetWorldAngularAccel();

		gazebo::math::Matrix3 link_inertia_i = link-> GetWorldInertiaMatrix();

		gazebo::math::Vector3 link_ang_acc_i = link_inertia_i * world_ang_acc_i ;
		gazebo::math::Vector3 H_d_COM2_i =  world_AngVel_i.Cross(world_ang_mom_i);
		double mass_i = link->GetInertial()->GetMass();


		//angular_velocity_R += world_AngVel_i ;
		linea_vel_R += mass_i*world_LinVel_i;
		angular_mom_R += world_ang_mom_i ;
		angular_acc_R += link_ang_acc_i ;
		H_d_COM2 += H_d_COM2_i ;
		weighted_position_acc += mass_i*worldCoG_i;
		mass_acc += mass_i;

/*

		//////////////////////////// DynTree
		//////////////////////////////////////////////////
		//yarp::sig::Vector robot_mm_acc(6)
		double tree_mass ;
		KDL::RigidBodyInertia Handle = ICUB_ROBOT_TREE.getKDLUndirectedTree().getLink(i)->getInertia();
				double Mass_i = Handle.getMass();

				tree_mass += Mass_i ;

		gazebo::physics::JointPtr joint = joints[i];
		gazebo::math::Angle joint_ang  = joint->GetAngle(i);
		double joint_ang_r = joint_ang.Radian();
		q(i) = joint_ang_r ;
		dq(i)  = joint->GetVelocity(i);


		yarp::sig::Vector link_vel_Tree(6);
		yarp::sig::Vector link_acc_Tree(6);
		yarp::sig::Vector link_mm_Tree(6);
		yarp::sig::Vector tree_ang_vel(6);
		yarp::sig::Vector tree_ang_acc(6);

*/

		//link_vel_Tree = ICUB_ROBOT_TREE.getVel(i,false)   ;
		//link_acc_Tree = ICUB_ROBOT_TREE.getAcc(i,false) ;

		//tree_ang_vel = yarp::math::operator +=(tree_ang_vel,link_vel_Tree);
		//tree_ang_acc = yarp::math::operator +=(tree_ang_acc,link_acc_Tree);


		if (m_count == 1.0){

			//cout << i<< "    "<<q(i) << endl;
			link_com_icub << i << "--Link Name:  "<< link->GetName() << "......Link mass = "  <<  link->GetInertial()->GetMass() <<  " ----Link COG = "<<link->GetWorldCoGPose().pos   <<  endl ;

		}

	//	if (abs(world_LinVel_i.z) > 0.18 || abs(world_LinVel_i.x) > 0.15 || abs(world_LinVel_i.y) > 0.15) {
		/*if(true){
						cout << i << "--Link Name:  "<< link->GetName() << "   Angular Velocity =   "<< world_ang_acc_i << endl;
						cout << "Next Step" << endl;
					}*/
	}


//ofstream angular_vel_tree ;
//angular_vel_tree.open( "Angular_vel_tree.txt", std::ofstream::out | std::ofstream::app );


//angular_vel_tree << "Robot Angular Velocity    " << angular_mom_R << endl;
	//cout << "Robot Angular Velocity    " << linea_vel_R << endl;

	//cout << "Robot COM Vel " << " x_dot = "<<robot_vel_tot(0) <<  " y_dot = "<< robot_vel_tot(1) <<  " z_dot = "<< robot_vel_tot(2) << endl ;

	//Vel_data_icubtree << robot_vel(0) <<"----y = " << robot_vel(1) <<"------z = " <<robot_vel(2) << endl;

//cout<<"angular_velocity_acc.y "<< angular_acc_R<<endl;
	bot2.addDouble(H_d_COM2.x);
	bot2.addDouble(H_d_COM2.y);
	bot2.addDouble(H_d_COM2.z);
		//bot2.addDouble(angular_velocity_acc.y);
		//bot2.addDouble(angular_velocity_acc.z);

		m_AngularMomentum.write(bot2);

		//	cout << bot2.toString() << endl;

	/*if (m_AngularMomentumB) {
	yarp::os::Bottle &bot3 = m_AngularMomentumB->prepare();
	bot3.addDouble(H_d_COM2.x);
	bot3.addDouble(H_d_COM2.y);
	bot3.addDouble(H_d_COM2.z);
	//cout << bot2.toString() << endl;
	m_AngularMomentumB->write();
	}*/

	gazebo::math::Vector3 worldCoGLinVel = linea_vel_R/mass_acc ;
	gazebo::math::Vector3 worldCoGModel = weighted_position_acc/mass_acc;
	gazebo::math::Vector3 worldZMPModel = gazebo::math::Pose::Zero.pos;

	ofstream gz_data_x ;
	ofstream gz_data_y ;
	ofstream gz_data_z ;


	/*gz_data_x.open( "LinVel_x_gz.txt", std::ofstream::out | std::ofstream::app );
	gz_data_y.open( "LinVel_y_gz.txt", std::ofstream::out | std::ofstream::app );
	gz_data_z.open( "LinVel_z_gz.txt", std::ofstream::out | std::ofstream::app );
if (double time_diff = yarp::os::Time::now() > 10.0 ){

	gz_data_x <<  worldCoGLinVel.x << endl;
	gz_data_y <<  worldCoGLinVel.y << endl;
	gz_data_z <<  worldCoGLinVel.z << endl;
}


*/
	if (m_count ==1.0){

		cout << "FROM SDF FILE:     "<< worldCoGModel.x <<"----y = " << worldCoGModel.y <<"------z = " <<worldCoGModel.z << endl;
		cout << "MASS SDF = "<<mass_acc << endl ;

		m_count = 2.0 ;
		}

	// SG Filter
	int order = 2;
	int winlen = 11;
	SGF::real sample_time = 1e-3; // this is simply a float or double, you can change it in the header sg_filter.h if yo u want
	SGF::ScalarSavitzkyGolayFilter filter_x(order, winlen, sample_time);

	// add some data
	SGF::real x_data  = worldCoGModel.x;
	SGF::real y_data  = worldCoGModel.y;
	SGF::real z_data  = worldCoGModel.z;

	SGF::real x_gen,x_d_gen,x_dd_gen;
	SGF::real y_gen,y_d_gen,y_dd_gen;
	SGF::real z_gen,z_d_gen,z_dd_gen;


	// Filter x_position
	filter_x.AddData(x_data);
	float tmp;
	filter_x.GetOutput(0, 0, tmp);
	x_gen = tmp;
	filter_x.GetOutput(0, 1, tmp);
	x_d_gen = tmp;
	filter_x.GetOutput(0, 2, tmp);
	x_dd_gen = tmp;

	//std::cout << x_gen << " " << x_d_gen << " " << x_dd_gen << std::endl;


	// Filter y_position
	SGF::ScalarSavitzkyGolayFilter filter_y(order, winlen, sample_time);
	filter_y.AddData(y_data);
	float tmp1 ;
	filter_y.GetOutput(0, 0, tmp1);
	y_gen = tmp1;
	filter_y.GetOutput(0, 1, tmp1);
	y_d_gen = tmp1;
	filter_y.GetOutput(0, 2, tmp1);
	y_dd_gen = tmp1;

	//std::cout << y_gen << " " << y_d_gen << " " << y_dd_gen << std::endl;

	// Filter z_position
	SGF::ScalarSavitzkyGolayFilter filter_z(order, winlen, sample_time);
	filter_z.AddData(z_data);
	float tmp2 ;
	filter_z.GetOutput(0, 0, tmp2);
	z_gen = tmp2;
	filter_z.GetOutput(0, 1, tmp2);
	z_d_gen = tmp2;
	filter_z.GetOutput(0, 2, tmp2);
	z_dd_gen = tmp2;

	// std::cout << z_gen << " " << z_d_gen << " " << z_dd_gen << std::endl;

	gazebo::math::Vector3 Lin_Velocity_COM = gazebo::math::Pose::Zero.pos;
	Lin_Velocity_COM.x = x_d_gen ;
	Lin_Velocity_COM.y = y_d_gen ;
	Lin_Velocity_COM.z = z_d_gen ;

	ofstream sg_data ;
	sg_data.open( "LinVel_sg.txt", std::ofstream::out | std::ofstream::app );

	sg_data << "Linear Velocity = "<< Lin_Velocity_COM << "-----Magnitude = "  << Lin_Velocity_COM.GetLength() << endl ;

	gazebo::math::Vector3 Lin_Acc_COM = gazebo::math::Pose::Zero.pos;
	Lin_Acc_COM.x = x_d_gen ;
	Lin_Acc_COM.y = y_d_gen ;
	Lin_Acc_COM.z = z_d_gen ;

	//// Calculate the linear momentum for COM (P_COM)
	gazebo::math::Vector3 P_COM = gazebo::math::Pose::Zero.pos;
	P_COM = mass_acc  * Lin_Velocity_COM ;

	//// Calculate the first term of angular momentum (H1_COM) and its derivative (H_dot_m) for COM
	gazebo::math::Vector3 H1_COM = gazebo::math::Pose::Zero.pos;
	H1_COM = worldCoGModel.Cross(mass_acc * Lin_Velocity_COM) ;

	gazebo::math::Vector3 P_d_COM = gazebo::math::Pose::Zero.pos;
	P_d_COM = mass_acc * Lin_Acc_COM ;

	gazebo::math::Vector3 H_d_COM1 = gazebo::math::Pose::Zero.pos;
	H_d_COM1 =  worldCoGModel.Cross(mass_acc * Lin_Acc_COM) ;

	gazebo::math::Vector3 H_d_COM = gazebo::math::Pose::Zero.pos;

	H_d_COM = H_d_COM1 + H_d_COM2 ;

	//cout << P_d_COM <<endl;


	// Reading External Wrench Command
	yarp::os::Bottle *b = m_ExternalCmdInput.read(false);
	if (b!=NULL)
	{
		//printf("Got message: %s\n", b->toString().c_str());
	}

	// Reading the COM Position of Target Link
	yarp::os::Bottle *b2 = m_LinkPosInput.read(false);


	gazebo::math::Vector3 extforce(b->get ( 0 ).asDouble(),b->get ( 1 ).asDouble(),b->get ( 2 ).asDouble());

	gazebo::math::Vector3 exttorque(b->get ( 3 ).asDouble(),b->get ( 4 ).asDouble(),b->get ( 5 ).asDouble());

	gazebo::math::Vector3 target_position(b2->get ( 0 ).asDouble(),b2->get ( 1 ).asDouble(),b2->get ( 2 ).asDouble());


	gazebo::math::Vector3 moment_due_force = target_position.Cross(extforce) ;


	double g = 9.8 ;
	worldZMPModel.x = (exttorque.y + moment_due_force.y + worldCoGModel.x * mass_acc * g - H_d_COM.y)  /(mass_acc*g - extforce.z + P_d_COM.z) ;
	worldZMPModel.y = ( - exttorque.x - moment_due_force.x  + worldCoGModel.y * mass_acc * g + H_d_COM.x)/( mass_acc*g - extforce.z + P_d_COM.z) ;

	worldZMPModel.z = 0.0;

	//cout << worldZMPModel << endl;

	if (m_zmpOutputPort) {
		yarp::os::Bottle &bot = m_zmpOutputPort->prepare();
		bot.clear();
		bot.addDouble(worldZMPModel.x);
		bot.addDouble(worldZMPModel.y);
		bot.addDouble(worldZMPModel.z);
		m_zmpOutputPort->write();
	}

	gazebo::math::Pose WorldZMPPose = gazebo::math::Pose::Zero;
	WorldZMPPose.pos = worldZMPModel;


#if GAZEBO_MAJOR_VERSION >= 7
	msgs::Set(m_visualMsg.mutable_pose(), WorldZMPPose.Ign());
#else
	msgs::Set(m_visualMsg.mutable_pose(), WorldZMPPose);
#endif
	msgs::Set(m_visualMsg.mutable_material()->mutable_ambient(),common::Color(1,0,0,0.3));
	m_visualMsg.set_visible(1);
	m_visPub->Publish(m_visualMsg);




}


void ShowModelZMP::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{


m_count = 1.0 ;

	// Check if yarp network is active;
	if (!yarp::os::Network::checkNetwork()) {
		yError("ERROR Yarp Network was not found active");
		return;
	}


	m_ExternalCmdInput.open("/ExternalCommandRec") ;
	m_LinkPosInput.open("/LinkPosInput");
	yarp.connect("/ExternalCommand","/ExternalCommandRec");
	yarp.connect("/TargetLinkPosition","/LinkPosInput") ;


	// creat a new port to plot the angular momentum
	m_AngularMomentum.open("/RobotAngularMomentum");


	// What is the parent name??
	this->m_modelScope = _model->GetScopedName();

	std::string port_name = "/" + this->m_modelScope + "/ZMPInWorld:o";
	m_zmpOutputPort = new yarp::os::BufferedPort<yarp::os::Bottle>();
	if (!m_zmpOutputPort || !m_zmpOutputPort->open(port_name)) {
		yError("Could not open port %s", port_name.c_str());
		return;
	}

	std::string port_name1 = "/RobotAngularMomentumBuf";
	m_AngularMomentumB = new yarp::os::BufferedPort<yarp::os::Bottle>();
		if (!m_AngularMomentumB || !m_AngularMomentumB->open(port_name1)) {
			yError("Could not open port %s", port_name.c_str());
			return;
		}

	// Copy the pointer to the model to access later from UpdateChild
	this->m_myModel = _model;
	bool configuration_loaded = false;

	bool ok = ICUB_ROBOT_TREE.loadURDFModel("/home/neda/iCube/gazebo-yarp-plugins/plugins/showmodelzmp/src/icub_with_hands_dummy.urdf");

	cout <<"DOF = " << ICUB_ROBOT_TREE.getNrOfDOFs() << endl;

		cout <<"N. of Links = " <<ICUB_ROBOT_TREE.getNrOfLinks() << endl;
		ofstream com_data_icub ;
		com_data_icub.open( "LINK_COM_URDF.txt", std::ofstream::out | std::ofstream::app );


		for(unsigned int i = 0; i < ICUB_ROBOT_TREE.getNrOfLinks(); ++i)
				{
					KDL::RigidBodyInertia HHandle = ICUB_ROBOT_TREE.getKDLUndirectedTree().getLink(i)->getInertia();

					com_data_icub << i << "-   "<<ICUB_ROBOT_TREE.getKDLUndirectedTree().getLink(i)->getName()<< "----------Link Mass = "<< HHandle.getMass();
					com_data_icub <<"------Link COG =  "<<HHandle.getCOG().data[0]<<"   "<< HHandle.getCOG().data[1]<<"   "<< HHandle.getCOG().data[2]<<endl;
				}

	/// ############ TRYING TO MODIFY VISUALS #######################################################

	this->m_node = transport::NodePtr(new gazebo::transport::Node());

	this->m_node->Init ( _model->GetWorld()->GetName() );
	m_visPub = this->m_node->Advertise<msgs::Visual> ("~/visual", 10);

	// Set the visual's name. This should be unique.
	m_visualMsg.set_name ("__ZMP_VISUAL__");

	// Set the visual's parent. This visual will be attached to the parent
	m_visualMsg.set_parent_name ( _model->GetScopedName() );

	// Create a cylinder
	msgs::Geometry *geomMsg = m_visualMsg.mutable_geometry();
	geomMsg->set_type ( msgs::Geometry::SPHERE );
	geomMsg->mutable_sphere()->set_radius ( 0.02 );

	// Don't cast shadows
	m_visualMsg.set_cast_shadows ( false );




	/// ############ END: TRYING TO MODIFY VISUALS #####################################################

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ShowModelZMP::UpdateChild, this ) );

}


}








