/*
 * This plugin has been written to calculate the "Zero Moment Point" (ZMP)
 *
 */

#include "ShowModelZMP.hh"
#include "eigen3/Eigen/Dense"
#include <fstream>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/os/LogStream.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <time.h>

using namespace std;

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

	clock_t t_overall;
	t_overall = clock();

	static double time_init = yarp::os::Time::now() ;
	static int time_new = 0 ;


	int time_current = yarp::os::Time::now();

	int time_diff = time_current-time_init ;


	gazebo::physics::Link_V links = m_myModel->GetLinks();
	gazebo::physics::Joint_V joints = m_myModel-> GetJoints();


	double mass_acc = 0.0;
	gazebo::math::Vector3 weighted_position_acc = gazebo::math::Pose::Zero.pos;
	gazebo::math::Vector3 robot_IW = gazebo::math::Pose::Zero.pos;


	//gazebo::math::Vector3 Hd_R2 = gazebo::math::Pose::Zero.pos;   // sigma(w * (IW))
	gazebo::math::Vector3 Hd_R3 = gazebo::math::Pose::Zero.pos;  //  sigma(I Wdot)
	gazebo::math::Vector3 Hd_R1G = gazebo::math::Pose::Zero.pos;  //  sigma(m r2dot)


// input vectors for filtering
	SGF::Vec link_cog(246) ;
	SGF::Vec link_IW(246) ;

	ofstream link_com_icub ;
	link_com_icub.open( "Link_COM_data.txt", std::ofstream::out | std::ofstream::app );


	ofstream link_ang_acc ;
	link_ang_acc.open( "Link_Ang_Acc.txt", std::ofstream::out | std::ofstream::app );

	int j ;


	for(unsigned int i = 0; i < links.size(); ++i)
	{
		gazebo::physics::LinkPtr link = links[i];
		double mass_i = link->GetInertial()->GetMass();
		gazebo::math::Vector3 worldCoG_i = link->GetWorldCoGPose().pos;

		gazebo::math::Vector3 world_AngVel_i = link->GetWorldAngularVel();
		gazebo::math::Vector3 world_LinVel_i = link->GetWorldLinearVel();

		j = 3 * i ;
		link_cog(j) = worldCoG_i.x;
		link_cog(j+1) = worldCoG_i.y;
		link_cog(j+2) = worldCoG_i.z;

		weighted_position_acc += mass_i*worldCoG_i;
	    mass_acc += mass_i;

		gazebo::math::Matrix3 link_inertia_i = link-> GetWorldInertiaMatrix();
		gazebo::math::Vector3 link_IW_i = link_inertia_i * world_AngVel_i ;

		link_IW(j) = link_IW_i.x;
		link_IW(j+1) = link_IW_i.y;
		link_IW(j+2) = link_IW_i.z;

	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////    Filter     //////////////////////////////////////////////////////////////////////
	// SG Filter

	SGF::Vec x_d_gen,x_dd_gen;
	SGF::Vec y_d_gen,y_dd_gen;
	SGF::Vec z_d_gen,z_dd_gen;

	SGF::Vec r_dd_gen ;
	SGF::Vec r_d_gen ;
	SGF::Vec r_gen ;

	for (unsigned int j = 0 ; j< 246 ; j++)
	{
		link_com_icub   << link_cog(j) << " "  ;
	}

	link_com_icub << "\n"<< endl;

	//First filter: Calculate the linear acceleration of each link

    r_dd_gen.resize(dim);
	{
		clock_t t;
		t = clock();
		filter.AddData(link_cog);
		filter.GetOutput(2,r_dd_gen);
		t = clock() - t;
		printf ("1st filter took me %f seconds.\n",((float)t)/CLOCKS_PER_SEC);
	}
	gazebo::math::Vector3 link_linear_acc = gazebo::math::Pose::Zero.pos;
	gazebo::math::Vector3 link_mr2dot_i = gazebo::math::Pose::Zero.pos;
	gazebo::math::Vector3 mr2dot_R = gazebo::math::Pose::Zero.pos;
	gazebo::math::Vector3 Hd_1_link = gazebo::math::Pose::Zero.pos;
	gazebo::math::Vector3 Hd_R1 = gazebo::math::Pose::Zero.pos;


	//This loop calculates the summation of r * (m*r_2dot) for robot (Hd_R1)
	for (unsigned int i = 0; i< links.size(); i++)
	{
		int j = 3 * i ;
		link_linear_acc.x = r_dd_gen(j);
		link_linear_acc.y = r_dd_gen(j+1);
		link_linear_acc.z = r_dd_gen(j+2);


		gazebo::physics::LinkPtr link = links[i];
		double mass_i = link->GetInertial()->GetMass();
		gazebo::math::Vector3 worldCoG_i = link->GetWorldCoGPose().pos;

		link_mr2dot_i = mass_i * link_linear_acc ;
		mr2dot_R += link_mr2dot_i ;

		Hd_1_link = worldCoG_i.Cross(link_mr2dot_i);

		Hd_R1 += Hd_1_link;

	}

	SGF::Vec w_d_gen ;
	w_d_gen.resize(dim);
	int ret_code1 ;

	//Second filter: estimate the angular acceleration by diffrentiation of I*W

	{
		clock_t t;
		t = clock();
		ret_code1 = filter_w.AddData(link_IW);
		ret_code1 = filter_w.GetOutput(1,w_d_gen);
		t = clock() - t;
		printf ("2nd filter took me %f seconds.\n",((float)t)/CLOCKS_PER_SEC);
	}

	gazebo::math::Vector3 link_Ang_acc = gazebo::math::Pose::Zero.pos;
	gazebo::math::Vector3 Hd_R2 = gazebo::math::Pose::Zero.pos;

	// This loop calculates the summation of angular acceleration for whole robot (Hd_R2)

	for (unsigned int i = 0; i< links.size(); i++)
	{
		int j = 3 * i ;
		link_Ang_acc.x = w_d_gen(j);
		link_Ang_acc.y = w_d_gen(j+1);
		link_Ang_acc.z = w_d_gen(j+2);

		Hd_R2 += link_Ang_acc;

	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////

	// Robot Linear momentum Rate
	gazebo::math::Vector3 P_d_R = gazebo::math::Pose::Zero.pos;
	P_d_R = mr2dot_R;

    // Robot angular momentum rate
	gazebo::math::Vector3 H_d_R = gazebo::math::Pose::Zero.pos;
	H_d_R = Hd_R1 +  Hd_R2  ;


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

	gazebo::math::Vector3 worldCoGModel = weighted_position_acc/mass_acc;
	gazebo::math::Vector3 worldZMPModel = gazebo::math::Pose::Zero.pos;

	double g = 9.8 ;
	worldZMPModel.x = (exttorque.y + moment_due_force.y + worldCoGModel.x * mass_acc * g - H_d_R.y)  /(mass_acc*g - extforce.z + P_d_R.z) ;
	worldZMPModel.y = ( - exttorque.x - moment_due_force.x  + worldCoGModel.y * mass_acc * g + H_d_R.x)/( mass_acc*g - extforce.z + P_d_R.z) ;


	/*double g = 9.8 ;
	gazebo::math::Vector3 worldCoGModel = weighted_position_acc/mass_acc;
	gazebo::math::Vector3 worldZMPModel = gazebo::math::Pose::Zero.pos;
	worldZMPModel.x = ( worldCoGModel.x * mass_acc * g - H_d_R.y) /(mass_acc*g + P_d_R.z) ;
	worldZMPModel.y = ( worldCoGModel.y * mass_acc * g + H_d_R.x)/( mass_acc*g  + P_d_R.z) ;
*/
	worldZMPModel.z = 0.0;

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


	/*t_overall = clock() - t_overall;
	printf ("Overall processing time %f seconds.\n",((float)t_overall)/CLOCKS_PER_SEC);*/

}


void ShowModelZMP::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	m_count = 1.0 ;

	// Check if yarp network is active;
	if (!yarp::os::Network::checkNetwork()) {
		yError("ERROR Yarp Network was not found active");
		return;
	}

	// Two ports to read the external wrench and its point of effect
	m_ExternalCmdInput.open("/ExternalCommandRec") ;
	m_LinkPosInput.open("/LinkPosInput");
	yarp.connect("/ExternalCommand","/ExternalCommandRec");
	yarp.connect("/TargetLinkPosition","/LinkPosInput") ;

	// creat a new port to plot the angular momentum
	//m_test_data.open("/Test_Data");
	std::string port_test_name =  "/Test_Data";
	m_test_data = new yarp::os::BufferedPort<yarp::os::Bottle>();
	if (!m_test_data || !m_test_data->open(port_test_name)) {
		yError("Could not open port %s", port_test_name.c_str());
		return;
	}


	// What is the parent name??
	this->m_modelScope = _model->GetScopedName();

	std::string port_name = "/" + this->m_modelScope + "/ZMPInWorld:o";
	m_zmpOutputPort = new yarp::os::BufferedPort<yarp::os::Bottle>();
	if (!m_zmpOutputPort || !m_zmpOutputPort->open(port_name)) {
		yError("Could not open port %s", port_name.c_str());
		return;
	}

	// Copy the pointer to the model to access later from UpdateChild
	this->m_myModel = _model;
	bool configuration_loaded = false;


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


