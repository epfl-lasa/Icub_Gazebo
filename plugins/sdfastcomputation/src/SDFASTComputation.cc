/*
 * SDFASTComputation plugin has been written in order to compute required data for input description file needed by SDFAST library.
 *
 */

#include "SDFASTComputation.hh"

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>


#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>



namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN (SDFASTComputation);

SDFASTComputation::SDFASTComputation()
{
	yarp::os::Network::init();

}
SDFASTComputation::~SDFASTComputation()
{
	gazebo::event::Events::DisconnectWorldUpdateBegin(this->m_updateConnection);
	yarp::os::Network::fini();
}

void SDFASTComputation::UpdateChild()
{

	std::ofstream link_data_icub ;
	std::ofstream joint_data_icub ;
	std::ofstream l_shoulder_data ;
	std::ofstream l_elbow_data ;

	link_data_icub.open( "Link_data_iCub.txt", std::ofstream::out | std::ofstream::app );
	joint_data_icub.open( "Joint_data_iCub.txt", std::ofstream::out | std::ofstream::app );
	l_shoulder_data.open( "l_shoulder_data.txt", std::ofstream::out | std::ofstream::app );
	l_elbow_data.open( "l_elbow_data.txt", std::ofstream::out | std::ofstream::app );

	static bool check_l =true ;
	static bool check_j =true ;

	gazebo::physics::Link_V links = m_myModel-> GetLinks();
	gazebo::physics::Joint_V joints = m_myModel-> GetJoints();

	for(unsigned int i = 0; i < joints.size(); ++i)
	{
		gazebo::physics::JointPtr joint = joints[i];

		// Get all joints' position in world reference frame.

		gazebo::math::Vector3 jointCoG_i = joint -> GetWorldPose().pos;
		gazebo::math::Angle q = joint -> GetAngle(0);

		if (check_j == true)
		{
			joint_data_icub<< "   " << i << "- Joint Name = " << joint->GetName() << "--------Parent Name = "<<joint->GetParent()->GetName()  << "------------Joint Pos = "	<< jointCoG_i << "-------------Joint angle = " << q << std::endl;

		}

	}


	check_j = false ;

	gazebo::math::Matrix3 rot_mat;
	gazebo::math::Matrix3 w_2link_rot_i;

	for(unsigned int i = 0; i < links.size(); ++i)
	{
		gazebo::physics::LinkPtr link = links[i];

		// Get COG position of each link in world reference frame.

		gazebo::math::Vector3 wordlCoG_i = link->GetWorldCoGPose().pos;
		double mass_i = link->GetInertial()->GetMass();

		// Get the euler angles for each link

		gazebo::math::Vector3 link_euler_i = link->GetWorldCoGPose().rot.GetAsEuler() ;

		// Rotation matrix from link to world reference frame

		rot_mat[0][0] = cos(link_euler_i[1])*cos(link_euler_i[2]);
		rot_mat[1][0] = cos(link_euler_i[1])*sin(link_euler_i[2]);
		rot_mat[2][0] = -sin(link_euler_i[1]);

		rot_mat[0][1] = sin(link_euler_i[0])*sin(link_euler_i[1])*cos(link_euler_i[2])-cos(link_euler_i[0])*sin(link_euler_i[2]);
		rot_mat[1][1] = sin(link_euler_i[0])*sin(link_euler_i[1])*cos(link_euler_i[2])+cos(link_euler_i[0])*cos(link_euler_i[2]);
		rot_mat[2][1] = sin(link_euler_i[0])*cos(link_euler_i[1]);


		rot_mat[0][2] = cos(link_euler_i[0])*sin(link_euler_i[1])*cos(link_euler_i[2])+sin(link_euler_i[0])*sin(link_euler_i[2]);
		rot_mat[1][2] = cos(link_euler_i[0])*sin(link_euler_i[1])*sin(link_euler_i[2])-sin(link_euler_i[0])*cos(link_euler_i[2]);
		rot_mat[2][2] = cos(link_euler_i[0])*cos(link_euler_i[1]);

		// Rotation matrix from world to link reference frame

		w_2link_rot_i[0][0] = rot_mat[0][0] ;
		w_2link_rot_i[0][1] = rot_mat[1][0] ;
		w_2link_rot_i[0][2] = rot_mat[2][0] ;
		w_2link_rot_i[1][0] = rot_mat[0][1] ;
		w_2link_rot_i[1][1] = rot_mat[1][1] ;
		w_2link_rot_i[1][2] = rot_mat[2][1] ;
		w_2link_rot_i[2][0] = rot_mat[0][2] ;
		w_2link_rot_i[2][1] = rot_mat[1][2] ;
		w_2link_rot_i[2][2] = rot_mat[2][2] ;

		gazebo::math::Vector3 link_COG = w_2link_rot_i * wordlCoG_i ;


		if (check_l == true ){

			link_data_icub << "   " << i  <<"-    Link Name = " << link->GetName() << " " << ": "<< " Mass = " << mass_i << "    Euler Angles: "<< link_euler_i << "   World COG =  "<< wordlCoG_i << std::endl;

			//link_data_icub <<  " link_2_w = "<< rot_mat <<  std::endl;
		//	link_data_icub <<  i  << "-    Link Name = " << link->GetName() << std::endl;
			link_data_icub << "W_2_link = "<< w_2link_rot_i << std::endl;
			//std::cout <<  "Euler = "<< link_euler_i<< "          " <<  sin(link_euler_i[2]) << std::endl;
		}


	}

	check_l = false ;

	l_shoulder_data << "   " << 20  <<"-    Link Name = " << links[20]->GetName() << " " << ": "<< "   World COG =  "<< links[20]->GetWorldCoGPose().pos << std::endl;
	l_elbow_data << "   " << 24  <<"-    Link Name = " << links[24]->GetName() << " " << ": "<< "   World COG =  "<< links[24]->GetWorldCoGPose().pos << std::endl;


}

void SDFASTComputation::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	// Check if yarp network is active;
	if (!yarp::os::Network::checkNetwork()) {
		yError("ERROR Yarp Network was not found active");
		return;
	}


	// Copy the pointer to the model to access later from UpdateChild
	this->m_myModel = _model;

	// Listen to the update event. This event is broadcast every simulation iteration.

	this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &SDFASTComputation::UpdateChild, this ) );
}
}
