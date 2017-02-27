/*
 * Copyright (C) 2013-2015 Istituto Italiano di Tecnologia RBCS & ADVR & iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */
#ifndef SHOWMODELZMP_HH
#define SHOWMODELZMP_HH

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <yarp/math/Math.h>
#include <yarp/os/Log.h>
#include <yarp/math/SVD.h>
#include <gazebo/math/Angle.hh>
#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>


#include <gsl/gsl_math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_version.h>
#include <gsl/gsl_eigen.h>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

#include <thread>

#include "sg_filter.h"

	int order = 2;
	int winlen = 11	;
	int dim = 3 * 82 ;
	SGF::real sample_time = 1e-3; // this is simply a float or double, you can change it in the header sg_filter.h if yo u want
	SGF::SavitzkyGolayFilter filter(dim,order, winlen, sample_time);
	SGF::SavitzkyGolayFilter filter_w(dim,order, winlen, sample_time);


using namespace std;


namespace gazebo
{
class ShowModelZMP;

namespace transport {
class Publisher;
typedef boost::shared_ptr<Publisher> PublisherPtr;
class Node;
typedef boost::shared_ptr<Node> NodePtr;
}

namespace Physics {
class Model;
typedef boost::shared_ptr<Model> ModelPtr;
}
namespace event {
class Connection;
typedef boost::shared_ptr<Connection> ConnectionPtr;
}

namespace msgs {
class Visual;
}
}

namespace yarp {
namespace os {
class Bottle;

template <class T>
class BufferedPort;
}
}


class gazebo::ShowModelZMP : public gazebo::ModelPlugin
{
public:
	ShowModelZMP();


	yarp::os::Network					yarp;


	virtual ~ShowModelZMP();

	std::string retrieveSubscope(gazebo::physics::Link_V& v, std::string  scope);

	std::string          robotName;
protected:
	// Inherited
	void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
	// Inherited
	virtual void UpdateChild();

private:
	std::string             m_subscope;
	yarp::os::Property      m_iniParams;
	gazebo::transport::NodePtr m_node;

	void 		Thread_example();

	gazebo::physics::ModelPtr       m_myModel;

	std::string             m_modelScope;
	gazebo::event::ConnectionPtr    m_updateConnection;

	gazebo::transport::PublisherPtr m_visPub;
	gazebo::msgs::Visual            m_visualMsg;

	yarp::os::BufferedPort<yarp::os::Bottle> *m_zmpOutputPort;
	yarp::os::BufferedPort<yarp::os::Bottle>   m_ExternalCmdInput;
	yarp::os::BufferedPort<yarp::os::Bottle>   m_LinkPosInput;
//	yarp::os::Port 			   m_test_data;

	yarp::os::BufferedPort<yarp::os::Bottle> *m_test_data;


	//iCub::iDynTree::DynTree		   ICUB_ROBOT_TREE;
	//KDL::CoDyCo::UndirectedTree    iCubTree;
	//iDynTree::Model				m_ICUB_NEW_MODEL ;
	//KDL::CoDyCo::UndirectedTree m_robot_model;
	//iDynTree::HighLevel::DynamicsComputations dynComp ;


	bool                    m_newCommand;

	boost::mutex            m_lock;

	yarp::os::Bottle 		bot ;
	yarp::os::Bottle	 bot2;
	bool open_file = true ;
	bool check = true ;
	double m_count;

		std::thread* the_thread ;

		gazebo::math::Vector3  link_m_a ;

		yarp::os::Bottle	 bot_test;



};



#endif
