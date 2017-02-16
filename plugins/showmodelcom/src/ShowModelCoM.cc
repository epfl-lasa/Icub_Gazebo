/*
 * Copyright (C) 2013-2015 Istituto Italiano di Tecnologia RBCS & ADVR & iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ShowModelCoM.hh"

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>



namespace gazebo
{

    GZ_REGISTER_MODEL_PLUGIN (ShowModelCoM)

    ShowModelCoM::ShowModelCoM()
    : m_comOutputPort(0)
    {
        yarp::os::Network::init();

    }

    ShowModelCoM::~ShowModelCoM()
    {
        if (m_comOutputPort) {
            m_comOutputPort->interrupt();
            m_comOutputPort->close();
            delete m_comOutputPort;
            m_comOutputPort = 0;
        }

        if (m_comProjOutputPort) {
        	m_comProjOutputPort->interrupt();
        	m_comProjOutputPort->close();
                    delete m_comProjOutputPort;
                    m_comProjOutputPort = 0;
                }
        gazebo::event::Events::DisconnectWorldUpdateBegin(this->m_updateConnection);
        yarp::os::Network::fini();
    }

    void ShowModelCoM::UpdateChild()
    {

    	static double time_init = yarp::os::Time::now() ;
    	static int time_new = 0 ;

    	double time_current = yarp::os::Time::now();
    	int time_diff = time_current-time_init ;

    	std::ofstream link_name_icub ;
    		link_name_icub.open( "Link_Name_data.txt", std::ofstream::out | std::ofstream::app );


        gazebo::physics::Link_V links = m_myModel->GetLinks();

        double mass_acc = 0.0;
        gazebo::math::Vector3 weighted_position_acc = gazebo::math::Pose::Zero.pos;
        for(unsigned int i = 0; i < links.size(); ++i)
        {
        	gazebo::physics::LinkPtr link = links[i];
        	gazebo::math::Vector3 wordlCoG_i = link->GetWorldCoGPose().pos;
        	double mass_i = link->GetInertial()->GetMass();
       // 	gazebo::math::Matrix3 link_pos_i = link->GetWorldPose().rot.GetAsEuler() ;

        	weighted_position_acc += mass_i*wordlCoG_i;
        	mass_acc += mass_i;

        //	std::cout << links[0]->GetName() << "----------GET COG = " << links[0]->GetWorldCoGPose().pos << std::endl;
        //	link_name_icub <<  i <<"-    Link Name = " << links[i]->GetName() << " " <<    std::endl;

        }


        gazebo::math::Vector3 wordlCoGModel = weighted_position_acc/mass_acc;


        if (m_comOutputPort) {
            yarp::os::Bottle &bot = m_comOutputPort->prepare();
            bot.clear();
            bot.addDouble(wordlCoGModel.x);
            bot.addDouble(wordlCoGModel.y);
            bot.addDouble(wordlCoGModel.z);
            m_comOutputPort->write();
        }

        gazebo::math::Pose WorldCoGPose = gazebo::math::Pose::Zero;
        WorldCoGPose.pos = wordlCoGModel;

       // std::cout << wordlCoGModel << std::endl;

#if GAZEBO_MAJOR_VERSION >= 7
        msgs::Set(m_visualMsg.mutable_pose(), WorldCoGPose.Ign());
#else
        msgs::Set(m_visualMsg.mutable_pose(), WorldCoGPose);
#endif
        msgs::Set(m_visualMsg.mutable_material()->mutable_ambient(),common::Color(1,0,0,0.3));
        m_visualMsg.set_visible(1);
        m_visPub->Publish(m_visualMsg);


    /*    //// Added by Neda To show the projection of COM on the floor

              if (m_comProjOutputPort) {
                         yarp::os::Bottle &bot = m_comProjOutputPort->prepare();
                         bot.clear();
                         bot.addDouble(wordlCoGModel.x);
                         bot.addDouble(wordlCoGModel.y);
                         bot.addDouble(0.0);
                         m_comProjOutputPort->write();
                     }

              gazebo::math::Pose WorldCoGProjPose = gazebo::math::Pose::Zero;
              WorldCoGProjPose.pos.x = wordlCoGModel.x ;
              WorldCoGProjPose.pos.y = wordlCoGModel.y ;
              WorldCoGProjPose.pos.z = 0.0 ;


      #if GAZEBO_MAJOR_VERSION >= 7
              msgs::Set(m_visualMsg.mutable_pose(), WorldCoGProjPose.Ign());
      #else
              msgs::Set(m_newvisualMsg.mutable_pose(), WorldCoGProjPose);
      #endif
              msgs::Set(m_newvisualMsg.mutable_material()->mutable_ambient(),common::Color(1,0,0,0.3));
              m_newvisualMsg.set_visible(1);
              m_visnewPub->Publish(m_newvisualMsg);

      ////// End of Added by Neda*/
    }

    void ShowModelCoM::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Check if yarp network is active;
        if (!yarp::os::Network::checkNetwork()) {
            yError("ERROR Yarp Network was not found active");
            return;
        }

        // What is the parent name??
        this->m_modelScope = _model->GetScopedName();

        std::string port_name = "/" + this->m_modelScope + "/CoMInWorld:o";
        m_comOutputPort = new yarp::os::BufferedPort<yarp::os::Bottle>();
        if (!m_comOutputPort || !m_comOutputPort->open(port_name)) {
            yError("Could not open port %s", port_name.c_str());
            return;
        }

        std::string port_new_name = "/" + this->m_modelScope + "/CoMProjInWorld:o";
        m_comProjOutputPort = new yarp::os::BufferedPort<yarp::os::Bottle>();
                if (!m_comProjOutputPort || !m_comProjOutputPort->open(port_new_name)) {
                    yError("Could not open port %s", port_new_name.c_str());
                    return;
                }



    //	yarp::os::Bottle bot ;
       // m_readExtWr.open("/ReadExtWrench") ;
     //  yarp::os::Network::connect("/ExtWrNEDA:o","/ReadExtWrench");
      // m_readExtWr.read(bot);


        // Copy the pointer to the model to access later from UpdateChild
        this->m_myModel = _model;
        /// ############ TRYING TO MODIFY VISUALS #######################################################

        this->m_node = transport::NodePtr(new gazebo::transport::Node());

        this->m_node->Init ( _model->GetWorld()->GetName() );
        m_visPub = this->m_node->Advertise<msgs::Visual> ("~/visual", 10);

        // Set the visual's name. This should be unique.
        m_visualMsg.set_name ("__COM_VISUAL__");

        // Set the visual's parent. This visual will be attached to the parent
        m_visualMsg.set_parent_name ( _model->GetScopedName() );

        // Create a cylinder
        msgs::Geometry *geomMsg = m_visualMsg.mutable_geometry();
        geomMsg->set_type ( msgs::Geometry::SPHERE );
        geomMsg->mutable_sphere()->set_radius ( 0.02 );

        // Don't cast shadows
        m_visualMsg.set_cast_shadows ( false );

        /// ############ END: TRYING TO MODIFY VISUALS #####################################################

      /*  /// ############ TRYING TO MODIFY VISUALS (For Projection of COM) #######################################################

        this->m_new_node = transport::NodePtr(new gazebo::transport::Node());

        this->m_new_node->Init ( _model->GetWorld()->GetName() );
        m_visnewPub = this->m_new_node->Advertise<msgs::Visual> ("~/visual", 10);

        // Set the visual's name. This should be unique.
        m_newvisualMsg.set_name ("__COMProj_VISUAL__");

        // Set the visual's parent. This visual will be attached to the parent
        m_newvisualMsg.set_parent_name ( _model->GetScopedName() );

        // Create a cylinder
        msgs::Geometry *geomnewMsg = m_newvisualMsg.mutable_geometry();
        geomnewMsg->set_type ( msgs::Geometry::SPHERE );
        geomnewMsg->mutable_sphere()->set_radius ( 0.02 );

        // Don't cast shadows
        m_newvisualMsg.set_cast_shadows ( false );

        /// ############ END: TRYING TO MODIFY VISUALS #####################################################

*/
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ShowModelCoM::UpdateChild, this ) );
    }
}
