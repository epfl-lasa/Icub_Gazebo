# Plugin Description

This plugin provides a yarp rpc port for the user to send external commands to te robot. Also, the plugin is able to visualize the vecotor of external wrench in the simulation environment.

Here, we have created two new yarp ports and then we write the information which is received from the rpc port on these ports. The information includes the vector of external forces and moments, contact position and contact duration. These data will be used in another plugin to calculate the effect of external wrench on ZMP position.

# Implementation

In order to apply external wrench to each link, following rpc port should be implemented:

/iCub/applyExternalWrench/rpc:i

Then, open a new terminal and use this command:

    --yarp rpc /iCub/applyExternalWrench/rpc:i 
    target_link_name Fx Fy Fz Mx My Mz t

example:

    chest 10 0 0 0 0 0 1.5





