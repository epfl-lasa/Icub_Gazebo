# ZMP Calculation

This plugin has been written to calculate the position of "zero moment point"(ZMP).

To this end, three different terms should be considered:

1- projection of center of mass on the floor
2- effect of external forces and moments
3- Change in linear and angular momentum.

Here, a breif explanation is provided for each part:

1- Center of mass calculation is like something done in "showmodelcom" plugin. Center of gravity, mass and position of each link are taken from gazebo and COM is simply calculated as follow:

COM = Sigma (m_i * r_i) / m_total

2- Two ports are opened here to read the vector of external wrench and its point of effect. It's notable that these ports are created in another plugin named "apply external wrench". The vector of external force and moment can be read from the first port (m_ExternalCmdInput) and target position is read from the second one (m_LinkPosInput). Then, the moment due to force is also calculated. Here, making the ports connnected to each other will decrease greatly the real time factor of simulation. 

3- The main part of ZMP calculation is the effect of change in linear and angular momentums. Here, the approach is getting the link center of gravity position and its angular velocity from gazebo and then using the sg_differentiation library to compute the first and second derivatives. Pleaes note that computational time is too high for this part and it needs more improvement to be as fast as possible.

# Add Plugin

Pleaes follow the steps in the iCub/gazebo-yarp-plugin/plugins/README.md



