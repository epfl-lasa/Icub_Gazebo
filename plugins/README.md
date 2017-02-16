# Add a new plugin

In order to add a new plugin, in addition to write a spesific "CMakeLists.txt" related to the new plugin, the name of plugin sub-directory should be added to the "CMakeLists.txt" in the upper level (iCub/gazebo-yarp-plugin/plugins) as the following example:

add_subdirectory(showmodelzmp)


Also, following line should be included in icub.world (iCub/icub-gazebo/world/icub.world) 
```
<plugin name="........." filename="libgazebo_yarp_.........so"/> 
```
for example: 
```
<plugin name="ShowModelZMP" filename="libgazebo_yarp_showmodelzmp.so"/> 
```
Then:
```
-> cd iCub/gazebo-yarp-plugin/build
-> make
-> sudo make install
```





