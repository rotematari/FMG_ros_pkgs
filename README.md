# FMG_ros_pkgs

for the real time pkg 

TODO: add move joint srvice witch need to be ready to publish to the /jointState once every reading 
needs to have 2 input options 
1. model output 
2. natnet output 



options - 
checks if the new locatin it logical (not to far from the corrent location)


## in franka pkg 

the panda.urdf.xacro file change here 
  <!-- safety_distance: Minimum safety distance in [m] by which the collision volumes are expanded and which is enforced during robot motions -->
  <!-- arm_id: Namespace of the panda arm. Serves to differentiate between arms in case of multiple instances. -->
  <xacro:macro name="panda_arm" params="arm_id:='panda' description_pkg:='franka_description' connected_to:='world' xyz:='0 -0.3 1.05' rpy:='0 0 1.5708' safety_distance:=0">
    <xacro:unless value="${not connected_to}">
      <joint name="${arm_id}_joint_${connected_to}" type="fixed">
        <parent link="${connected_to}"/>
        <child link="${arm_id}_link0"/>
        <origin rpy="${rpy}" xyz="${xyz}"/>
      </joint>


in bringup change all nodes to name space franka include griper