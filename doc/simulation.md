
roslaunch oscar_gazebo demo_pure_pursuit.launch path_filename:=/home/poine/work/two_d_guidance/paths/demo_z/circle_01.npz



# old
roslaunch oscar_gazebo empty_world.launch world_name:=`rospack find julie_gazebo`/worlds/enac_south_east.world

# new

roslaunch oscar_gazebo empty_world.launch world_name:=`rospack find common_simulations`/worlds/ethz_cam1.world
roslaunch oscar_gazebo demo_pure_pursuit.launch start_gazebo:=false path_filename:=`rospack find two_d_guidance`/paths/demo_z/track_ethz_cam1_new.npz vel_setpoint:=0.04

roslaunch oscar_gazebo empty_world.launch world_name:=`rospack find common_simulations`/worlds/oval_01.world
roslaunch oscar_gazebo demo_pure_pursuit.launch start_gazebo:=false path_filename:=`rospack find two_d_guidance`/paths/demo_z/oval_01.npz



#### Simulation with smocap

     roslaunch oscar_gazebo  empty_world.launch
     roslaunch smocap_gazebo  single_cam.launch pos_x:=0. pos_y:=0. pos_z:=3. rot_P:=1.5707963267948966
     rosrun smocap smocap_node.py _cameras:=camera_1 _detector_cfg_path:=`rospack find smocap`/params/gazebo_detector_cfg.yaml _run_mono_tracker:=true _trap_losses:=false 
