roslaunch oscar_navigation enac_z_demo_pure_pursuit.launch path_filename:=/home/poine/work/two_d_guidance/paths/demo_z/oval_01.npz

rosrun oscar_control record_debug_io.py
rosrun homere_control io_dataset.py /tmp/oscar_io.npz
rosrun homere_control fit_odometry.py /tmp/oscar_io.npz

