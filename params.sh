export WORLD="world20" ##options: world20 world21 world0. ##Note that world23proj1 will not create a gazebo, cus theres no sim world. Note that world0 is for tuning
export TASK="mode1" ##options: mode1 mode2 mode3


#The following describes the possible combinations of TASK and WORLD params.
# ___________________________________________________________________________________________________________________________________________________
#    TASK     |             WORLD              |                                          TASK DESCRIPTION                                          |  
#             |                                |                                                                                                    |
#   "mode1"   |      "world20", "world21"      |       "mode1"  will run exclusively the turtlebot in simulation                      | 
#             |                                |                                                                                                    |            
#             |                                |                                                                                                    |
#   "mode3"   |      "world20", "world21"      |       "mode3" will run both the turtebot and the hector in simulation in world20 or world21        |  
#             |                                |                                                                                                    |
#   "mode2"   |           "world0"             |       "mode2" will spawn only the hector in world0, which is a training world that can             |
#             |                                |         be used for testing only the hector. If you use this option, you must ensure that waypoints| 
#             |                                |        are set in the goal file and set co_op param to false in drone_commander.yaml.              |
#             |                                |        See README for more information                                                             |
# ____________|________________________________|____________________________________________________________________________________________________|
