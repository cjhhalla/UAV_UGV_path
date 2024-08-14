# Caution
## launch file 
### 1. UGV namespace -> argment 
### 2. cmd_vel str -> jackal / rover
#### jackal : "/jackal_velocity_controller/cmd_vel"
#### rover : "/cmd_vel/managed"
### 3. path size, speed, laps customizing
### 4. cw_ccw -> true cw / false ccw
### 5. is_sim -> sim or not 
### 6. use_gps -> global_position/local or mavros/local_position/odom

# Failsafe node
## input :  fail safe topic
## output : stop signal(topic)
## regardless of robot type

# visualization 
## independent node for debugging.

# path planning and follow
## global way point 

# path following -> pure pursuit
