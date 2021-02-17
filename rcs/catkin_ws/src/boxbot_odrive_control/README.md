# Boxbot Odrive Pack


## Procedure of descending 

1. FSM calls the service /boxbot_descend after which the arm should start to lower its position slowly.
2. When the limit switch get triggered. The FSM should call the service again with cmd STOP. 
3. The STOP service will return the distance traveled during descend to FSM
4. FSM calls service /rcs_control with cmd==5 and goal==(x, y, prev_z + pos_diff, w)
