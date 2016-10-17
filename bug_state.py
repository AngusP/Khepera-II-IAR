import constants

class Bug_State:

    theta_start = 0	
    exploration_cycle = 0
    algorithm_activated = False
    algorithm_point = False

    last_m_x = 0
    last_m_y = 0

    done = False
    

    in_control = True

    m_line_start = [0,0] # 0,0 always, here for completeness
    m_line_end   = [0,0] 
