#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#


import constants

class Bug_State:

    #absolute angle that we have before starting return algorithm
    theta_start = 0	
    #number of control loop epochs that we have been exploring for
    exploration_cycle = 0
    #is the return algorithm activated
    algorithm_activated = False
    #is the point fro mwhich we need to head back towards (0,0) recorded
    algorithm_point = False

    #last closest recorded position on mline, closest in regards to distance to goals
    last_m_x = 0
    last_m_y = 0
    #have we reached the goal
    done = False
    
    #does the return algorithm have control
    in_control = True

    #two end points of the M-line
    m_line_start = [0,0] # 0,0 always, here as a variable for completeness
    m_line_end   = [0,0] 
