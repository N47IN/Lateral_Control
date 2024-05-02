import numpy as np
import os

################DO NOT MODIFY: FROM HERE#####################
#Measurement
OPEN = 1
SHUT = 0

#Controls
DO_NOTHING = 0
PUSH = 1

#these are the defacult values, but your solution will be evalutred by setting different values for these varibled, they are changed by the evaluation program
pinit = 0.5
p_open = 0
p_shut = 0
p_action = 0
################DO NOT MODIFY: TO HERE#####################

# YOU HAVE TO IMPLEMENT THIS FUCNTION WHERE
# bel_t_1 is the previos belief in form of a tuple [belief_open, belief_closed]
# control is either PUSH (1) or DO_NOTHING (0)
# observation is either OPEN(1) or SHUT(0)
# this function should return a tuple in form of new belief calcualted as [belief_open_new, belief_closed_new]
################DO NOT MODIFY: FROM HERE#####################
def bayes_filter(bel_t_1,control,observation): 
    bel_open = 0 # update this with calcualted belief for open
    bel_close = 0 # # update this with calcualted belief for clsoed/shut
  
    ################DO NOT MODIFY: TO HERE#####################
    if control ==1:
        closed_to_open = p_action
        closed_to_closed = 1 - p_action
        open_to_open = 1
        open_to_closed = 0
        bel_open = open_to_open*bel_t_1[0] + closed_to_open*bel_t_1[1]
        bel_closed = closed_to_closed*bel_t_1[1] + open_to_closed*bel_t_1[0]
    else :
        bel_open = bel_t_1[0]
        bel_closed = bel_t_1[1]
    
    new_bel = [bel_open,bel_closed]
    return new_bel
