#import needed libraries
import numpy as np
import random.random as rand

#Cognition class for defining cognitive abilities
class Cognition():
    #Cognition class constructor
    def __init__(self):
        #Intantiate the Q_table: { "Color" : {"RUN" : Q_Value, "Approach" : Q_value} }
        self.q_table = dict()

    def _softmax(self, np_value_list):
        #referencing alvas & Trevor Merrifield on stack overflow: https://stackoverflow.com/questions/34968722/how-to-implement-the-softmax-function-in-python
        #solution used for numerical stability: e^x / sum(e^x) = e^(x-x_max) / sum(e^(x-x_max))
        exponential_vector = np.exp(np_value_list - np.max(np_value_list))
        return(exponential_vector/exponential_vector.sum())        

    def get_state_with_largest_punishment(self, colors_in_view):
        #error check for passing an empty list
        if(len(colors_in_view) == 0):
            return(None)
        
        #Initiallize the most_punishing_color and min_value to None
        most_punishing_color = None
        min_value = None
        #loop through each color
        for color in colors_in_view:
            #try to access the color from the q_table
            try:
                #get the min value from approach and run
                current_color_min_q_value = min(self.q_table[color]["RUN"], self.q_table[color]["APPROACH"])
                #update the most punishing color if the possible punishment is lower than before
                if(current_color_min_q_value < min_value):
                    most_punishing_color = color
                    min_value = current_color_min_q_value
            #if color does not exist in q_table, then add it in
            except:
                self.q_table[color] = {
                    "RUN" : 0, 
                    "APPROACH" : 0
                }
                #update the most_punishing color if it is currently positive
                if((most_punishing_color == None) or min_value > 0):
                    most_punishing_color = color
                    min_value = 0
        #return the most_punishing color
        return(most_punishing_color)


    def get_action(self, state):
        try:
            run_approach_q_value_list = np.array([self.q_table[state]["RUN"], self.q_table[state]["APPROACH"]])
        #if state does not exist in q_table, then add it in
        except:
            self.q_table[state] = {
                "RUN" : 0, 
                "APPROACH" : 0
            }
            run_approach_q_value_list = np.array([self.q_table[state]["RUN"], self.q_table[state]["APPROACH"]])
        
        run_approach_probabilities = self._softmax(run_approach_q_value_list)
        random_value = rand()
        if(random_value < run_approach_probabilities[0]):
            return("RUN")
        else:
            return("APPROACH")
        
    
    def update_q_table(self, state, action, reward):
        pass