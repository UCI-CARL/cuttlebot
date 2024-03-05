#import needed libraries
import numpy as np
from random import random as rand

#Cognition class for defining cognitive abilities
class Cognition():
    #Cognition class constructor
    def __init__(self):
        #Intantiate the Q_table: { "Color" : {"RUN" : Q_Value, "Approach" : Q_value} }
        self.q_table = {
            "NONE" : {
                "UP" : 0,
                "DOWN" : 0,
                "LEFT" : 0,
                "RIGHT" : 0
            }
        }
        #define a learning rate
        self.learning_rate = 0.2
        #define a discount factor
        self.discount_factor = 0.9

    def print_q_table(self):
        print("=== CURRENT Q-TABLE ===")
        for state in self.q_table.keys():
            if(state == "NONE"):
                continue
            print(f"{state}:")
            for action in self.q_table[state].keys():
                print(f"\t{action}: {self.q_table[state][action]}")
        print("==========")

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
                if(min_value == None):
                   min_value = current_color_min_q_value 
                   most_punishing_color = color
                elif(current_color_min_q_value < min_value):
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
        print(f"probs: {run_approach_probabilities}")
        print(f"val: {random_value}")
        if(random_value < run_approach_probabilities[0]):
            return("RUN")
        else:
            return("APPROACH")
    
    def get_new_state(self, state, action):
        new_state = "NONE"
        return(new_state)
    
    def get_max_q_at_state(self, state):
        #get the q_table row of actions for a given state
        q_action_dict = self.q_table[state]
        #declare the max_q value
        max_q = None
        #loop through all actions
        for action in q_action_dict.keys():
            #make the first q value the max value
            if(max_q == None):
                max_q = q_action_dict[action]
            #reassign the value for larger q values found
            elif(max_q < q_action_dict[action]):
                max_q = q_action_dict[action]
        #return the max_q value found
        return(max_q)
    
    #update the q-table from the s, a, r, and s' values
    def update_q_table(self, state, action, reward, new_state):
        #q-learning update rule: use max possible q value of actions at the current state
        print(f"Updating Q-Table: {state}, {action}")
        self.q_table[state][action] += self.learning_rate * (reward + self.discount_factor * self.get_max_q_at_state(new_state) - self.q_table[state][action])
        print(f"\tChanged to {self.q_table[state][action]}")