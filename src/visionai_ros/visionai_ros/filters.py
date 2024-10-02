"""
Created on Thu Aug 24 14:11:02 2023

Filters to get the best Pick Instance from all detected ones

@author: st34616, rw39401
"""
import numpy as np
from analyze_tools import timeit_decorator

class Filters():
    def __init__(self) -> None:
        pass

    @timeit_decorator
    def filter_probability(ls_probs):
        '''
        Input :  List of probability

        Returns
        -------
        int : index of the best probabilty

        '''
        return np.array(ls_probs).argmax()

    @timeit_decorator
    def filter_probability_ns(ls_probs, n=5):
        '''
        Input :  List of probability

        Returns
        -------
        int : index of the best probabilty

        '''
        len_prob = len(ls_probs)
        if len_prob >= 2:
            if n < len_prob:
                n = len_prob
            max_values = [float('-inf')] * n
            max_indices = [-1] * n
            for index, value in enumerate(ls_probs):
                # Check if the current value is greater than any of the maximum values
                for i in range(n):
                    if value > max_values[i]:
                        # Update the maximum values and their indices
                        max_values.insert(i, value)
                        max_indices.insert(i, index)
                        # Remove the fourth maximum value and its index
                        max_values.pop()
                        max_indices.pop()
                        break
        return max_indices

    @timeit_decorator
    def filter_area(ls_area):
        '''
        Input :  List of area

        Returns
        -------
        int : index of the best probabilty
        '''
        return np.array(ls_area).argmax()