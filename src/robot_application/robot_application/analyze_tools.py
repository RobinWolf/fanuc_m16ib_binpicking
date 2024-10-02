'''
    tools and functions to analyze the performance of the pipeline algorithms
'''

import time
import params

def timeit_decorator(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        elapsed_time = end_time - start_time
        message = f"{func.__name__} took {elapsed_time} seconds to execute."
        print(message)

        # write to logs file
        with open(params._logfile, 'a') as file:
            file.write(message + "\n")
        return result
    return wrapper


def timeit_function(name,start_time):
    end_time = time.time()
    elapsed_time = end_time - start_time
    message = f"{name} took {elapsed_time} seconds to execute."
    print(message)

    # write to logs file
    with open(params._logfile, 'a') as file:
        file.write(message + "\n")