import numpy as np

def estimate_velocity(xaxis,yaxis):
    return np.polyfit(xaxis,yaxis,1)[0]