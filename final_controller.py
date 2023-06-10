from math import inf
import numpy as np
from numpy.lib.npyio import loads
from initialization import *
import enum
# from bug0 import *
# from bug1 import *
# from bug2 import *

if __name__ == "__main__":
    runBug = 1

    if runBug == 0:
        exec(open("./bug0.py").read())
    elif runBug == 1:
        exec(open("./bug1.py").read())
    elif runBug == 2:
        exec(open("./bug2.py").read())

        
