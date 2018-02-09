import matplotlib
matplotlib.use('Agg')

import cvxpy
import cvxpy as cvx, numpy as np, matplotlib.pyplot as plt
from qcqp import *
import cvxopt
import mosek
import math
import time
import os, datetime

import matplotlib.pyplot as plt
import matplotlib.patches as patches
# %matplotlib inline

results_dir = os.path.join(os.getcwd(), 'Results/Jan22/')
sample_file_name = datetime.datetime.now().strftime('%m-%d_%H-%M') + "_plotTest.jpeg"

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)


X = np.linspace(0, 10,10)
# Y = np.zeros((1,len(X)),dtype=float)
Y = []

for i in range (len(X)):
	Y.append(1)


print "X = ", X, " , Y = ", Y
plt.plot(X,Y,"-k")

plt.autoscale(enable=True, axis=u'both', tight=False)
plt.show()
plt.savefig(results_dir + sample_file_name,dpi = 600)
	