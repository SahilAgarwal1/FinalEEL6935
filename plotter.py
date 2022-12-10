import matplotlib.pyplot as plt
import numpy as np

# create ellipses



landmarks = [(18,7), (19,6), (19,6)]



wall_x = [0]
wall_y = [0,1,2,3,4,5,6,7,8,9,10,11]

x_y_uncertainty = [(2,1.5), (4, 3), (3,1), (2, 0.1), (1, 4)]

t = np.linspace(0, 2*np.pi, 100)



for landmark,uncertainty in zip(landmarks,x_y_uncertainty):
    u,v = landmark
    a,b = uncertainty
    plt.plot( u+a*np.cos(t) , v+b*np.sin(t) )
plt.grid(color='lightgray',linestyle='--')
plt.show()
