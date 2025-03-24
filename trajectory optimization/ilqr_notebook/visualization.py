import numpy as np 
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Rectangle
from matplotlib import animation

def visualize_cartpole(q_trj, dt, L):
    fig, ax = plt.subplots()
    pole, = ax.plot([], [], color='royalblue')
    min_lim = -4
    max_lim =  4
    ax.axis('equal')
    ax.set_xlim([min_lim, max_lim])
    ax.set_ylim([min_lim, max_lim])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title('Cartpole Simulation:')

    x_trj = q_trj[:,0]
    theta_trj = q_trj[:,1]

    width = 1  # Width of Cart
    height = width/2  # Height of Cart
    xy_cart = (0., 0.)  # Bottom Left Corner of Cart
    cart = Rectangle(xy_cart, width, height, color='cornflowerblue')  # Rectangle Patch

    # Draw the Ground:
    ax.hlines(-height/2, min_lim, max_lim, colors='black')

    def init():
        ax.add_patch(cart)  
        return cart, pole,

    def animate(i):
        cart.set(xy=(x_trj[i]-width/2, -height/2))
        x_pole = x_trj[i] + L*np.sin(theta_trj[i])
        y_pole =          - L*np.cos(theta_trj[i])
        x_pole_arm = [x_trj[i], x_pole]
        y_pole_arm = [0.,       y_pole]
        pole.set_data(x_pole_arm, y_pole_arm)

        return cart, pole,
    
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=q_trj.shape[0], interval=50, blit=True)

    anim.save('cartpole.mp4', writer='ffmpeg', dpi=500)
