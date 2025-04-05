'''
Auralius Manurung
Universitas Telkom
2025
'''

from IPython.display import HTML

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from matplotlib import rc
rc('animation', html='jshtml')

import base64


plt.rcParams.update({'font.size': 10})

'''
This function plot and animate the root locus.
The animation shows the movements of the closed-loop poles as the gain chnages.
'''
def animate(gains, cl_poles, ol_poles, ol_zeros, gif_fn="./gif.gif", fps=30):
    fig = plt.figure();
    ax = fig.add_subplot();

    ax.axhline(y=0, color='k');
    ax.axvline(x=0, color='k');

    for i in range(cl_poles.shape[1]):
        ax.plot(np.real(cl_poles[:,i]), np.imag(cl_poles[:,i]), linewidth=5, linestyle='-', color='b');
    ax.plot(np.real(ol_poles), np.imag(ol_poles), linestyle='none', markersize=10, marker='x', color='k');
    ax.plot(np.real(ol_zeros), np.imag(ol_zeros), linestyle='none', markersize=10, marker='o', color='k');

    if len(ol_poles) != len(ol_zeros):
        ax.set_xlim([np.min(np.real(cl_poles)), np.max(np.real(cl_poles))]);
    else:
        ax.axis("equal"); # equal axis only if there are no infinite poles / infinite zeros!

    ax.set_ylim([np.min(np.imag(cl_poles)) - 0.1, np.max(np.imag(cl_poles)) + 0.1]);

    
    H = []
    for i in range(cl_poles.shape[1]):
        h, = ax.plot(np.real(cl_poles[0,i]), np.imag(cl_poles[0,i]), markersize=10, marker='x', color='r');
        H.append(h);
    
    c = np.array([(np.min(np.real(cl_poles)) + np.max(np.real(cl_poles)))/2, 
                  np.max(np.imag(cl_poles)) + 0.11]);
    h = ax.text(c[0], c[1], "");
    H.append(h);
      

    def update(k):
        for i in range(len(H)-1):
            H[i].set_data([np.real(cl_poles[k,i]), np.real(cl_poles[k,i])], [np.imag(cl_poles[k,i]), np.imag(cl_poles[k,i])]);
        H[-1].set_text(f"K = {gains[k]:.1f}")
        return H;

    ani = FuncAnimation(fig, update, len(gains), interval=1, blit=True);    

    ani.save(gif_fn,  writer='imagemagick', fps=fps);
    
    return ani;


def display_gif(fn):
    b64 = base64.b64encode(open(fn,'rb').read()).decode('ascii');
    display(HTML(f'<img src="data:image/gif;base64,{b64}" />'));
