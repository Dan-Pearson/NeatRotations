import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.grid(False)
ax.xaxis.pane.set_edgecolor('black')
ax.yaxis.pane.set_edgecolor('black')
ax.xaxis.pane.fill = False
ax.yaxis.pane.fill = False
ax.zaxis.pane.fill = False
ax.set_frame_on(False)
ax.set_axis_off()
phase = np.pi/8
# Output path for you animation images
out_path = './'
out_path = os.path.abspath(out_path)
fps = 40
prefix = 'myNewAni'
ext = '_.png'
N_steps = 2000
N_points = 1000
padding = len(str(N_steps))
t = np.linspace(0,12*np.pi,N_steps)
phi = np.linspace(0,2*np.pi,N_points)
c1 = np.zeros(shape=(3,N_points),dtype=np.float)
c1[1,:] = np.cos(phi)
c1[2,:] = np.sin(phi)
ax.plot(c1[0,:],c1[1,:],c1[2,:],'k')

b1 = np.zeros(shape=(3,),dtype=np.float)
#c_list = c1
for j in xrange(1,8):
    loop_theta = j*2*np.pi/16.
    loop_rot = np.array([[np.cos(loop_theta),-np.sin(loop_theta),0],[np.sin(loop_theta),np.cos(loop_theta),0],[0,0,1]],dtype=np.float)
    loop_c = np.matmul(loop_rot,c1)
    ax.plot(loop_c[0,:],loop_c[1,:],loop_c[2,:],'k')

q=1
for i in xrange(N_steps):
    loop_t = t[i]
    b1[0] = 0
    b1[1] = np.cos(loop_t)
    b1[2] = np.sin(loop_t)
    loopPs = np.ndarray(shape=(8,3),dtype=np.float)
    loopPs[0] = b1
    for k in xrange(1,8):
        loop_theta = k*2*np.pi/16.
        loop_rot = np.array([[np.cos(loop_theta),-np.sin(loop_theta),0],[np.sin(loop_theta),np.cos(loop_theta),0],[0,0,1]],dtype=np.float)
        shifted_b1 = np.zeros(shape=(3,),dtype=np.float)
        shifted_b1[1] = np.cos((k+1)**q*loop_t+k*phase)
        shifted_b1[2] = np.sin((k+1)**q*loop_t+k*phase)
        loopPs[k] = np.matmul(loop_rot,shifted_b1)

    zeros = '0'*(padding - len(str(i)))
    filename = os.path.join(out_path, '{}_{}{}{}'.format(prefix, zeros, i, ext))
    if i < N_steps/2.:
        angle = -90+360.*2*i/N_steps
    else:
        angle = 90#360.*i/N_steps-90
    ax.view_init(angle, 0)
    loopP = ax.scatter(loopPs[:,0],loopPs[:,1],loopPs[:,2],s=300,alpha=1,edgecolors='none')
    fig.tight_layout()
    myTrim=0.7
    fig.subplots_adjust(left=-myTrim,right=1+myTrim,top=1+myTrim,bottom=-myTrim)
    plt.savefig(filename)#,bbox_inches='tight')
    loopP.remove()


import subprocess
ffmpeg_fname = os.path.join(out_path, '{}_%0{}d{}'.format(prefix, padding, ext))
cmd = 'ffmpeg -f image2 -r {} -i {} -vcodec mpeg4 -y {}.mp4'.format(fps,ffmpeg_fname,prefix)

print cmd
subprocess.check_output(['bash','-c', cmd])
# Remove temp image files with extension
[os.remove(f) for f in os.listdir(out_path) if f.endswith(ext)]
