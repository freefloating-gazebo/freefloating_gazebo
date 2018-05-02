#!/usr/bin/env python

import xacro, sys
from subprocess import check_output
from roslaunch import substitution_args
import os
from lxml import etree
import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider,Button


# 2nd-order Butter
class Butter:
    def __init__(self, fc, dt):
        ita = 1./np.tan(np.pi*fc*dt)
        q = np.sqrt(2)
        self.b = [1./(1+q*ita+ita*ita)]
        self.b.append(2*self.b[0])
        self.b.append(self.b[0])
        self.a = [2*(ita*ita - 1)*self.b[0], -(1-q*ita+ita*ita)*self.b[0]]
        self.x = [0,0,0]
        self.y = [0,0]
        
    def butter(self, v):
        self.x[2] = self.x[1]
        self.x[1] = self.x[0]
        self.x[0] = v
        
        v = self.b[2]*self.x[2] + self.b[1]*self.x[1] + self.a[1]*self.y[1] + self.b[0]*self.x[0] + self.a[0]*self.y[0]

        self.y[1] = self.y[0];
        self.y[0] = v;  
        return v
    
# write default values
def write(d, key, val):
    print 'Dict: ', d
    print '   Called with', key, val
    if '/' in key:
        keys = key.split('/')
        print '   ', keys
        if keys[0] not in d:
            print '   Creating ' + keys[0]
            d[keys[0]] = {}
        write(d[keys[0]], '/'.join(keys[1:]), val)
    elif key not in d:
        d[key] = val

plt.close('all')
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)
dt = 0.01
t = np.arange(0,10,dt)

ax2 = ax.twinx()

axcolor = 'lightgoldenrodyellow'
axKp = plt.axes([0.25, 0.2, 0.65, 0.03])
axKi = plt.axes([0.25, 0.15, 0.65, 0.03])
axKd = plt.axes([0.25, 0.1, 0.65, 0.03])

sKp = Slider(axKp, 'Kp', 0, 100.0, valinit=5)
sKi = Slider(axKi, 'Ki', 0, 50.0, valinit=1)
sKd = Slider(axKd, 'Kd', 0, 2, valinit=0)

resetax = plt.axes([0.1, 0.15, 0.1, 0.04])
button = Button(resetax, 'OK', color=axcolor, hovercolor='0.975')


fig.tight_layout()


if __name__ == '__main__':
    
    gains = {'p': 2, 'i': 0, 'd': 0}
    
    if len(sys.argv) < 2:
        print 'Input a .xacro or .urdf file'
        print 'syntax : gen_pid.py <package> <urdf/xacro file>'
        sys.exit(0)
        
    robot_package = substitution_args.resolve_args('$(find %s)' % sys.argv[1])        
    robot_file = os.path.splitext(sys.argv[2])[0]
    
    # find file
    robot_abs_file = ''
    for urdf_dir in ['urdf', 'sdf']:
        for ext in ('urdf','xacro'):
            candidate = '{}/{}/{}.{}'.format(robot_package, urdf_dir, robot_file, ext)
            if os.path.lexists(candidate):
                robot_abs_file = candidate
    print('Using model ' + robot_abs_file)
            
    # create config directory
    if not os.path.lexists('%s/config' % robot_package):
        os.mkdir('%s/config' % robot_package)
    # look for existing config file
    config_file = ''
    for cf in os.listdir('{}/config'.format(robot_package)):
        if robot_file in cf:
            with open('{}/config/{}'.format(robot_package,cf)) as f:
                content = yaml.load(f)
            if 'controllers' in content:
                config_file = '{}/config/{}'.format(robot_package,cf)
    if config_file == '':       
        config_file = '{}/config/{}_pid.yaml'.format(robot_package, robot_file)
    print('Using config ' + config_file)
    # load description
    robot_description = etree.fromstring(check_output(['rosrun', 'xacro', 'xacro', '--inorder', robot_abs_file]))
    
    # init config dictionary
    if os.path.lexists(config_file):
        with open(config_file) as f:
            pid = yaml.load(f)['controllers']
    else:
        pid = {'config': {}}
    
    # parse joints
    joints = [joint for joint in robot_description.findall('joint') if joint.get('type') != "fixed"]
    if len(joints) != 0:
        write(pid, 'config/joints/state', 'joint_states')
        write(pid, 'config/joints/setpoint', 'joint_setpoint')
        write(pid, 'config/joints/command', 'joint_command')
        write(pid, 'config/joints/cascaded_position', False)
        write(pid, 'config/joints/dynamic_reconfigure', True)  
        for joint in joints:
            for mode in ('position','velocity'):
                for g in gains:
                    write(pid, '{}/{}/{}'.format(joint.get('name'), mode, g), gains[g])
                
    # parse thrusters
    thrusters = []
    base_link = 'base_link'
    damping = [100. for i in range(6)]
    m = 5
    
    for gazebo in robot_description.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            if plugin.get('name') == 'freefloating_gazebo_control':
                thrusters += plugin.findall('thruster')
                if plugin.find('link') != None:
                    base_link = plugin.find('link').text
            
    if len(thrusters) != 0:        
        write(pid, 'config/body/state', 'body_state')
        write(pid, 'config/body/setpoint', 'body_setpoint')
        write(pid, 'config/body/command', 'body_command')
        write(pid, 'config/body/cascaded_position', False)
        write(pid, 'config/body/dynamic_reconfigure', True)
        body_axis_candidates = {0: 'x', 1: 'y', 2: 'z', 3: 'roll', 4: 'pitch', 5: 'yaw'}
        body_axis = {}
        
        # get thruster characteristics
        T = []
        Umax = []
        for thruster in thrusters:
            thruster_map = [float(v) for v in thruster.find('map').text.split(' ')]
            T.append(thruster_map)
            Umax.append(float(thruster.find('effort').text))
            for x in body_axis_candidates:
                if thruster_map[x] != 0:
                    body_axis[x] = body_axis_candidates[x]
        T = np.matrix(T).transpose()
        n = len(Umax)
        Umax = np.matrix(Umax).reshape(n,1)  
        Fmax = np.abs(T)*Umax
        
        # get damping
        for link in robot_description.findall('link'):
            if link.get('name') == base_link:
                damp = link.find('buoyancy').find('damping')
                damping = [float(v) for v in damp.get('xyz').split(' ') + damp.get('rpy').split(' ')]        
                m = float(link.find('inertial').find('mass').get('value'))
                mass = [m,m,m]
                for key in 'ixx','iyy','izz':
                    mass.append(float(link.find('inertial').find('inertia').get('ixx')))
                
        # get mass / inertia
        
                
      
        
        # plot for x velocity
        vmax = np.sqrt(Fmax[0]/damping[0])
        vstar = vmax/2
        sd, = ax.plot([t[0],t[-1]], [vstar,vstar], lw=2, color='red')
        s, = ax.plot(t, 0*t, lw=2, color='blue')
        U, = ax2.plot(t, 0*t, lw=2, color='green')
        ax.axis((0,t[-1],0,2*vstar))
        ax2.axis((0,t[-1],0,1.05*Fmax[0]))
        
        butter = Butter(5, dt)

        def update(val):
            Kp = sKp.val
            Ki = sKi.val
            Kd = sKd.val 
            
            v = [0]
            u = [0]
            e_i = 0
            prev = 0
            
            for ti in t[1:]:
                cur = butter.butter(v[-1])
                e = vstar - cur
                if np.abs(u[-1]) < Fmax[0]:
                    e_i += e
                if prev:
                    cmd = Kp*(e + Ki*dt*e_i + Kd*(prev-cur)/dt)
                else:
                    cmd = Kp*(e + Ki*dt*e_i)                    
                    
                cmd = min(max(cmd,-Fmax[0]),Fmax[0])
                u.append(cmd)                                    
                v.append(v[-1] + dt*(cmd - damping[0]*cur*cur)/m)
                prev = cur
            s.set_data(t,v)
            U.set_data(t, u)
            
            fig.canvas.draw_idle()            

        sKp.on_changed(update)
        sKi.on_changed(update)
        sKd.on_changed(update)
        
        def save_config(val):
            for i,a in body_axis.iteritems():
                pid[a] = {}
                for controller in ['position', 'velocity']:
                    pid[a][controller] = {'p': p_default, 'i': i_default, 'd': d_default}
            # write config
            if len(joints) + len(thrusters) != 0:
                with open(config_file, 'w') as f:
                    yaml.dump({'controllers': pid}, f)
                print 'controller written in', config_file   
        button.on_clicked(save_config)
                
        plt.show()
        update(0)
                


        
