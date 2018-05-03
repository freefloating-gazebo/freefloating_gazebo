#!/usr/bin/env python

import pylab as pl
from matplotlib import pyplot as pp
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class PID:
    def __init__(self, dt):
        self.dt = dt
        self.reset()
        
    def set_gains(self, Kp, Ki, Kd, umax):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.umax = umax        
        
    def reset(self):
        self.u = 0
        self.x = 0
        self.i = 0
        
    def command(self, x, xd):
        e = xd - x
        if abs(self.u) < self.umax:
            self.i += e
        if self.x:
            cmd = self.Kp*(e + self.Ki*self.dt*self.i + self.Kd*(self.x - x)/self.dt)
        else:
            cmd = self.Kp*(e + self.Ki*self.dt*self.i)  
        self.x = x
        return min(max(cmd,-self.umax),self.umax)
    

class Sim:
    def __init__(self, mode, dt = 0.01):
        
        self.fig = pp.figure()
        self.mode = mode
        
        self.dt = dt
        self.t = pl.arange(0, 15, self.dt)
        
        # setpoint vs output
        self.ax = self.fig.add_subplot(111)        
        self.lxd, = self.ax.plot(self.t, 0*self.t, 'b--', lw=2, label='setpoint')
        self.lx, = self.ax.plot(self.t, 0*self.t, lw=2, color='red', label='output')        
        self.ax.set_xlim(self.t[0], self.t[-1])
        if mode == 'p':
            self.ax.set_ylabel('Setpoint / output [m - rad]')
        else:
            self.ax.set_ylabel('Setpoint / output [m/s - rad/s]')
        
        # command
        self.ax2 = self.ax.twinx()        
        self.lu, = self.ax2.plot(self.t, 0*self.t, 'g', lw=2, label='command')
        self.ax2.plot(self.t, 0*self.t, 'k--')
        self.ax2.set_ylabel('Command (N)')
        self.fig.tight_layout()
                        
        
        
        self.canvas = FigureCanvas(self.fig)      
        
    def run(self, gains, xd, m, k, fmax):
        
        if xd == 0:
            xd = 0.1
        
        u = [0]
        x = [0]
        v = 0
        self.lxd.set_data([self.t[0],self.t[-1]], [xd, xd])
        
        pid = PID(self.dt)
        pid.set_gains(gains['p'], gains['i'], gains['d'], fmax)
        
        for ti in self.t[1:]:
            # control
            u.append(pid.command(x[-1], xd))
            
            # model
            v += self.dt*(u[-1] - k*v)/m
            # velocity PID
            x.append(v)
            
        # rescale time if needed
        last = self.t.shape[0]-1
        while abs(x[last]-xd)/xd < 0.01:
            last -= 1
                
        self.lx.set_data(self.t[:last], x[:last])
        self.lu.set_data(self.t[:last], u[:last])
               
        self.ax.set_xlim(0, self.t[last])        
        self.ax.set_ylim(0, 1.2*xd)
        self.ax2.set_xlim(0, self.t[last])
        self.ax2.set_ylim(-1.3*fmax, 1.3*fmax)
                
        self.canvas.draw()            
