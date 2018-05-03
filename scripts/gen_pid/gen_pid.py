#!/usr/bin/env python

import yaml
import pids
import parsers
from PyQt5 import QtWidgets
import tunepid
import sys

try:
    parsers = reload(parsers)
    pids = reload(pids)
    tunepid = reload(tunepid)
except:
    pass

# load config
if len(sys.argv) < 2:
    print 'Input a .xacro or .urdf file'
    print 'syntax : gen_pid.py <package> <urdf/xacro file>'
    sys.exit(0)        
pid, Umax, Fmax, mass, damping, config_file = parsers.parse(sys.argv[1], sys.argv[2])

axes = ('x','y','z','roll','pitch','yaw')
max_gains = {'p': 150., 'i': 50., 'd': 10.}


class TunePID(QtWidgets.QMainWindow):

    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.ui = tunepid.Ui_TunePID()
        self.ui.setupUi(self)
        
        self.psim = pids.Sim('p')
        self.vsim = pids.Sim('v')
        self.ui.p_sim.addWidget(self.psim.canvas)
        self.ui.v_sim.addWidget(self.vsim.canvas)
        
        # build axes menu        
        for i,ax in enumerate(axes):
            if Fmax[i]:
                self.ui.axes_list.addItem(ax)
        self.ui.axes_list.currentTextChanged.connect(self.axis_change)
        
        self.ui.time_slider.valueChanged.connect(self.sims)
        
        # connect gains / setpoint sliders
        for mode,fct in (('position', self.position_sim), ('velocity', self.velocity_sim)):
            for g in ('p', 'i', 'd'):            
                getattr(self.ui, mode[0]+'_K{}_slider'.format(g)).valueChanged.connect(fct)
                getattr(self.ui, mode[0]+'_setpoint').valueChanged.connect(fct)
            
        # connect save gains button
        getattr(self.ui, 'p_save').clicked.connect(lambda: self.save_gains('position'))
        getattr(self.ui, 'v_save').clicked.connect(lambda: self.save_gains('velocity'))
        # connect save to file
        getattr(self.ui, 'p_tofile').clicked.connect(lambda: self.write('position'))
        getattr(self.ui, 'v_tofile').clicked.connect(lambda: self.write('velocity'))

        # global save
        self.ui.tofile.clicked.connect(self.write_all)
        
        self.axis_change()
                
    def gains(self, mode):      
        gains = {}
        for g in ('p','i','d'):
            label = mode + '_K' + g
            gains[g] = getattr(self.ui, label+'_slider').value()*max_gains[g]/1000
            getattr(self.ui, label).setText(str(gains[g]))
        return gains
       
    def axis_change(self):
        ax = str(self.ui.axes_list.currentText())
        # load gains
        for mode in ('position','velocity'):
            for g in ('p','i','d'):
                label = mode[0] + '_K' + g + '_slider'
                getattr(self.ui, label).setValue(pid[ax][mode][g]*1000/max_gains[g])
                
    def sims(self):
        self.position_sim()
        self.velocity_sim()
                    
    def position_sim(self):
        
        # get simulated axis
        ax = str(self.ui.axes_list.currentText())
        # cascaded or not
        
        # setpoint
        i = axes.index(ax)
        sp = self.ui.p_setpoint.value()*10./1000
        self.ui.p_sp.setText(str(sp))
        
        # final time
        tmax = self.ui.time_slider.value()*20./1000
        
        self.psim.run(self.gains('p'), sp, mass[i], damping[i], Fmax[i], tmax)
        
        
    def velocity_sim(self):
        # get simulated axis
        ax = str(self.ui.axes_list.currentText())
        
        # get max setpoint
        i = axes.index(ax)
        vmax = Fmax[i]/damping[i]
        sp = self.ui.v_setpoint.value()*1.5*vmax/1000
        self.ui.v_sp.setText(str(sp))
        
        tmax = self.ui.time_slider.value()*20./1000
        
        self.vsim.run(self.gains('v'), sp, mass[i], damping[i], Fmax[i], tmax)
        
    def save_gains(self, mode, ax = None):
        global pid
        if ax == None:
            ax = str(self.ui.axes_list.currentText())
        pid[ax][mode] = self.gains(mode[0]) 
        print 'Writing gains for {}/{}'.format(ax, mode)
                
    def write(self, mode, ax = None):
        
        self.save_gains(mode, ax)
            
        # only save current axis
        if ax == None:
            ax = str(self.ui.axes_list.currentText())
        
        # reload current gains from config file
        with open(config_file) as f:
            pid_base = yaml.load(f)['controllers']
        # replace with current
        pid_base[ax][mode] = pid[ax][mode]
        
        # and save
        with open(config_file, 'w') as f:
            yaml.dump({'controllers': pid_base}, f, default_flow_style=False)
            
            
    def write_all(self):        
        # reload current gains from config file
        with open(config_file) as f:
            pid_base = yaml.load(f)['controllers']
        
        for ax in [self.ui.axes_list.itemText(i) for i in range(self.ui.axes_list.count())]:
            for mode in ('position','velocity'):
                self.save_gains(mode, ax)
                # replace with current
                pid_base[ax][mode] = pid[ax][mode]

        # and save
        with open(config_file, 'w') as f:
            yaml.dump({'controllers': pid_base}, f, default_flow_style=False)
                

if __name__ == '__main__':
    
    pids.pl.close('all')    
    app = QtWidgets.QApplication(sys.argv)
    window = TunePID()
    window.show()
    sys.exit(app.exec_())
    
