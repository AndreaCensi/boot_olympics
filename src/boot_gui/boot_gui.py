from . import logger
from .boot_gui_generated import MyFrame
from .boot_gui_run import create_vehicles_launch
from bootstrapping_olympics.loading.load_all import (Configuration as BOConf,
    load_configuration as BO_load_all)
from vehicles.configuration.load_all import (Configuration as VConf,
    load_configuration as V_load_all)
import os
import subprocess
import wx



class BootGUIFrame(MyFrame):
    
    def h_vehicle_selected(self, event): # wxGlade: MyFrame.<event_handler>
        id_vehicle = self.bg_vehicle_list.GetValue()
        desc = VConf.vehicles[id_vehicle]['desc']
        self.bg_vehicle_desc.SetLabel(desc)

    def h_world_selected(self, event): # wxGlade: MyFrame.<event_handler>
        id_world = self.bg_world_list.GetValue()
        desc = VConf.worlds[id_world]['desc']
        self.bg_world_desc.SetLabel(desc)

    def h_agent_selected(self, event): # wxGlade: MyFrame.<event_handler>
        id_agent = self.bg_agent_list.GetValue()
        desc = BOConf.agents[id_agent]['desc']
        self.bg_agent_desc.SetLabel(desc)
        
    def h_task_selected(self, event): # wxGlade: MyFrame.<event_handler>
        #print "hello: %s" % event        
        pass
        
    def fill_choices(self):
        try:
            cmds = ['rospack', 'find', 'boot_olympics_launch']
            output_dir = subprocess.check_output(cmds).strip() #@UndefinedVariable
        except:
            logger.warning('Could not find package using rospack.')
            output_dir = os.curdir
                
        self.bg_output_dir.SetValue(output_dir)
        
        vehicles = list(VConf.vehicles.keys())
        worlds = list(VConf.worlds.keys())
        agents = list(BOConf.agents.keys())
        fill_combobox(self.bg_agent_list, sorted(agents)) 
        fill_combobox(self.bg_world_list, sorted(worlds))
        fill_combobox(self.bg_vehicle_list, sorted(vehicles))

        self.h_vehicle_selected(None)
        self.h_world_selected(None)
        self.h_agent_selected(None)
        self.h_task_selected(None)
        

    def h_button_run(self, event): # wxGlade: MyFrame.<event_handler>
        id_vehicle = self.bg_vehicle_list.GetValue()
        id_agent = self.bg_agent_list.GetValue()
        id_world = self.bg_world_list.GetValue()
        
        output_dir = self.bg_output_dir.GetValue()
        
        create_vehicles_launch(id_agent=id_agent,
                               id_vehicle=id_vehicle,
                               id_world=id_world,
                               output_dir=output_dir)
        event.Skip()


def fill_combobox(widget, choices):
    widget.Clear()
    #if not choices:
    #    logger.warning('No choices for widget %s' % widget)
    for c in choices:
        #print('Adding %s to %s' % (c, widget))
        widget.Append(c)
    widget.SetValue(choices[0])
    
def main():
    BO_load_all()
    V_load_all()
    app = wx.PySimpleApp(0) #@UndefinedVariable
    wx.InitAllImageHandlers() #@UndefinedVariable
    frame_1 = BootGUIFrame(None, -1, "")
    frame_1.fill_choices()
    app.SetTopWindow(frame_1)
    
    frame_1.Show()
    app.MainLoop()
    
if __name__ == '__main__':
    main()
