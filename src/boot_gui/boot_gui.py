from . import logger
from .boot_gui_generated import *
from .boot_gui_generated_mdi import *
from .boot_gui_run import create_vehicles_launch
from bootstrapping_olympics.loading.load_all import (Configuration as BOConf,
    load_configuration as BO_load_all)
from pprint import pformat
from vehicles.configuration.load_all import (Configuration as VConf,
    load_configuration as V_load_all)
from wxPython.wx import wxMDIParentFrame #@UnresolvedImport
import os
import subprocess
import wx
import yaml



#    
#class BootGUIFrame(MainFrame):
#    
#    def h_vehicle_selected(self, event): # wxGlade: MyFrame.<event_handler>
#        show_selection(VConf.vehicles, self.bg_vehicle_list, self.bg_vehicle_desc,
#                       self.bg_vehicle_config)
#        
#    def h_world_selected(self, event): # wxGlade: MyFrame.<event_handler>
#        show_selection(VConf.worlds, self.bg_world_list, self.bg_world_desc,
#                       self.bg_world_config)
#
#    def h_agent_selected(self, event): # wxGlade: MyFrame.<event_handler>
#        show_selection(BOConf.agents, self.bg_agent_list, self.bg_agent_desc,
#                       self.bg_agent_config)
#        
#    def h_task_selected(self, event): # wxGlade: MyFrame.<event_handler>
#        #print "hello: %s" % event        
#        pass
#        
#    def fill_choices(self):
#        try:
#            cmds = ['rospack', 'find', 'boot_olympics_launch']
#            self.output_dir = subprocess.check_output(cmds).strip() #@UndefinedVariable
#        except:
#            logger.warning('Could not find package using rospack.')
#            self.output_dir = os.curdir
#        logger.info('Using output dir:\n%s' % self.output_dir)
#        # self.bg_output_dir.SetValue(output_dir)
#        
#        vehicles = list(VConf.vehicles.keys())
#        worlds = list(VConf.worlds.keys())
#        agents = list(BOConf.agents.keys())
#        fill_combobox(self.bg_agent_list, sorted(agents)) 
#        fill_combobox(self.bg_world_list, sorted(worlds))
#        fill_combobox(self.bg_vehicle_list, sorted(vehicles))
#
#        self.h_vehicle_selected(None)
#        self.h_world_selected(None)
#        self.h_agent_selected(None)
#        self.h_task_selected(None)
#        
#
#    def h_button_run(self, event): # wxGlade: MyFrame.<event_handler>
#        id_vehicle = self.bg_vehicle_list.GetValue()
#        id_agent = self.bg_agent_list.GetValue()
#        id_world = self.bg_world_list.GetValue()
#        
#        #output_dir = self.bg_output_dir.GetValue()
#        output_dir = self.output_dir
#        
#        create_vehicles_launch(id_agent=id_agent,
#                               id_vehicle=id_vehicle,
#                               id_world=id_world,
#                               output_dir=output_dir)
#        event.Skip()


def fill_combobox(widget, choices):
    widget.Clear()
    #if not choices:
    #    logger.warning('No choices for widget %s' % widget)
    for c in choices:
        #print('Adding %s to %s' % (c, widget))
        widget.Append(c)
    if choices:
        widget.SetValue(choices[0])
    
class MyConfigFrame(ConfigFrame):
    def __init__(self, parent, choices, label, is_valid_config):
        self.parent = parent
        self.choices = choices
        self.label = label
        ConfigFrame.__init__(self, parent)
        ordered = sorted(list(choices.keys()))
        fill_combobox(self.bg_choice, ordered) 
        self.h_choice_changed()
        
        self.label = label
        self.Show(True)
        if not choices:
            logger.warning('No choices for %s' % label)
        self.SetTitle(label)
        
        self.is_valid_config = is_valid_config
        
    def h_choice_changed(self, event=None): 
        show_selection(self.choices, self.bg_choice,
                       self.bg_desc, self.bg_config)
        self.h_text()
        
    def h_save_pressed(self, event):  
        print('save')

    def h_text(self, event=None):
        try: 
            parsed = yaml.load(self.bg_config.GetValue())
        except Exception as e:
            # cannot parse YAML
            self.bg_config.SetForegroundColour(wx.Colour(50, 0, 0)) #@UndefinedVariable
            self.bg_config_status.SetLabel('Cannot parse YAML:\n%s' % e)
            self.bg_config.SetForegroundColour(wx.Colour(50, 0, 0)) #@UndefinedVariable
            return
        try:
            self.is_valid_config(parsed)
        except Exception as e:
            self.bg_config.SetForegroundColour(wx.Colour(50, 50, 0)) #@UndefinedVariable
            self.bg_config_status.SetLabel('Valid YAML, but invalid conf:\n%s' % e)
            self.bg_config.SetForegroundColour(wx.Colour(50, 50, 0)) #@UndefinedVariable
            
        self.bg_config.SetForegroundColour(wx.Colour(0, 0, 0)) #@UndefinedVariable
        self.bg_config_status.SetForegroundColour(wx.Colour(0, 0, 0)) #@UndefinedVariable
            
        self.bg_config_status.SetLabel('Good configuration.')
        
    
    def h_text_enter(self, event):
        pass #print('Text changed') 
        

def show_selection(choices, widget, label, config):
    selected = widget.GetValue()
    conf = dict(**choices[selected])
    desc = conf['desc']
    remove_fields = ['filename']
    for f in remove_fields:
        if f in conf:
            del conf[f]
    label.SetLabel(desc)
    y = yaml.dump(conf)
    config.SetValue(y)

def valid_agent_config(x):
    if not isinstance(x, dict):
        raise Exception('Must be a dictionary, not a %s.' % x.__class__.__name__)
    return True

class Parent(wxMDIParentFrame):

    def __init__(self):
        wxMDIParentFrame.__init__(self, None, -1,
                                  "Bootstrapping GUI", size=(500, 500))

        self.c_vehicle = MyConfigFrame(self,
                                      choices=VConf.vehicles,
                                      label='Vehicles',
                                      is_valid_config=valid_agent_config)
        self.c_world = MyConfigFrame(self,
                                      choices=VConf.worlds,
                                      label='Worlds',
                                      is_valid_config=valid_agent_config)
        self.c_agent = MyConfigFrame(self,
                                      choices=BOConf.agents,
                                      label='Agent',
                                      is_valid_config=valid_agent_config)
        
    def fill_choices(self):
        pass
    
def main():
    BO_load_all()
    V_load_all()
    app = wx.PySimpleApp(0) #@UndefinedVariable
    wx.InitAllImageHandlers() #@UndefinedVariable
    frame_1 = Parent()
    frame_1.fill_choices()
    app.SetTopWindow(frame_1)
    
    frame_1.Show()
    app.MainLoop()
    
if __name__ == '__main__':
    main()
