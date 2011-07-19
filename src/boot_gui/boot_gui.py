from . import logger
from .boot_gui_mdi import ConfigFrame, MainFrame
from .boot_gui_run import create_vehicles_launch
from bootstrapping_olympics.loading.load_all import (Configuration as BOConf,
    load_configuration as BO_load_all)
from vehicles import (VehiclesConfiguration as VConf,
    load_configuration as V_load_all, check_valid_world_config,
    check_valid_vehicle_config)

from wx import MDIParentFrame #@UnresolvedImport

import subprocess
import wx
import yaml



def fill_combobox(widget, choices):
    widget.Clear()
    for c in choices:
        widget.Append(c)
    if choices:
        widget.SetSelection(0)
    
class MyConfigFrame(ConfigFrame):
    def __init__(self, parent, choices, label, is_valid_config, id_custom):
        ConfigFrame.__init__(self, parent)
        
        self.id_custom = id_custom
        self.editing_custom = False
        self.parent = parent
        self.choices = choices
        self.label = label
        self.ordered = sorted(list(choices.keys()))
        fill_combobox(self.bg_choice, self.ordered)
        
        self.react_to_changes = False 
        self.update_config()
        
        self.label = label
        self.Show(True)
        if not choices:
            logger.warning('No choices for %s' % label)
        self.SetTitle(label)
        
        self.is_valid_config = is_valid_config
        self.react_to_changes = True
         
        
    def update_config(self):
        self.react_to_changes = False
        show_selection(self.choices, self.bg_choice,
                       self.bg_desc, self.bg_config)
        self.react_to_changes = True
        
    def h_choice_changed(self, event=None):
        if self.editing_custom:
            self.editing_custom = False
            # print('Go back to not custom')
            # reset labels
            current = self.bg_choice.GetStringSelection()
            fill_combobox(self.bg_choice, self.ordered)
            self.bg_choice.SetStringSelection(current)

        self.update_config()
        
    def h_save_pressed(self, event):  
        print('save')

    def h_text(self, event=None):
        if self.react_to_changes and not self.editing_custom:
            #print('Editing custom')
            self.editing_custom = True
            
            self.editing_custom = True
            self.bg_choice.Clear()
            fill_combobox(self.bg_choice, self.ordered)
            self.bg_choice.Append(self.id_custom)
            n = self.bg_choice.GetCount()
            self.bg_choice.SetSelection(n - 1)
            
        if self.editing_custom:
            self.check_correctness()
        else:
            self.bg_config_status.SetLabel('Stock configuration.')

        
    def check_correctness(self):
        try: 
            parsed = yaml.load(self.bg_config.GetValue())
        except Exception as e:
            # cannot parse YAML
#            self.bg_config.SetForegroundColour(wx.Colour(50, 0, 0)) #@UndefinedVariable
            self.bg_config_status.SetLabel('Cannot parse YAML:\n%s' % e)
            self.bg_config_status.SetForegroundColour(wx.Colour(100, 0, 0)) #@UndefinedVariable
            self.bg_config_status.Layout()
            self.Layout()
            return
        try:
            if not 'id' in parsed:
                parsed['id'] = self.id_custom
            self.is_valid_config(parsed)
        except Exception as e:
#            self.bg_config.SetForegroundColour(wx.Colour(100, 50, 0)) #@UndefinedVariable
            self.bg_config_status.SetLabel('Valid YAML, but invalid conf:\n%s' % e)
            self.bg_config_status.SetForegroundColour(wx.Colour(100, 50, 0)) #@UndefinedVariable
            self.bg_config_status.Layout()
            self.Layout()
            return
        
        self.bg_config.SetForegroundColour(wx.Colour(0, 0, 0)) #@UndefinedVariable
        self.bg_config_status.SetForegroundColour(wx.Colour(0, 0, 0)) #@UndefinedVariable
            
        self.bg_config_status.SetLabel('Good configuration.' + ' ' * 20)
        self.bg_config_status.Layout()
        self.Layout()
        parsed['id'] = self.id_custom
        parsed['filename'] = 'custom_entry'
        self.choices[self.id_custom] = parsed 
    
    def h_text_enter(self, event):
        pass #print('Text changed')
    
    def get_selection(self):
        return self.bg_choice.GetStringSelection() 
        

def show_selection(choices, widget, label, config):
    selected = widget.GetStringSelection()
    conf = dict(**choices[selected])
    desc = conf['desc']
    remove_fields = ['filename', 'id']
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

class Parent(MDIParentFrame):

    def __init__(self):
        MDIParentFrame.__init__(self, None, -1,
                                  "Bootstrapping GUI", size=(1024, 500))

        try:
            cmds = ['rospack', 'find', 'boot_olympics_launch']
            output_dir = subprocess.check_output(cmds).strip() #@UndefinedVariable
        except:
            logger.warning('Could not find package using rospack.')
            output_dir = '/Users/andrea/scm/boot11_env/src/bootstrapping_olympics/ros-packages/boot_olympics_launch'
        logger.info('Using output dir:\n%s' % output_dir)
        

        self.c_vehicle = MyConfigFrame(self,
                                      choices=VConf.vehicles,
                                      label='Vehicles',
                                      is_valid_config=check_valid_vehicle_config,
                                      id_custom='custom_vehicle')
        self.c_world = MyConfigFrame(self,
                                      choices=VConf.worlds,
                                      label='Worlds',
                                      is_valid_config=check_valid_world_config,
                                      id_custom='custom_world')
        self.c_agent = MyConfigFrame(self,
                                      choices=BOConf.agents,
                                      label='Agent',
                                      is_valid_config=valid_agent_config,
                                      id_custom='custom_agent')
        
        csize = (800, 400)
        
        self.c_agent.SetSize(csize)
        self.c_world.SetSize(csize)
        self.c_vehicle.SetSize(csize)
        self.SetSize((800, 500))
        
        def run(log_level, viz_level, pub_interval):
            id_vehicle = self.c_vehicle.get_selection()
            id_agent = self.c_agent.get_selection()
            id_world = self.c_world.get_selection()
            pub_interval = {0: 0, 1: 100, 2: 1000}[pub_interval]
            create_vehicles_launch(id_agent=id_agent,
                               id_vehicle=id_vehicle,
                               id_world=id_world,
                               output_dir=output_dir,
                               log_level=log_level,
                               viz_level=viz_level,
                               publish_interval=pub_interval)

        class MyMain(MainFrame):
            def h_button_run(self, event=None):
                log_level = self.bg_logging.GetSelection()
                viz_level = self.bg_visualization.GetSelection()
                pub_interval = self.bg_publish.GetSelection()
                run(log_level, viz_level, pub_interval)
                
        self.main = MyMain(self)
        self.main.Show(True)
        
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
