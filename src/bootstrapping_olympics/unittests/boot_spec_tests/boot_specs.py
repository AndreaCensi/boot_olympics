import yaml

valid_boot_specs = yaml.load_all("""
---
id:  
observations: 
    shape: [180]
    format: C
    range: [0,1]
commands:
    shape: [2]
    format: [C, C]
    range: [[-1,+1],[-1,+1]]
    names: ['linear velocity', 'angular velocity']
extra: 
    type: simulation
    random: 1
""")


invalid_boot_specs = yaml.load_all("""
---
id: mah
observations: 
    shape: [180]
    format: boh
commands:
    shape: [1]
    format: C
type: simulation
extra: 
  random: 1
""")
