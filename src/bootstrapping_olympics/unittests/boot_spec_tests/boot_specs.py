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
---
observations:
  shape: [10,10]
  format: C
  range: [0,1]
commands:
  shape: [2]
  format: C
  range: [-1,+1]
  default: [0,0]
---
observations:
  shape: [5]
  format: [C, I, C, I, C]
  range: [[0,1],null,[0,1],null,[0,1]]
commands:
  shape: [2]
  format: C
  range: [-1,+1]
  default: [0,0]
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
