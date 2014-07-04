import yaml

valid_xstream_spec = yaml.load_all("""
---
y1:
  shape: [2]
  format: [D, D]
  range: [[0,1], [0,1]]
y2:   
  shape: [2]
  format: [D, D]
  range: [[0,1], [0,1]]
---
shape: [2]
format: [D, D]
range: [[0,1], [0,1]]
""")


invalid_xstream_spec = yaml.load_all("""
---
y1:
  shape: [2]
  format: [D, D]
  range: [[0,1], [0,1]]
y2: 0
---
y1:
  shape: [2]
  format: [D, D]
  range: x
y2:   
  shape: [2]
  format: [D, D]
  range: [[0,1], [0,1]]
""")
