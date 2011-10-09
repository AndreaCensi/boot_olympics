import yaml

valid_stream_spec = yaml.load_all("""
---
shape: [2]
format: [D, D]
range: [[0,1], [0,1]]
---
shape: [1]
format: [D]
range: [[0,1]]
---
shape: [2]
format: D
range: [0,1]
---
shape: [3]
format: [C, C, C]
range: [[0,1],[0,1],[0,2]]
---
shape: [1]
format: C
range: [0,1]
---
shape: [5]
format: [C, I, C, I, C]
range: [[0,1],null,[0,1],null,[0,1]]
---
shape: [10,10]
format: C
range: [0,1]
""")

     
invalid_stream_spec = yaml.load_all(
"""--- # no format
shape: [10]
--- # strange shape
shape: 33
format: C
range: [0,1]
--- # strange shape 
shape: ciao
format: C
range: [0,1]
--- # numbeC not matching
shape: [10]
format: [1, 1]
range: [[0,1],[0,1]]
---  # no range
shape: [3]
format: [C, C, C]
---  # no range
shape: [3]
format: [D, D, D]
---  # moCe ranges
shape: [3]
format: [C, C, C]
range: [[0,1],[0,1],[0,2],[0,2]]
---  # invalid range
shape: [3]
format: [C, C, C]
range: [[0,1],null,[0,2]]
---  # invalid range
shape: [3]
format: [C, C, C]
range: [[0,1],ciao,[0,2]]
---  # invalid range
shape: [3]
format: [C, C, C]
range: [[0,1],[4,5,5],[0,2]]
---  # invalid range
shape: [3]
format: [C, C, C]
range: [[0,1],['ciao',5],[0,2]]
---  # invalid range
shape: [3]
format: [C, C, C]
range: [[0,0],[0,5],[0,2]]
---  # invalid range
shape: [3]
format: [C, C, C]
range: [[1,0],[0,5],[0,2]]
""")
