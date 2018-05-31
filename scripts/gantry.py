# Data file holding variables used for gantry robot
# Se floor.py for utfyllende kommentarer

joint_names = [
    "gantry_joint_a1",
    "gantry_joint_a2",
    "gantry_joint_a3",
    "gantry_joint_a4",
    "gantry_joint_a5",
    "gantry_joint_a6",
    "gantry_joint_e1",
    "gantry_joint_e2",
    "gantry_joint_e3"
]

position = {}
position['home'] = [
    -1.5708,
    -1.5708,
     1.5708,
     0.0,
     0.0,
     0.0,
     0.532,
     3.5321,
     -0.8921
]

position['pregrab'] = [
    -2.0000, 
    -1.8866, 
    1.2640, 
    -1.4000, 
    1.1811, 
    4.36,
    1.17,
    3.7,
    -1.0121
]

position['grab'] = list(position['pregrab'])
position['grab'][7] += -0.05


position['lift'] = list(position['grab'])
position['lift'][8] += 0.4


position['rotate'] = list(position['lift'])
position['rotate'][5] += -1.5708

position['release'] = list(position['rotate'])
position['release'][7] += 0.05

