# Datafile holding values for floor robot
joint_names = [
    "floor_joint_a1",
    "floor_joint_a2",
    "floor_joint_a3",
    "floor_joint_a4",
    "floor_joint_a5",
    "floor_joint_a6"
]
	
position = {}
position['home'] = [
    1.5708,
    -1.5708,
     1.5708,
     0.0,
     0.0,
     0.0
]

position['pregrab'] = [
    1.0850, 
    -0.3866, 
    1.2640, 
    -2.0048, 
    1.1811, 
    1.2412
]

position['grab'] = list(position['pregrab'])

position['lift'] = [
    1.0850, 
    -0.8190, 
    1.3316, 
    -1.8359, 
    1.0544, 
    0.8597
]

position['rotate'] = list(position['lift'])
position['rotate'][5] += 1.5708

