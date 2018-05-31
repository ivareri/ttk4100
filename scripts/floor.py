# Datafile holding values for floor robot


# Denne inneholder navnet paa alle
# leddene som er paa roboten. Leddverdiene i 
# position variablene lenger ned maa vaere i samme
# rekefolge som navnet paa leddene.
joint_names = [
    "floor_joint_a1",
    "floor_joint_a2",
    "floor_joint_a3",
    "floor_joint_a4",
    "floor_joint_a5",
    "floor_joint_a6"
]

# Lag en tom dictonary
position = {}

# Sett verdiene til 'home', som vi bruker 
# som utgangspunkt for bevegelsene
position['home'] = [
    1.5708,
    -1.5708,
     1.5708,
     0.0,
     0.0,
     0.0
]

# pregrab er punktet vi onsker at roboten skal 
# vaere paa for tar tak i eska med aa klemme den
# mellom robotarmene
position['pregrab'] = [
    1.0850, 
    -0.3866, 
    1.2640, 
    -2.0048, 
    1.1811, 
    1.2412
]

# For gulv-roboten er grab og pregrab posisjonen lik.
# list() gjor at vi lager en kopi av lista, og ikke bare
# lager en referanse til samme liste.
position['grab'] = list(position['pregrab'])

# For aa unngaa at vi roterer paa eska mens vi lofter
# maa vi kompansere med aa forandre alle led. Paa gantry-roboten
# har vi en liner akse som vi bruker for aa lofte, og trenger dermed
# ikke aa endre paa mer enn ett ledd.
position['lift'] = [
    1.0850, 
    -0.8190, 
    1.3316, 
    -1.8359, 
    1.0544, 
    0.8597
]

# Her kopierer vi lista, for vi roterer 
# A6 med 90 grader. Python er 0 indekstert,
# slik at elemnt 5 i lista er den 6 verdien.
position['rotate'] = list(position['lift'])
position['rotate'][5] += 1.5708

# Her kopierer vi lista. Paa gantry roboten 
# flytter vi akse 8 med 0.05m.  
position['release'] = list(position['rotate'])


