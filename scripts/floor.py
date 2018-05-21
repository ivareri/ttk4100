# Datafile holding values for floor robot


# Denne inneholder navnet på alle
# leddene som er på roboten. Leddverdiene i 
# position variablene lenger ned må være i samme
# rekefølge som navnet på leddene.
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

# pregrab er punktet vi ønsker at roboten skal 
# være på før tar tak i eska med å klemme den
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
# list() gjør at vi lager en kopi av lista, og ikke bare
# lager en referanse til samme liste.
position['grab'] = list(position['pregrab'])

# For å unngå at vi roterer på eska mens vi løfter
# må vi kompansere med å forandre alle led. På gantry-roboten
# har vi en liner akse som vi bruker for å løfte, og trenger dermed
# ikke å endre på mer enn ett ledd.
position['lift'] = [
    1.0850, 
    -0.8190, 
    1.3316, 
    -1.8359, 
    1.0544, 
    0.8597
]

# Her kopierer vi lista, før vi roterer 
# A6 med 90 grader. Python er 0 indekstert,
# slik at elemnt 5 i lista er den 6 verdien.
position['rotate'] = list(position['lift'])
position['rotate'][5] += 1.5708

