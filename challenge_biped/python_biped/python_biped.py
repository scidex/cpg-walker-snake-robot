import matplotlib.pyplot as plt
import numpy as np

# General constants
PI = 3.1415926535897932384626433832795
PI_ = 3.1415
HALF_PI = 1.5707963267948966192313216916398
TWO_PI = 6.283185307179586476925286766559
DEG_TO_RAD = 0.017453292519943295769236907684886
RAD_TO_DEG = 57.295779513082320876798154814105

# Global parameters
mySpeed = 100
hipstep = 50
torsoStep = 120
myd = 5
myd2 = 1000
myd3 = 20
hight_factor = 2

delta_phi = 3

time_step = 0.1
tau = 1  # same effect as tau

TO = 0
LH = 1
RH = 2
LK = 3
RK = 4
LA = 5
RA = 6

motor_id = {
    0: 'TO',
    1: 'LH',
    2: 'RH',
    3: 'LK',
    4: 'RK',
    5: 'LA',
    6: 'RA'
}

# Define the initial positions
knee_shift = 200
hip_shift = 75
ankle_shift = 30
torso_shift = 0
initial_pos = [512 - torso_shift + 15, 512 - hip_shift + 8, 512 + hip_shift - 8, 512 - knee_shift, 512 + knee_shift,
               512 - ankle_shift, 512 + ankle_shift + 10]

'''
int initial_pos[7]={ 512-torso_shift+15 , 512-hip_shift+8, 512+hip_shift-8, 512-knee_shift , 512+knee_shift , 512-ankle_shift , 512+ankle_shift+10};
// Works fine: int initial_pos[7]={ 512-torso_shift+15 , 512-hip_shift+8, 512+hip_shift-8, 512-knee_shift , 512+knee_shift , 512-ankle_shift , 512+ankle_shift+10};
int phi[7] = {60,0,0,-180,-180,135,-135}; // [deg]
int range[7]={150,50,50,50,50,25,25};
'''

'''
/******************************************************/ 
#struct RSneuron 
/******************************************************/
'''


class RSneuron:
    def __init__(self):
        self.x = 0  # membrane potential
        self.x_old = 0  # old membrane potential --> for calculating the new one
        self.y = 0  # output response of neuron
        # tau = 1   # time const.
        self.s = 0.0  # impulse rate | outside network stimulation (we can set it) # Increase: amplitude increase
        self.b = 2.5  # adaptation coefficient b = 0, 2.5, inf # increase: decreases amplitude, increases frequ.
        self.x_prime = 0  # degree of adaptation / rate of change # No interesting behavior changes
        self.x_prime_old = 0
        self.y_old = 0
        self.T = 100  # prev. 12 time constant for adaptation T = 2.5, 12, inf # time period / 1/f
        self.threshold = 0.0  # threshold # derivative at start is lower


number_neurons = 9

rs_neuron = []
for i in range(0, number_neurons):
    rs_neuron.append(RSneuron())

delta_phi = 3
#######################################
#             Setup neurons           #
#######################################
# Torso (2 neurons)
rs_neuron[0].T = 42
rs_neuron[0].s = 1.0
rs_neuron[1].T = 42
rs_neuron[1].s = 1.0

# LK
# neuron | LK_dt:
rs_neuron[2].b = 10.0
rs_neuron[2].threshold = 0.3

# LH
# neuron | LH_dt:
rs_neuron[3].b = 5.0

# RK
# neuron | RK_dt:
rs_neuron[4].b = 10.0
rs_neuron[4].threshold = 0.3

# RH
# neuron | RH_dt:
rs_neuron[5].b = 5.0

# Recurrent connection RH
rs_neuron[6].b = 0.5
# rs_neuron[6].threshold = 0.7

# Hip forward neuron
rs_neuron[7].b = 0.5
rs_neuron[7].threshold = 9  # 39.5 # 38 - 39.7 # 7: still too big curve
rs_neuron[7].T = 100
# rs_neuron[7].s = -1

# Hip output neuron
rs_neuron[8].b = 0.00000002
rs_neuron[8].threshold = 0.0  # 39.5 # 38 - 39.7 # 7: still
rs_neuron[8].T = 1000
# rs_neuron[7].s = -1

print(rs_neuron[0].b)
print(rs_neuron[0].threshold)

#######################################
#           Connect neurons           #
#######################################

connection_mat = np.zeros((number_neurons, number_neurons))

# CONNECTION INITIALIZATION
# Connect both TO neurons:
connection_mat[0][1] = -2.5
connection_mat[1][0] = -2.5

# Connect
# LK to TO:
connection_mat[2][0] = 2.5

# Connect
# LH to TO:
connection_mat[3][0] = 2.5

# Connect
# RK to TO:
connection_mat[4][1] = 2.5

# Connect
# RH to TO:
connection_mat[5][1] = 2.5

# Recurrent
# RH
connection_mat[6][5] = 2.5
connection_mat[6][6] = 1.07

# RH_move_fw
connection_mat[7][6] = 2.5

# Hip
# output neuron
connection_mat[8][7] = 2.5

print(connection_mat)

#######################################
#             Simulation              #
#######################################

prev_torso = 0
print_neuron_output = False
time_length = 2500

TO_current = 0
LH_current = 0
RH_current = 0
LK_current = 0
RK_current = 0
LA_current = 0
RA_current = 0

all_current_list = np.zeros((7, time_length))
print(all_current_list.shape)

for t in range(0, time_length):
    # Connection: Input from j to i! (connection_mat[1][2] : If neuron 2 is firing, neuron 1 receives an input depending on the connection strength defined in connection_mat[1][2].

    for i in range(0, number_neurons):
        network_stimulus = 0
        for j in range(0, number_neurons):
            network_stimulus += connection_mat[i][j] * rs_neuron[j].y

        stimulus = network_stimulus + rs_neuron[i].s

        # Calculate membrane potential
        rs_neuron[i].x = (1 - time_step / tau) * rs_neuron[i].x_old + time_step * (
                stimulus - (rs_neuron[i].b * rs_neuron[i].x_prime_old)) / tau

        # Calculate adaptation rate.
        rs_neuron[i].x_prime = (1 - time_step / rs_neuron[i].T) * rs_neuron[i].x_prime_old + time_step * rs_neuron[
            i].y / rs_neuron[i].T

        # Calculate firing rate / "output" activation
        rs_neuron[i].y = max(0.0, rs_neuron[i].x_old - rs_neuron[i].threshold)

        # Saving values for future calculations.
        rs_neuron[i].x_old = rs_neuron[i].x
        rs_neuron[i].x_prime_old = rs_neuron[i].x_prime

    # TO
    torso = rs_neuron[0].y - rs_neuron[1].y

    TO_current = initial_pos[TO] - (torsoStep * min(1.5, torso * 1.5))
    LK_current = initial_pos[LK] - (hipstep * max(-2.1, -2.1 * rs_neuron[2].y))
    LH_current = initial_pos[LH] - (hipstep * min(1.3, 1.3 * rs_neuron[3].y))
    RK_current = initial_pos[RK] - (hipstep * min(2.1, 2.1 * rs_neuron[4].y))
    RH_current = initial_pos[RH] - (hipstep * max(-1.3, -1.3 * rs_neuron[5].y))

    all_current_list[0, t] = TO_current
    all_current_list[1, t] = LH_current
    all_current_list[2, t] = RH_current
    all_current_list[3, t] = LK_current
    all_current_list[4, t] = RK_current
    all_current_list[5, t] = LA_current
    all_current_list[6, t] = RA_current

fig, ax = plt.subplots()
for motor in range(0, 5):
    ax.plot(range(0, time_length), all_current_list[motor], label=motor_id[motor])
ax.legend()
plt.show()
