import spynnaker8 as p
import numpy as np
from pyNN.utility.plotting import Figure, Panel
import matplotlib.pyplot as plt
from python_models.pendulum import Pendulum
from spinn_arm.python_models.arm import Arm
from spinn_front_end_common.utilities.globals_variables import get_simulator

def get_scores(game_pop, simulator):
    g_vertex = game_pop._vertex
    scores = g_vertex.get_data(
        'score', simulator.no_machine_time_steps, simulator.placements,
        simulator.graph_mapper, simulator.buffer_manager, simulator.machine_time_step)
    return scores.tolist()

def connect_to_arms(pre_pop, from_list, arms, r_type, plastic, stdp_model):
    arm_conn_list = []
    for i in range(len(arms)):
        arm_conn_list.append([])
    for conn in from_list:
        arm_conn_list[conn[1]].append((conn[0], 0, conn[2], conn[3]))
        # print "out:", conn[1]
        # if conn[1] == 2:
        #     print '\nit is possible\n'
    for i in range(len(arms)):
        if len(arm_conn_list[i]) != 0:
            if plastic:
                p.Projection(pre_pop, arms[i], p.FromListConnector(arm_conn_list[i]),
                             receptor_type=r_type, synapse_type=stdp_model)
            else:
                p.Projection(pre_pop, arms[i], p.FromListConnector(arm_conn_list[i]),
                             receptor_type=r_type)


runtime = 21000
exposure_time = 200
encoding = 1
time_increment = 20
pole_length = 1
pole_angle = 2.6
reward_based = 1
force_increments = 100
max_firing_rate = 50
number_of_bins = 3
central = 1

inputs = 2
outputs = 2

p.setup(timestep=1.0, min_delay=1, max_delay=127)
p.set_number_of_neurons_per_core(p.IF_cond_exp, 100)
# one of these variable can be replaced with test_data depending on what needs to be tested
input_model = Pendulum(encoding=encoding,
                       time_increment=time_increment,
                       pole_length=pole_length,
                       pole_angle=pole_angle,
                       reward_based=reward_based,
                       force_increments=force_increments,
                       max_firing_rate=max_firing_rate,
                       number_of_bins=number_of_bins,
                       central=central,
                       rand_seed=[np.random.randint(0xffff) for i in range(4)],
                       bin_overlap=2,
                       label='pendulum_pop')

pendulum_pop_size = input_model.neurons()
pendulum = p.Population(pendulum_pop_size, input_model)
null_pop = p.Population(4*number_of_bins, p.IF_cond_exp(), label='null')
p.Projection(pendulum, null_pop, p.OneToOneConnector(), p.StaticSynapse(weight=0.09))
null_pop.record(['spikes', 'v'])
# null_pop.record(['spikes', 'v'])

arm_collection = []
input_spikes = []
rates = [0, 0]
# rates = [0, 20]
# rates = [0, 10]
print 'rates = ', rates
for j in range(outputs):
    arm_collection.append(p.Population(int(np.ceil(np.log2(outputs))),
                                       Arm(arm_id=j, reward_delay=exposure_time,
                                           rand_seed=[np.random.randint(0xffff) for k in range(4)],
                                           no_arms=outputs, arm_prob=1),
                                           label='arm_pop{}'.format(j)))
    p.Projection(arm_collection[j], pendulum, p.AllToAllConnector(), p.StaticSynapse())
    # p.Projection(null_pop, arm_collection[j], p.AllToAllConnector())
    input_spikes.append(p.Population(1, p.SpikeSourcePoisson(rate=rates[j])))
    # p.Projection(input_spikes[j], arm_collection[j], p.AllToAllConnector(), p.StaticSynapse())
    p.Projection(pendulum, arm_collection[j], p.FromListConnector(arm_conns[j]))

from_list_conn_left = [[0, 0, 0.1, 1], [3, 0, 0.1, 1], [6, 0, 0.1, 1], [9, 0, 0.1, 1]]
from_list_conn_right = [[2, 0, 0.1, 1], [5, 0, 0.1, 1], [8, 0, 0.1, 1], [11, 0, 0.1, 1]]

simulator = get_simulator()
p.run(runtime)

scores = []
scores.append(get_scores(game_pop=pendulum, simulator=simulator))
print scores

spikes = null_pop.get_data('spikes').segments[0].spiketrains
v = null_pop.get_data('v').segments[0].filter(name='v')[0]
plt.figure("spikes out")
Figure(
    Panel(spikes, xlabel="Time (ms)", ylabel="nID", xticks=True),
    Panel(v, ylabel="Membrane potential (mV)", yticks=True)
)
plt.show()

print 'rates = ', rates
p.end()