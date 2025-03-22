from owmeta_core.command import OWM
conn = OWM().connect()

from owmeta_core.context import Context
ctx = conn(Context)(ident='http://openworm.org/data')

from owmeta.worm import Worm
net = ctx.stored(Worm).query().neuron_network()

neurons = set(net.neurons())
receptors = set()
for neuron in neurons:
    receptors = receptors | set(neuron.receptors())
receptors = sorted(list(receptors))

print(receptors)

with open('data/receptors.txt', 'w') as f:
    for receptor in receptors:
        f.write(f"{receptor}\n")


