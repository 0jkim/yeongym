ns3-gym
============

[OpenAI Gym](https://gym.openai.com/) is a toolkit for reinforcement learning (RL) widely used in research. The network simulator [ns-3](https://www.nsnam.org/) is the de-facto standard for academic and industry studies in the areas of networking protocols and communication technologies. ns3-gym is a framework that integrates both OpenAI Gym and ns-3 in order to encourage usage of RL in networking research.

Installation
============

1. Install all required dependencies required by ns-3.
```
# minimal requirements for C++:
apt-get install gcc g++ python3 python3-pip cmake
```
Check [ns-3 requirements](https://www.nsnam.org/docs/tutorial/html/getting-started.html#prerequisites/)

2. Install ZMQ, Protocol Buffers and pkg-config libs:
```
sudo apt-get update
apt-get install libzmq5 libzmq3-dev
apt-get install libprotobuf-dev
apt-get install protobuf-compiler
apt-get install pkg-config
```
3. Clone ns3-gym repository in to `contrib` directory and change the branch:
```
cd ./contrib
git clone https://github.com/tkn-tub/ns3-gym.git ./opengym
cd opengym/
git checkout app-ns-3.36+
```
Check [working with cmake](https://www.nsnam.org/docs/manual/html/working-with-cmake.html)

It is important to use the `opengym` as the name of the ns3-gym app directory. 

3. Configure and build ns-3 project:
```
./ns3 configure --enable-examples
./ns3 build
```
Note: Opengym Protocol Buffer messages (C++ and Python) are build during configure.

4. Install ns3gym located in model/ns3gym (Python3 required)
```
cd ./contrib/opengym/
pip3 install --user ./model/ns3gym
```

5. (Optional) Install all libraries required by your agent (like tensorflow, keras, etc.).

6. Run example:
```
cd ./contrib/opengym/examples/opengym/ 
./simple_test.py
```

7. (Optional) Start ns-3 simulation script and Gym agent separately in two terminals (useful for debugging):
```
# Terminal 1
./ns3 run "opengym"

# Terminal 2
cd ./contrib/opengym/examples/opengym/ 
./test.py --start=0
```

Examples
========

All examples can be found [here](./examples/).

## Basic Interface

1. Example Python script. Note, that `gym.make('ns3-v0')` starts ns-3 simulation script located in current working directory.
```
import gym
import MyAgent
from ns3gym import ns3env

#env = gym.make('ns3-v0')  <--- causes some errors with the new OpenAI Gym framework, please use ns3env.Ns3Env()
env = ns3env.Ns3Env()
obs = env.reset()
agent = MyAgent.Agent()

while True:
  action = agent.get_action(obs)
  obs, reward, done, info = env.step(action)

  if done:
    break
env.close()
```
2. Any ns-3 simulation script can be used as a Gym environment. This requires only to instantiate OpenGymInterface and implement the ns3-gym C++ interface consisting of the following functions:
```
Ptr<OpenGymSpace> GetObservationSpace();
Ptr<OpenGymSpace> GetActionSpace();
Ptr<OpenGymDataContainer> GetObservation();
float GetReward();
bool GetGameOver();
std::string GetExtraInfo();
bool ExecuteActions(Ptr<OpenGymDataContainer> action);
```
Note, that the generic ns3-gym interface allows to observe any variable or parameter in a simulation.

A more detailed description can be found in our [Paper](http://www.tkn.tu-berlin.de/fileadmin/fg112/Papers/2019/gawlowicz19_mswim.pdf).

## Cognitive Radio
We consider the problem of radio channel selection in a wireless multi-channel environment, e.g. 802.11 networks with external interference. The objective of the agent is to select for the next time slot a channel free of interference. We consider a simple illustrative example where the external interference follows a periodic pattern, i.e. sweeping over all channels one to four in the same order as shown in the table.

<p align="center">
<img src="doc/figures/interferer-pattern.png" alt="drawing" width="500"/>
</p>

We created such a scenario in ns-3 using existing functionality from ns-3, i.e. interference created using `WaveformGenerator` class and sensing performed using `SpectrumAnalyzer` class.

Such a periodic interferer can be easily learned by an RL-agent so that based on the current observation of the occupation on each channel in a given time slot the correct channel can be determined for the next time slot avoiding any collision with the interferer.

Our proposed RL mapping is:
- observation - occupation on each channel in the current time slot, i.e. wideband-sensing,
- actions - set the channel to be used for the next time slot,
- reward - +1 in case of no collision with interferer; otherwise -1,
- gameover - if more than three collisions happened during the last ten time-slots

The figure below shows the learning performance when using a simple neural network with fully connected input and an output layer.
We see that after around 80 episodes the agent is able to perfectly predict the next channel state from the current observation hence avoiding any collision with the interference.

The full source code of the example can be found [here](./examples/interference-pattern/).

<p align="center">
<img src="doc/figures/cognitive-radio-learning.png" alt="drawing" width="600"/>
</p>

Note, that in a more realistic scenario the simple waveform generator in this example can be replaced by a real wireless technology like LTE unlicensed (LTE-U).


## RL-TCP
The proper RL-TCP agent example is still under development. However, we have already implemented and released two versions (i.e. time and event-based) of an interface allowing to monitor parameters of a TCP instance and control its `Congestion Window` and `Slow Start Threshold` -- see details [here](./examples/rl-tcp/tcp_base.py). Note, that both versions inherits from `TcpCongestionOps` and hence can be used as an argument for `ns3::TcpL4Protocol::SocketType`.

Moreover, using the event-based interface, we already have an example Python Gym agent that implements TCP NewReno and communicates with the ns-3 simulation process using ns3gym -- see [here](./examples/rl-tcp/tcp_newreno.py). The example can be used as a starting point to implement an RL-based TCP congestion control algorithms.

In order to run it, please execute:
```
cd ./contrib/opengym/examples/rl-tcp/
./test_tcp.py
```

Or in two terminals:
```
# Terminal 1:
./ns3 run "rl-tcp --transport_prot=TcpRl"

# Terminal 2:
cd ./contrib/opengym/examples/rl-tcp/
./test_tcp.py --start=0
```

Note, that our Python TCP NewReno implementation achieves the same number of transmitted packets like the one implemented in ns3 (see the output of ns-3 simulation, i.e. `RxPkts: 5367` in both cases). Please execute the following command to cross-check:
```
./ns3 run "rl-tcp --transport_prot=TcpNewReno"
```

Contact
============
* Piotr Gawlowicz, TU-Berlin, gawlowicz@tkn
* Anatolij Zubow, TU-Berlin, zubow@tkn
* tkn = tkn.tu-berlin.de

How to reference ns3-gym?
============

Please use the following bibtex :

```
@inproceedings{ns3gym,
  Title = {{ns-3 meets OpenAI Gym: The Playground for Machine Learning in Networking Research}},
  Author = {Gaw{\l}owicz, Piotr and Zubow, Anatolij},
  Booktitle = {{ACM International Conference on Modeling, Analysis and Simulation of Wireless and Mobile Systems (MSWiM)}},
  Year = {2019},
  Location = {Miami Beach, USA},
  Month = {November},
  Url = {http://www.tkn.tu-berlin.de/fileadmin/fg112/Papers/2019/gawlowicz19_mswim.pdf
}
```
