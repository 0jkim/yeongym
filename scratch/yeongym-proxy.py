import time
import gym
import numpy as np
import argparse

from gym import spaces
from ns3gym import ns3env

# === Argument Parser ===
parser = argparse.ArgumentParser()
parser.add_argument("--port", type=int, default=5555, help="Port to connect with ns-3 environment")
parser.add_argument("--seed", type=int, default=0, help="Seed for environment reproducibility")
parser.add_argument("--simTime", type=int, default=10, help="Simulation time")
args = parser.parse_args()

# === ns-3 환경과 연결 ===
env = ns3env.Ns3Env(
    port=args.port,
    stepTime=0.1,
    startSim=False, 
    simArgs={
        # "--simTime": str(args.simTime),
        # "--simSeed": str(args.seed)
    }
)

obs = env.reset()
done = False

action_space = env.action_space
# === Dummy Agent: Random Action 선택 ===
while not done:
    print("\n=== Observation per Flow ===")
    for i in range(0, len(obs), 6):
        rnti = obs[i]
        lcid = obs[i+1]
        priority = obs[i+2]
        hol_delay = obs[i+3]
        aoi = obs[i+4]
        cqi = obs[i+5]
        print(f"[Flow {i//6}] RNTI: {rnti}, LCID: {lcid}, Priority: {priority}, "
              f"HOL Delay: {hol_delay}, AoI: {aoi}, CQI: {cqi}")
    action = action_space.sample()  # 이후 agent에서 받아오는 로직으로 대체
    obs, reward, done, info = env.step(action)
    print(f"Obs: {obs}, Reward: {reward}, Done: {done}, Info: {info}")
    print(f"Action space Output {env.action_space}")

env.close()
