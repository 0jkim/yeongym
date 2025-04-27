import time
import gym
import numpy as np
import argparse
import matplotlib.pyplot as plt

from yeongym_agent import YeongymAgent
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
agent = YeongymAgent()

rewards = []
step_count = 0

max_train_steps = 5000

while not done and step_count < max_train_steps:
    # print("\n=== Observation per Flow ===")
    for i in range(0, len(obs), 6): # sinr 추가시 7로 변경경
        rnti = obs[i]
        lcid = obs[i+1]
        priority = obs[i+2]
        hol_delay = obs[i+3]
        aoi = obs[i+4]
        cqi = obs[i+5]
        # sinr = obs[i+6]
        # print(f"[Flow {i//6}] RNTI: {rnti}, LCID: {lcid}, Priority: {priority}, "
        #       f"HOL Delay: {hol_delay}, AoI: {aoi}, CQI: {cqi}")
    action = agent.act(obs)
    obs, reward, done, info = env.step(action)
    agent.learn(reward)
    rewards.append(reward)
    step_count += 1
    
    # print(f"Obs: {obs}, Reward: {reward}, Done: {done}, Info: {info}")
    # print(f"[Proxy] Action sent to env: {action}")
print("\n[INFO] Training phase complete. Switching to Inference mode.\n")
agent.save_q_table()
agent.load_q_table()
agent.inference_mode = True

while not done:
    action = agent.act(obs)
    obs, reward, done, info = env.step(action)
    # Inference 모드니까 learn()은 호출하지 않음
    rewards.append(reward)
    step_count += 1

env.close()

print("[INFO] Total steps (Train + Inference):", step_count)

# === Reward History Plot ===
plt.figure(figsize=(12, 6))
plt.plot(rewards, label="Reward per Step", color="blue")
plt.xlabel("Step", fontsize=14)
plt.ylabel("Reward", fontsize=14)
plt.title("Reward History Over Time", fontsize=16)
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()