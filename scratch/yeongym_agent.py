import numpy as np
import random

class YeongymAgent:
    def __init__(self, max_aoi=10000, max_cqi=15, aoi_bins=10, cqi_bins=5, num_weight_bins=11, epsilon=0.1, alpha=0.5, gamma=0.99,inference_mode=False):
        self.max_aoi = max_aoi
        self.max_cqi = max_cqi
       
        self.aoi_bins = aoi_bins
        self.cqi_bins = cqi_bins
       

        self.num_weight_bins = num_weight_bins  # 가중치 bin 수 (0.0 ~ 1.0 사이를 나누는 칸수)
        
        self.epsilon = epsilon  # exploration 확률
        self.alpha = alpha      # learning rate
        self.gamma = gamma      # discount factor
        
        self.q_table = np.zeros((self.aoi_bins, self.cqi_bins, self.num_weight_bins))
        
        self.last_state = None
        self.last_action = None

        self.inference_mode = inference_mode

    def discretize(self, aoi, cqi):
        aoi_bin = min(int((aoi / self.max_aoi) * self.aoi_bins), self.aoi_bins-1)
        cqi_bin = min(int((cqi / self.max_cqi) * self.cqi_bins), self.cqi_bins-1)
       
        return aoi_bin, cqi_bin

    def act(self, observation):
        actions = []
        self.current_states = []

        print(f"받은 observation 길이 = {len(observation)}")
        print(f"obs 내용: {observation}")

        for i in range(0, len(observation), 6): # sinr 추가시 7로 변경경
            aoi = observation[i + 4]
            cqi = observation[i + 5]
            # sinr = observation[i + 6]

            aoi_bin, cqi_bin= self.discretize(aoi, cqi)
            self.current_states.append((aoi_bin, cqi_bin))

            if (not self.inference_mode) and random.uniform(0, 1) < self.epsilon:
                action = np.random.uniform(0.0, 1.0)
            else:
                q_values = self.q_table[aoi_bin, cqi_bin]
                best_index = np.argmax(q_values)
                action = best_index / (self.num_weight_bins-1)

            actions.append(float(action))  # ns3gym은 weight처럼 받으니 float으로 변환

        print(f"[Agent] Action weights: {actions}")
        self.last_action = actions
        self.last_state = self.current_states

        return actions

    def learn(self, reward):
        if self.last_state is None or self.last_action is None:
            return
        
        for idx, (aoi_bin, cqi_bin) in enumerate(self.last_state):
            action_bin = int(round(self.last_action[idx] * (self.num_weight_bins - 1)))

            current_q = self.q_table[aoi_bin, cqi_bin, action_bin]
            max_next_q = np.max(self.q_table[aoi_bin, cqi_bin])

            # Q-learning update
            new_q = current_q + self.alpha * (reward + self.gamma * max_next_q - current_q)
            self.q_table[aoi_bin, cqi_bin, action_bin] = new_q
        
        print(f"[Learn] State (AoI_bin={aoi_bin}, CQI_bin={cqi_bin}, "
              f"Action_bin={action_bin}, "
              f"Reward={reward:.3f}, "
              f"Q(before)={current_q:.3f} -> Q(after)={new_q:.3f}")
    
    def save_q_table(self, filename = "q_table.npy"):
        np.save(filename, self.q_table)
        print(f"[Agent] Q-Table saved to {filename}")
    
    def load_q_table(self, filename = "q_table.npy"):
        self.q_table = np.load(filename)
        print(f"[Agent] Q-Table loaded from {filename}")