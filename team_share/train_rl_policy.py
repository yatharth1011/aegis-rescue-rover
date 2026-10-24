import numpy as np
import matplotlib.pyplot as plt
import os

HERE = os.path.dirname(os.path.abspath(__file__))  # outputs land next to this script

"""
PROJECT A.E.G.I.S. — Reinforcement Learning Training & Quantization Pipeline
Domain: Autonomous Search & Rescue (SAR) Rover for TL101 Tinkering Lab, IIT Ropar

MDP Formulation:
- State Space S: Discretized tuple (d_front_bin, d_left_bin, d_right_bin, audio_angle_bin, target_visible)
- Action Space A: {0: FORWARD, 1: TURN_LEFT, 2: TURN_RIGHT, 3: REVERSE, 4: ENGAGE_CLAMP}
- Reward R(s, a):
    R = -α * Collision + β * Δd_target + γ * Docked - δ * Step + η * UnvisitedCell
"""

class RescueRoverSim:
    def __init__(self, grid_size=10):
        self.grid_size = grid_size
        self.reset()
        
    def reset(self):
        # 0: Empty, 1: Obstacle, 2: Target (Survivor)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=int)
        
        # Place static obstacles (furniture/debris)
        np.random.seed(42)
        obstacle_coords = [(2, 2), (2, 3), (3, 2), (5, 6), (5, 7), (6, 6), (7, 3), (8, 3)]
        for r, c in obstacle_coords:
            self.grid[r, c] = 1
            
        # Target position (Acoustic distress beacon / survivor)
        self.target_pos = np.array([8, 8])
        self.grid[self.target_pos[0], self.target_pos[1]] = 2
        
        # Rover start position & heading (0: North, 1: East, 2: South, 3: West)
        self.rover_pos = np.array([1, 1])
        self.rover_heading = 1 # Facing East
        
        self.visited = set()
        self.visited.add(tuple(self.rover_pos))
        self.steps = 0
        self.max_steps = 100
        
        return self._get_state()
        
    def _get_heading_vector(self):
        vectors = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        return np.array(vectors[self.rover_heading])
        
    def _get_state(self):
        # Sensor 1: Front distance (ToF)
        h_vec = self._get_heading_vector()
        front_pos = self.rover_pos + h_vec
        front_blocked = 1 if not (0 <= front_pos[0] < self.grid_size and 0 <= front_pos[1] < self.grid_size) or self.grid[front_pos[0], front_pos[1]] == 1 else 0
        
        # Sensor 2: Left distance
        left_h = (self.rover_heading - 1) % 4
        l_vec = np.array([(-1, 0), (0, 1), (1, 0), (0, -1)][left_h])
        left_pos = self.rover_pos + l_vec
        left_blocked = 1 if not (0 <= left_pos[0] < self.grid_size and 0 <= left_pos[1] < self.grid_size) or self.grid[left_pos[0], left_pos[1]] == 1 else 0
        
        # Sensor 3: Right distance
        right_h = (self.rover_heading + 1) % 4
        r_vec = np.array([(-1, 0), (0, 1), (1, 0), (0, -1)][right_h])
        right_pos = self.rover_pos + r_vec
        right_blocked = 1 if not (0 <= right_pos[0] < self.grid_size and 0 <= right_pos[1] < self.grid_size) or self.grid[right_pos[0], right_pos[1]] == 1 else 0
        
        # Sensor 4: Acoustic Incident Angle (TDoA) relative to current heading
        disp = self.target_pos - self.rover_pos
        target_angle_rad = np.arctan2(disp[1], disp[0]) # y: col, x: row
        heading_angles = [-np.pi/2, 0, np.pi/2, np.pi]
        rel_angle = target_angle_rad - heading_angles[self.rover_heading]
        rel_angle = (rel_angle + np.pi) % (2 * np.pi) - np.pi
        
        if rel_angle < -0.3:
            audio_bin = 0 # Target is to the LEFT
        elif rel_angle > 0.3:
            audio_bin = 2 # Target is to the RIGHT
        else:
            audio_bin = 1 # Target is STRAIGHT AHEAD
            
        # Sensor 5: Camera visual lock (Direct line of sight within 3 cells)
        dist = np.linalg.norm(disp)
        target_visible = 1 if dist <= 3.0 and audio_bin == 1 else 0
        
        return (front_blocked, left_blocked, right_blocked, audio_bin, target_visible)

    def step(self, action):
        self.steps += 1
        reward = -0.5 # Small step penalty to encourage shortest path
        done = False
        info = {}
        
        old_dist = np.linalg.norm(self.target_pos - self.rover_pos)
        
        if action == 0: # FORWARD
            h_vec = self._get_heading_vector()
            new_pos = self.rover_pos + h_vec
            if not (0 <= new_pos[0] < self.grid_size and 0 <= new_pos[1] < self.grid_size) or self.grid[new_pos[0], new_pos[1]] == 1:
                reward -= 50 # Severe collision penalty
                done = True
                info['event'] = 'collision'
            else:
                self.rover_pos = new_pos
                if tuple(self.rover_pos) not in self.visited:
                    reward += 2.0 # Exploration bonus
                    self.visited.add(tuple(self.rover_pos))
        elif action == 1: # TURN_LEFT
            self.rover_heading = (self.rover_heading - 1) % 4
        elif action == 2: # TURN_RIGHT
            self.rover_heading = (self.rover_heading + 1) % 4
        elif action == 3: # REVERSE
            h_vec = -self._get_heading_vector()
            new_pos = self.rover_pos + h_vec
            if not (0 <= new_pos[0] < self.grid_size and 0 <= new_pos[1] < self.grid_size) or self.grid[new_pos[0], new_pos[1]] == 1:
                reward -= 50
                done = True
                info['event'] = 'collision'
            else:
                self.rover_pos = new_pos
        elif action == 4: # ENGAGE_CLAMP
            if np.array_equal(self.rover_pos, self.target_pos) or np.linalg.norm(self.target_pos - self.rover_pos) <= 1.0:
                reward += 200 # Target salvage reward!
                done = True
                info['event'] = 'rescue_success'
            else:
                reward -= 10 # False clamp trigger penalty
                
        # Potential-based reward shaping for acoustic homing
        new_dist = np.linalg.norm(self.target_pos - self.rover_pos)
        reward += 5.0 * (old_dist - new_dist)
        
        if self.steps >= self.max_steps:
            done = True
            info['event'] = 'timeout'
            
        return self._get_state(), reward, done, info

def train_q_learning(episodes=1500, alpha=0.15, gamma=0.95, epsilon=1.0, epsilon_decay=0.995, min_epsilon=0.05):
    env = RescueRoverSim()
    # State space shape: 2 x 2 x 2 x 3 x 2 = 48 states. 5 actions.
    q_table = np.zeros((2, 2, 2, 3, 2, 5))
    
    episode_rewards = []
    successes = 0
    
    for ep in range(episodes):
        state = env.reset()
        total_reward = 0
        done = False
        
        while not done:
            if np.random.rand() < epsilon:
                action = np.random.randint(5)
            else:
                action = np.argmax(q_table[state])
                
            next_state, reward, done, info = env.step(action)
            total_reward += reward
            
            # Bellman Equation Update
            best_next_a = np.argmax(q_table[next_state])
            td_target = reward + gamma * q_table[next_state][best_next_a] * (0 if done and info.get('event') != 'timeout' else 1)
            td_error = td_target - q_table[state][action]
            q_table[state][action] += alpha * td_error
            
            state = next_state
            
            if done and info.get('event') == 'rescue_success':
                successes += 1
                
        epsilon = max(min_epsilon, epsilon * epsilon_decay)
        episode_rewards.append(total_reward)
        
    return q_table, episode_rewards, successes


def evaluate_greedy(q_table, episodes=100):
    """Greedy rollouts (no exploration). The map is fixed, so this measures
    how reliably the table solves the training map, not generalisation."""
    env = RescueRoverSim()
    outcomes = {}
    for _ in range(episodes):
        state, done, info = env.reset(), False, {}
        while not done:
            state, _, done, info = env.step(int(np.argmax(q_table[state])))
        outcomes[info.get('event')] = outcomes.get(info.get('event'), 0) + 1
    return outcomes

if __name__ == '__main__':
    print("=== Training A.E.G.I.S. RL Navigation Policy ===")
    q_table, rewards, successes = train_q_learning(episodes=2000)
    print(f"Training completed across 2000 episodes.")
    print(f"Mean reward (last 100 eps): {np.mean(rewards[-100:]):.2f}")
    print(f"Successful episodes during training (with exploration): {successes}/2000 = {successes / 20:.1f} %")
    outcomes = evaluate_greedy(q_table)
    print(f"Greedy evaluation, 100 episodes on the training map: {outcomes}")
    visited = int(np.sum(np.any(q_table != 0, axis=-1)))
    print(f"Q-table states with any trained value: {visited}/48")
    
    # Save training curve plot for the evaluation report
    plt.figure(figsize=(9, 4.5))
    smoothed_rewards = np.convolve(rewards, np.ones(50)/50, mode='valid')
    plt.plot(smoothed_rewards, color='#1e3a8a', linewidth=2, label='Running Average Reward (Window=50)')
    plt.title('A.E.G.I.S. RL Agent Training Curve (Q-Learning Convergence)', fontsize=13, fontweight='bold')
    plt.xlabel('Training Episodes', fontsize=11)
    plt.ylabel('Cumulative Episode Reward', fontsize=11)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(HERE, 'aegis_rl_training_curve.png'), dpi=300)
    plt.close()
    print("Saved training curve plot: aegis_rl_training_curve.png")
    
    # Export Q-Table to C++ Header for direct deployment on ESP32-S3!
    with open(os.path.join(HERE, 'aegis_q_table.h'), 'w') as f:
        f.write("// Auto-generated Q-Table weights for Project A.E.G.I.S. (ESP32-S3 Firmware)\n")
        f.write("// States: [front_block(2)][left_block(2)][right_block(2)][audio_angle(3)][target_vis(2)][actions(5)]\n\n")
        f.write("#ifndef AEGIS_Q_TABLE_H\n#define AEGIS_Q_TABLE_H\n\n")
        f.write("const float Q_TABLE[2][2][2][3][2][5] =\n")
        
        def format_arr(arr, indent=1):
            if arr.ndim == 1:
                return "\t" * indent + "{" + ", ".join(f"{x:.4f}f" for x in arr) + "}"
            lines = []
            for sub in arr:
                lines.append(format_arr(sub, indent + 1))
            return ("\t" * indent + "{\n") + ",\n".join(lines) + ("\n" + "\t" * indent + "}")
            
        f.write(format_arr(q_table, indent=1))
        f.write(";\n\n#endif // AEGIS_Q_TABLE_H\n")
        
    print("Exported embedded C++ header: aegis_q_table.h for ESP32-S3 deployment.")
