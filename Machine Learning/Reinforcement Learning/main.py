import numpy as np
import matplotlib.pyplot as plt
from qlearning_agent import QLearningAgent
from grid_world import GridWorld

# 定数
NB_EPISODE = 100    # エピソード数
EPSILON = .1    # 探索率
ALPHA = .1      # 学習率
GAMMA = .90     # 割引率
ACTIONS = np.arange(4)  # 行動の集合

if __name__ == '__main__':
    grid_env = GridWorld()  # grid worldの環境の初期化
    ini_state = grid_env.start_pos  # 初期状態（エージェントのスタート地点の位置）
    # エージェントの初期化
    agent = QLearningAgent(
        alpha=ALPHA,
        gamma=GAMMA,
        epsilon=EPSILON,  # 探索率
        actions=ACTIONS,   # 行動の集合
        observation=ini_state)  # Q学習エージェント
    rewards = []    # 評価用報酬の保存
    actions = []    # 評価用でそのエピソードの行動回数を保存
    is_end_episode = False  # エージェントがゴールしてるかどうか？

    # 実験
    for episode in range(NB_EPISODE): #ゴールするまでが1エピソード
        episode_reward = []  # 1エピソードの累積報酬
        while(is_end_episode == False):    # ゴールするまで続ける
            print("position:", grid_env.agent_pos)
            action = agent.act()  # 行動選択
            state, reward, is_end_episode = grid_env.step(action)
            while(reward is None): #rewardがNoneならずっと回す
                agent.set_wall(action, state)
                action = agent.act()  # 行動選択
                state, reward, is_end_episode = grid_env.step(action)
            print("Action!", action)
            #print("observe:", state, reward)
            agent.observe(state, reward)   # 状態と報酬の観測
            episode_reward.append(reward)
        #rewards.append(np.sum(episode_reward))  # このエピソードの平均報酬を与える
        rewards.append(np.sum(episode_reward) / len(episode_reward))
        actions.append(len(episode_reward))
        state = grid_env.reset()  # 初期化
        agent.observe(state)    # エージェントを初期位置に
        is_end_episode = False

    for state, q_list in agent.q_values.items():
        print(state, q_list)
    print(len(agent.q_values))
    # 結果のプロット
    plt.plot(np.arange(NB_EPISODE), actions)
    plt.xlabel("episode")
    plt.ylabel("The number of action")
    plt.savefig("result.jpg")
    plt.show()
