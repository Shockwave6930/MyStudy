import copy
import numpy as np
import random

class QLearningAgent:
    """
        Q学習 エージェント
    """

    def __init__(
            self,
            alpha=.2,
            epsilon=.1,
            gamma=.99,
            actions=None,
            observation=None):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.reward_history = [] #報酬の記録用
        self.actions = actions
        self.state = str(observation) #stateは(x, y)のタプル
        self.ini_state = str(observation)
        self.previous_state = None #とりあえずNoneで置いてるだけで、宣言部でGridWorldクラス内のstart.posが代入される
        self.previous_action = None
        self.q_values = self._init_q_values() #q_values[self.state] = np.array([0(↑), 0(→), 1(↓), 0(←)])のような形で行動してたどり着いた状態が初めてである時に初めて追加される。最初は初期状態の行動価値以外記録されていない

    def _init_q_values(self):
        """
           Q テーブルの初期化
        """
        q_values = {} #dict型　q_values[self.state] = np.array([0(↑), 0(→), 1(↓), 0(←)])のように各座標に対してactionの数だけ値を持つ
        q_values[self.state] = np.repeat(0.0, len(self.actions)) #Q値は各座標における上下左右の移動の数(self.action)だけ必要。今回は初期状態のみ初期値0で埋める
        return q_values

    def init_state(self): #用意されてるけど使用されてないので不要
        """
            状態の初期化
        """
        self.previous_state = copy.deepcopy(self.ini_state)
        self.state = copy.deepcopy(self.ini_state)
        return self.state

    def act(self):
        # ε-greedy選択
        if np.random.uniform() < self.epsilon:  # np.random.uniform()は[0, 1]の間で一様乱数を出力. epsilonよりも小さければrandom行動
            action = np.random.randint(0, len(self.q_values[self.state])) #np.random.randiant()で[0, 4(各座標における選択可能な行動の数)]の整数を出力(行動選択)
            while(np.isnan(action)):
                action = np.random.randint(0, len(self.q_values[self.state]))
            #print("epsilon:", self.q_values[self.state], self.state, action)
        else:   # greedy行動(最大の行動価値を持つ行動を選択)
            index = np.where(self.q_values[self.state] == np.nanmax(self.q_values[self.state]))
            action = random.choice(index[0])
            #print("greedy:", self.q_values[self.state], self.state, action)
            #action = np.where(self.q_values[self.state] == np.nanmax(self.q_values[self.state])) #np.argmaxで行動価値が最大の要素のindexを返却(全く同じ行動価値を持つ行動に関してはindexが前の行動が優先)

        self.previous_action = action #選択した行動を記録
        return action #その行動を返却

    def observe(self, next_state, reward=None):
        """
            次の状態と報酬の観測
        """
        next_state = str(next_state)
        if next_state not in self.q_values:  # 始めて訪れる状態であれば行動価値を全て0で埋める
            self.q_values[next_state] = np.repeat(0.0, len(self.actions))

        self.previous_state = copy.deepcopy(self.state) #現在の状態を記録
        self.state = next_state #現在の状態にnext_stateを記録

        if reward is not None: #報酬があれば実行(ほとんどの環境設定でそうで今回もrewardは必ず返却される)
            self.reward_history.append(reward) #reword_historyに記録
            self.learn(reward)

    def learn(self, reward):
        """
            Q値の更新
        """
        q = self.q_values[self.previous_state][self.previous_action]  # Q(s, a) ##Q((0, 1), 0)
        max_q = np.nanmax(self.q_values[self.state])  # maxQ(s', a') ##0.0

        # Q(s, a) ← Q(s, a) + alpha * (r + gamma * maxQ(s', a') - Q(s, a))
        self.q_values[self.previous_state][self.previous_action] = q + (self.alpha * (reward + (self.gamma * max_q) - q))
        #print("observe-learn:", q, max_q, self.q_values)

    def set_wall(self, action, state):
        state = str(state)
        #print("np.nan更新前:", self.q_values[state], state, action, self.q_values[state][action])
        self.q_values[state][action] = np.nan
        #print(self.q_values)
        #print("np.nan更新後:", self.q_values[self.state], state, action)
