import copy


class GridWorld:

    def __init__(self):

        self.filed_type = { #dict 
            "N": 0,  # 通常
            "G": 1,  # ゴール
            "W": 2,  # 壁
        #    "T": 3,  # トラップ
        }
        self.actions = { #dict
            "UP": 0,
            "DOWN": 1,
            "LEFT": 2,
            "RIGHT": 3
        }
        self.map = [[0, 0, 2, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 2, 0, 2, 0, 0, 2, 2, 0],
                    [2, 0, 2, 0, 2, 2, 2, 2, 0, 0],
                    [0, 0, 0, 0, 0, 2, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 2, 0, 0, 0, 0],
                    [2, 2, 2, 2, 0, 2, 0, 2, 0, 0],
                    [0, 0, 0, 0, 0, 2, 0, 2, 0, 0],
                    [0, 2, 2, 2, 2, 2, 0, 2, 0, 0],
                    [0, 0, 0, 0, 2, 0, 0, 2, 0, 0],
                    [0, 0, 2, 0, 0, 0, 0, 2, 0, 1]]

        self.start_pos = (0, 0)   # エージェントのスタート地点(x, y) ()なしでもタプルと判定される
        self.agent_pos = copy.deepcopy(self.start_pos)  # エージェントがいる地点

    def step(self, action): #step関数が呼ばれた時点でactinonは[UP,DOWN, LEFT, RIGHT]のいずれかを指定済み
        """
            行動の実行
            状態, 報酬、ゴールしたかを返却
        """
        to_x, to_y = copy.deepcopy(self.agent_pos) #現在座標

        # 移動可能かどうかの確認。移動不可能であれば、ポジションはそのままにマイナス報酬
        if self._is_possible_action(to_x, to_y, action) == False:
            return self.agent_pos, None, False #報酬はNone.これはmain側でNoneなら壁である判定をさせる

        if action == self.actions["UP"]:
            to_y += -1
        elif action == self.actions["DOWN"]:
            to_y += 1
        elif action == self.actions["LEFT"]:
            to_x += -1
        elif action == self.actions["RIGHT"]:
            to_x += 1

        is_goal = self._is_end_episode(to_x, to_y) # エピソードの終了の確認
        reward = self._compute_reward(to_x, to_y)
        self.agent_pos = to_x, to_y #ここで座標変更(移動)
        return self.agent_pos, reward, is_goal #返すのは現在座標、即時報酬、ゴール判定

    def _is_end_episode(self, x, y):
        """
            x, yがエピソードの終了かの判定。
        """
        #x, yはマップの左上のマスを原点として右と下がそれぞれの正の方向なので配列で扱うときはxとyは逆になる
        if self.map[y][x] == self.filed_type["G"]:      # ゴール
            return True
        #elif self.map[y][x] == self.filed_type["T"]:    # トラップ
        #    return True
        else:
            return False

    def _is_wall(self, x, y):
        """
            x, yが壁かどうかの確認
        """
        if self.map[y][x] == self.filed_type["W"]:
            return True
        else:
            return False

    def _is_possible_action(self, x, y, action):
        """
            実行可能な行動かどうかの判定
        """
        to_x = x
        to_y = y

        if action == self.actions["UP"]:
            to_y += -1
        elif action == self.actions["DOWN"]:
            to_y += 1
        elif action == self.actions["LEFT"]:
            to_x += -1
        elif action == self.actions["RIGHT"]:
            to_x += 1

        if len(self.map) <= to_y or 0 > to_y: #to_yが取りうる値は[0, 9]なのでそうでなければFalse
            return False
        elif len(self.map[0]) <= to_x or 0 > to_x: #to_xが取りうる値は[0, 9]なのでそうでなければFalse
            return False
        elif self._is_wall(to_x, to_y): #(to_x, to_y)が壁ならFalse
            return False

        return True

    def _compute_reward(self, x, y):
        if self.map[y][x] == self.filed_type["N"]:
            return 0
        elif self.map[y][x] == self.filed_type["G"]:
            return 1
#        elif self.map[y][x] == self.filed_type["T"]:
#            return -100

    def reset(self): #ほんとに単にスタートポジションに戻すだけ
        self.agent_pos = self.start_pos
        return self.start_pos
