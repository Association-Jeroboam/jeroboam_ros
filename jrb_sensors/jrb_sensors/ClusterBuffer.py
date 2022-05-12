from collections import deque


class ClusterBuffer:
    def __init__(self, maxlen, clusterlen):
        self.clusterlen = clusterlen
        self.maxlen = maxlen
        self._data = deque(maxlen=maxlen)
        self._valid = 0

    @property
    def is_full(self):
        return len(self._data) == self.maxlen

    @property
    def is_valid(self):
        return self._valid >= self.clusterlen

    def add_pose(self, pose, valid):
        if self.is_full:
            oldestPoseValid, _ = self._data.pop()

            if oldestPoseValid:
                self._valid -= 1

        self._data.appendleft((valid, pose))

        if valid:
            self._valid += 1

    def get_cluster_pose(self):
        if not self.is_valid:
            return None

        cluster_x, cluster_y, cluster_yaw = 0.0, 0.0, 0.0

        for valid, pose in self._data:
            if not valid:
                continue

            x, y, yaw = pose
            cluster_x += x
            cluster_y += y
            cluster_yaw += yaw

        cluster_x /= self._valid
        cluster_y /= self._valid
        cluster_yaw /= self._valid

        self._data.clear()
        self._valid = 0

        return [cluster_x, cluster_y, cluster_yaw]
