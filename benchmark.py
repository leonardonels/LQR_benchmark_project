import subprocess
import random
import pandas as pd
import numpy as np

class Benchmark:
    algs = ("annoy", "nanoflann")
    n = 100  # number of executions
    plot = False

    sample_scaling_min = 1
    sample_scaling_max = 10
    oversample_factor = 1

    trj_min_x = 1.0
    trj_max_x = 10.0
    trj_min_y = 1.0
    trj_max_y = 10.0

    odo_min_x = -3.0
    odo_max_x = 10.0
    odo_min_y = -5.0
    odo_max_y = 5.0
    odo_min_yaw = 0.0
    odo_max_yaw = 6.28


    def sample(self):
        trajectory = pd.read_csv("trajectories/cart_race_track_trajectory.csv")
        sample_scaling_max = min(self.sample_scaling_max, trajectory.shape[0])
        trajectory = trajectory.iloc[::int(random.uniform(self.sample_scaling_min, sample_scaling_max))]
        trajectory["x"] *= random.uniform(self.trj_min_x, self.trj_max_x)
        trajectory["y"] *= random.uniform(self.trj_min_y, self.trj_max_y)
        trajectory.to_csv("trajectories/test_trajectory_downsampled.csv", index=False)

        if self.oversample_factor > 1:
            num_oversamples = int(self.oversample_factor * len(trajectory)) - len(trajectory)
            oversampled_trajectory = trajectory.sample(num_oversamples, replace=True)
            trajectory = pd.concat([trajectory, oversampled_trajectory], ignore_index=True)

    def main(self):
        execution_time_arr = np.ndarray((len(self.algs), self.n))
        try:
            for i in range(self.n):
                self.sample()
                test_odometry = pd.DataFrame({"x": random.uniform(self.odo_min_x, self.odo_max_x),
                                              "y": random.uniform(self.odo_min_y, self.odo_max_y),
                                              "yaw": random.uniform(self.odo_min_yaw, self.odo_max_yaw)
                                              }, index=[0])
                test_odometry.to_csv("odometry/test_odometry.csv", index=False)
                for j in range(len(self.algs)):
                    print(f"\n ===== {i} : {self.algs[j]} =====\n")
                    build = "build/" + self.algs[j]
                    subprocess.run([build,
                                    "odometry/test_odometry.csv",
                                    "trajectories/test_trajectory_downsampled.csv"
                                    ], check=True)
                    execution_time = pd.read_csv("utils/time.csv")
                    execution_time_arr[j][i] = execution_time.iloc[0, 0]
                    if self.plot:
                        subprocess.run(["python3",
                                        "utils/print.py",
                                        "--odometry", "odometry/test_odometry.csv",
                                        "--trajectory", "trajectories/test_trajectory_downsampled.csv",
                                        "--closest_point", "closest_point.csv"
                                        ], check=True)
            execution_time_means = execution_time_arr.mean(axis=1)
            print('\n')
            for i in range(len(self.algs)):
                print(f"[{self.algs[i]}]: {execution_time_means[i]} milliseconds to compute on average")

        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    benchmark = Benchmark()
    benchmark.main()
