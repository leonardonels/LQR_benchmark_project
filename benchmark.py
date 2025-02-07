import subprocess
import random
import pandas as pd
import numpy as np

################ params ################

n=10    # number ofexecutions
algs = ("annoy", "nanoflann")
sample_scaling_min=1
sample_scaling_max=10
trj_min_x=1.0
trj_max_x=10.0
trj_min_y=1.0
trj_max_y=10.0
odo_min_x=-3.0
odo_max_x=10.0
odo_min_y=-5.0
odo_max_y=5.0
odo_min_yaw=0.0
odo_max_yaw=6.28
plot=True

########################################

def sample(sample_scaling_min, sample_scaling_max, trj_min_x,trj_max_x, trj_min_y,trj_max_y):
    trajectory=pd.read_csv("trajectories/cart_race_track_trajectory.csv")
    sample_scaling_max=(min(sample_scaling_max, trajectory.shape[0]))
    #trajectory=trajectory.sample(int(random.uniform(sample_scaling_min, sample_scaling_max)), axis=0)
    trajectory=trajectory.iloc[::int(random.uniform(sample_scaling_min, sample_scaling_max))]
    trajectory["x"]*=random.uniform(trj_min_x,trj_max_x)
    trajectory["y"]*=random.uniform(trj_min_y,trj_max_y)
    trajectory.to_csv("trajectories/test_trajectory_downsampled.csv", index=False)

execution_time_arr=np.ndarray((len(algs),n))
try:
    for i in range(n):
        sample(sample_scaling_min, sample_scaling_max, trj_min_x,trj_max_x, trj_min_y,trj_max_y)
        test_odometry=pd.DataFrame({"x":random.uniform(odo_min_x,odo_max_x),
                                    "y":random.uniform(odo_min_y,odo_max_y),
                                    "yaw":random.uniform(odo_min_yaw,odo_max_yaw)
                                    }, index=[0])
        for j in range(len(algs)):
            print(f"\n ===== {i} : {algs[j]} =====\n")
            build="build/" + algs[j]
            test_odometry.to_csv("odometry/test_odometry.csv", index=False)
            subprocess.run([build,
                            "odometry/test_odometry.csv",
                            "trajectories/test_trajectory_downsampled.csv"
                            ], check=True)
            execution_time=pd.read_csv("utils/time.csv")
            execution_time_arr[j][i]=execution_time.iloc[0]
            if plot:
                 subprocess.run(["python3",
                                "utils/print.py",
                                "--odometry", "odometry/test_odometry.csv",
                                "--trajectory", "trajectories/test_trajectory_downsampled.csv",
                                "--closest_point", "closest_point.csv"
                                ], check=True)
    execution_time_means=execution_time_arr.mean(axis=1)
    print('\n')
    for i in range(len(algs)):
        print(f"[{algs[i]}]: {execution_time_means[i]} milliseconds to compute on average")

except KeyboardInterrupt:
        pass