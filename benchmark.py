import subprocess
import random
import pandas as pd
import numpy as np

################ params ################

n=10    #number ofexecutions
build = "build/knn"
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

execution_time_n=np.ndarray(n)
trajectory=pd.read_csv("trajectories/trajectory_downsampled.csv")
trajectory["x"]*=random.uniform(trj_min_x,trj_max_x)
trajectory["y"]*=random.uniform(trj_min_y,trj_max_y)
trajectory.to_csv("trajectories/test_trajectory_downsampled.csv", index=False)
try:    
    for i in range(n):
        test_odometry=pd.DataFrame({"x":random.uniform(odo_min_x,odo_max_x),
                                    "y":random.uniform(odo_min_y,odo_max_y),
                                    "yaw":random.uniform(odo_min_yaw,odo_max_yaw)
                                    }, index=[0])
        test_odometry.to_csv("odometry/test_odometry.csv", index=False)
        subprocess.run([build,
                        "odometry/test_odometry.csv",
                        "trajectories/test_trajectory_downsampled.csv"
                        ], check=True)
        execution_time=pd.read_csv("utils/time.csv")
        execution_time_n.put(i,execution_time.iloc[0])
        print(f"[BENCHMARK]: {execution_time_n[i]} milliseconds to compute on test {i}")

        if plot:
             subprocess.run(["python3",
                            "utils/print.py",
                            "--odometry", "odometry/test_odometry.csv",
                            "--trajectory", "trajectories/test_trajectory_downsampled.csv",
                            "--closest_point", "closest_point.csv"
                            ], check=True)
    print(f"[BENCHMARK]: {execution_time_n.mean()} milliseconds to compute on average")

except KeyboardInterrupt:
        pass