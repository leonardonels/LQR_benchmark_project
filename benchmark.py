import subprocess
import random
import time
import pandas as pd

######## params ########

n=10    #number ofexecutions

########################

trajectory=pd.read_csv("trajectories/trajectory_downsampled.csv")
trajectory["x"]*=random.uniform(1.0,10.0)
trajectory["y"]*=random.uniform(1.0,10.0)
trajectory.to_csv("trajectories/test_trajectory_downsampled.csv", index=False)
try:    
    for i in range(n):
        start_time = time.time()
        test_odometry=pd.DataFrame({"x":random.uniform(-3.0,10.0),
                                    "y":random.uniform(-5.0,5.0),
                                    "yaw":random.uniform(0.0,6.28)
                                    }, index=[0])
        test_odometry.to_csv("odometry/test_odometry.csv", index=False)
        subprocess.run(["build/knn",
                        "odometry/test_odometry.csv",
                        "trajectories/test_trajectory_downsampled.csv"
                        ], check=True)
        end_time = time.time()
        execution_time = end_time - start_time
        print(f"[BENCHMARK]: {execution_time/100} milliseconds to compute")

        subprocess.run(["python3",
                        "print.py",
                        "--odometry", "odometry/test_odometry.csv",
                        "--trajectory", "trajectories/test_trajectory_downsampled.csv",
                        "--closest_point", "closest_point.csv"
                        ], check=True)

except KeyboardInterrupt:
        pass