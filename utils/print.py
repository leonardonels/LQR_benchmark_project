import pandas as pd
import matplotlib.pyplot as plt
import warnings
import math
import argparse

from matplotlib import use
use('TkAgg')
warnings.filterwarnings("ignore")

def main():
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--odometry', type=str, 
                           default='odometry/odometry.csv',
                           help='Path to dometry.csv')
        parser.add_argument('--trajectory', type=str, 
                           default='trajectories/trajectory_downsampled.csv',
                           help='Path to trajectory.csv')
        parser.add_argument('--closest_point', type=str, 
                           default='utils/closest_point.csv',
                           help='Path to closest_point.csv')
        
        args = parser.parse_args()
        
        #read trajectory from csv file
        odometry = pd.read_csv(args.odometry)
        
        trajectory = pd.read_csv(args.trajectory)
        
        closest_point = pd.read_csv(args.closest_point)
        
        odom_x = odometry['x'].iloc[0]
        odom_y = odometry['y'].iloc[0]
        odom_yaw = odometry['yaw'].iloc[0] # expressed in radians
        
        cp_x = closest_point['x'].iloc[0]
        cp_y = closest_point['y'].iloc[0]
        cp_tangent = closest_point['tangent'].iloc[0]
        
        radius = math.sqrt((odom_x-cp_x)**2 + (odom_y-cp_y)**2)
        
        circle = plt.Circle((odometry['x'], odometry['y']), radius=radius, edgecolor='red', facecolor='none', linewidth=2)
        
        fix, ax = plt.subplots()
        
        textstr = f'Radius: {radius:.5f}'
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        ax.text(1.05, 0.5, textstr, transform=ax.transAxes, fontsize=12,
                verticalalignment='center', bbox=props)
        
        ax.add_patch(circle)
        ax.set_aspect('equal')
        ax.legend()
        
        #plot trajectory
        plt.arrow(odom_x, odom_y, radius*math.cos(odom_yaw), radius*math.sin(odom_yaw), head_width=0.01, head_length=0.03, fc='r', ec='r')
        plt.arrow(cp_x, cp_y, radius*math.cos(cp_tangent), radius*math.sin(cp_tangent), head_width=0.01, head_length=0.03, fc='r', ec='r')
        
        plt.scatter(trajectory['x'], trajectory['y'], marker='o', color='blue')
        plt.plot(odometry['x'], odometry['y'], 'ro')
        plt.plot(closest_point['x'], closest_point['y'], 'ro')
        plt.show()

    except KeyboardInterrupt:
        plt.close('all')
        return

if __name__ == "__main__":
    main()