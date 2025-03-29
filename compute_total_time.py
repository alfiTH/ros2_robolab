import json
import os

files = os.listdir('.')

total_time = 0
total_valid_time = 0
for f in files:
    if f.startswith('rosbag2') and f.endswith('.json'):
        with open(f, 'r') as jsonfile:
            sequence = json.load(jsonfile)
        total_time += sequence['effective_time']
        total_valid_time += sequence['effective_time']
        trajectories = sequence['trajectories']
        for t in trajectories:
            fps_360 = t['nImgs360']/t['time']
            fps_zed = t['nImgsZed']/t['time']
            if fps_360<25 or fps_zed<12:
                print('invalid sequence', f)
                total_valid_time -= t['time']

print(f'total time: {total_time} s, {total_time/60.} m')
print(f'total valid time: {total_valid_time} s, {total_valid_time/60.} m')