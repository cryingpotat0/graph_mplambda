import glob, re, argparse
import pandas as pd
import numpy as np
from collections import defaultdict

STARTS_GOALS_DISTANCE_REGEX = re.compile(".*Path found for start \[(.*)\]\^T goal \[(.*)\]\^T with distance (.*)")

#parser = argparse.ArgumentParser(description='Process some integers.')
#parser.add_argument("--root")

def get_starts_goals_distances(fil):
    tracking = {}
    with open(fil, 'r') as f:
        req_lines = f.readlines()[-40:] # if nothing errors, the last 10 lines contain what we need
        for line in req_lines:
            ret = STARTS_GOALS_DISTANCE_REGEX.match(line)
            if ret is not None: 
                tracking[(ret.group(1), ret.group(2))] = ret.group(3)
    return tracking

if __name__=="__main__":

    all_data = {
            "start": [],
            "goal": [],
            "distance": [],
            "time_limit": [],
            "num_lambdas": [],
            "num_samples": [],
            }
    #args = parser.parse_args()
    for name in glob.glob("./outputs/*.txt"):
        curr_data = get_starts_goals_distances(name)
        label = name.split("/")[-1][:-4]
        _, num_lambdas_str, time_limit_str, _, num_samples_str = label.split("__")
        num_lambdas = int(num_lambdas_str.split("=")[1])
        num_samples = int(num_samples_str.split("=")[1])
        time_limit = int(time_limit_str.split("=")[1])
        for ((start, goal), distance) in curr_data.items():
            all_data["start"].append(start)
            all_data["goal"].append(goal)
            all_data["distance"].append(float(distance))
            all_data["time_limit"].append(time_limit)
            all_data["num_lambdas"].append(num_lambdas)
            all_data["num_samples"].append(num_samples)
    df = pd.DataFrame(all_data)
    df = df.groupby(['start', 'goal', 'time_limit', 'num_lambdas', 'num_samples'], as_index=False).agg({'distance': ['mean', 'std']})
    df.columns = ['start', 'goal', 'time_limit', 'num_lambdas', 'num_samples', 'distance_mean', 'distance_std']

    df = df.sort_values(by=["time_limit", "start", "goal",  "distance_mean", "num_lambdas"])
    print(df)
    for time_limit, time_group in df.groupby(['time_limit']):
        print("------------- time_limit {} ----------------".format(time_limit))
        average_percentage_performance = defaultdict(list)
        for grp_name, group in time_group.groupby(['start', 'goal']):
            print("Start and goal", grp_name)
            min_distance_mean = min(group["distance_mean"])
            printable_grp = group.reset_index()[["num_lambdas", "num_samples", "distance_mean"]]
            print(printable_grp)
            for num_lambdas, num_samples, distance_mean in printable_grp.values:
                d = distance_mean
                if d < float('inf'):
                    average_percentage_performance[num_lambdas].append(d)
        for k, v in average_percentage_performance.items():
            print("num lambdas:", k, " performance: ", len(v), np.sum(list(v)))
            average_percentage_performance[k] = np.mean(v)
    #print(average_percentage_performance)
