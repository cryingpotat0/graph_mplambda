import glob, re, argparse, os, shutil
import pandas as pd 
import numpy as np
from distutils.dir_util import copy_tree
from collections import defaultdict

STARTS_GOALS_DISTANCE_REGEX = re.compile(".*Path found for start \[(.*)\]\^T goal \[(.*)\]\^T with distance (.*)")
#EDGES_REGEX = re.compile(".*Received ([0-9]*) edges from lambda [0-9]*")
#VERTICES_REGEX = re.compile(".*Received ([0-9]*) vertices from lambda [0-9]*")
#MAX_GLOBAL_SAMPLES = re.compile(".* \(\/home\/ec2-user\/graph_mplambda\/include\/coordinator\.hpp:279\) (.*)")
EDGES_REGEX = re.compile(".*Num edges in graph ([0-9]*)")
VERTICES_REGEX = re.compile(".*Num vertices in graph ([0-9]*)")
MAX_GLOBAL_SAMPLES = re.compile(".*Final global_num_samples (.*)")
LOOP_ACTUAL_RUNTIME = re.compile(".*Loop finished in (.*) s")

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument("--root", required=True)
args = parser.parse_args()

def get_starts_goals_distances(fil):
    tracking = {}
    with open(fil, 'r') as f:
        req_lines = f.readlines()[-200:] # if nothing errors, the last few lines contain what we need
        for line in req_lines:
            ret = STARTS_GOALS_DISTANCE_REGEX.match(line)
            if ret is not None: 
                tracking[(ret.group(1), ret.group(2))] = ret.group(3)
    return tracking

def get_loop_runtime(fil):
    loop_actual_runtime = 0
    with open(fil, 'r') as f:
        req_lines = f.readlines()[-40:] 
        for line in req_lines:
            ret = LOOP_ACTUAL_RUNTIME.match(line)
            if ret is not None:
                loop_actual_runtime = float(ret.group(1))
    return loop_actual_runtime

def get_total_number_of_edges_and_vertices(fil):
    num_vertices, num_edges, max_global_samples = 0, 0, 0
    with open(fil, 'r') as f:
        for line in f.readlines()[-200:]:
            ret = VERTICES_REGEX.match(line)
            if ret is not None:
                num_vertices = int(ret.group(1))
            ret = EDGES_REGEX.match(line)
            if ret is not None:
                num_edges = int(ret.group(1))
            ret = MAX_GLOBAL_SAMPLES.match(line)
            if ret is not None:
                #import ipdb; ipdb.set_trace()
                #nums = [int(val) for val in ret.group(1).split(";")[:-1]]
                #max_global_samples = min(nums) * len(nums)
                max_global_samples = int(ret.group(1))
    return num_vertices, num_edges, max_global_samples

def graph_size_processing(reuse=False):
    if reuse and os.exists('{}/path_length_data.csv'.format(args.root)):
        df = pd.read_csv('{}/path_length_data.csv'.format(args.root))
    else:
        all_data = {
                "num_vertices": [],
                "num_edges": [],
                "total_size": [],
                "time_limit": [],
                "num_lambdas": [],
                "num_samples": [],
                "max_global_samples": [],
                "actual_time": []
                }
        #args = parser.parse_args()
        for name in glob.glob("./{}/*/out.txt".format(args.root)):
            num_vertices, num_edges, max_global_samples = get_total_number_of_edges_and_vertices(name)
            label = name.split("/")[-2]
            _, num_lambdas_str, time_limit_str, _, num_samples_str = label.split("__")
            num_lambdas = int(num_lambdas_str.split("=")[1])
            num_samples = int(num_samples_str.split("=")[1])
            time_limit = float(time_limit_str.split("=")[1])
            all_data["num_vertices"].append(num_vertices)
            all_data["num_edges"].append(num_edges)
            all_data["total_size"].append(num_vertices + num_edges)
            all_data["time_limit"].append(time_limit)
            all_data["num_lambdas"].append(num_lambdas)
            all_data["num_samples"].append(num_samples)
            all_data["max_global_samples"].append(max_global_samples)
            all_data["actual_time"].append(get_loop_runtime(name))
        df = pd.DataFrame(all_data)
        df.to_csv('{}/graph_size_data.csv'.format(args.root))
    df = df.groupby(['time_limit', 'num_lambdas', 'num_samples'], as_index=False).agg({'total_size': ['mean', 'std'], 'num_vertices': ['mean',], 'num_edges': ['mean'], 'max_global_samples': ['mean'], 'actual_time': ['min', 'max']})
    df = df[['time_limit', 'num_lambdas', 'num_samples', 'total_size', 'num_vertices', 'num_edges', 'max_global_samples', 'actual_time']]
    df.columns = ['time_limit', 'num_lambdas', 'num_samples', 'total_size_mean', 'total_size_std', 'num_vertices', 'num_edges', 'max_global_samples', 'actual_time_min', 'actual_time_max']

    df = df.sort_values(by=["time_limit", "max_global_samples", "num_lambdas"])
    print(df)
    for time_limit, time_group in df.groupby(['time_limit']):
        print("------------- time_limit {} ----------------".format(time_limit))
        #min_total_size_mean = min(group["total_size_mean"])
        printable_grp = time_group.reset_index()[["num_lambdas", "num_samples", "total_size_mean", "num_vertices", "num_edges", "max_global_samples", 'actual_time_max']]
        print(printable_grp)

def path_length_processing(reuse=False):
    if reuse and os.exists('{}/path_length_data.csv'.format(args.root)):
        df = pd.read_csv('{}/path_length_data.csv'.format(args.root))
    else:
        all_data = {
                "start": [],
                "goal": [],
                "distance": [],
                "time_limit": [],
                "num_lambdas": [],
                "num_samples": [],
                }
        #args = parser.parse_args()
        for name in glob.glob("./{}/*/out.txt".format(args.root)):
            curr_data = get_starts_goals_distances(name)
            label = name.split("/")[-2]
            _, num_lambdas_str, time_limit_str, _, num_samples_str = label.split("__")
            num_lambdas = int(num_lambdas_str.split("=")[1])
            num_samples = int(num_samples_str.split("=")[1])
            time_limit = float(time_limit_str.split("=")[1])
            for ((start, goal), distance) in curr_data.items():
                all_data["start"].append(start)
                all_data["goal"].append(goal)
                all_data["distance"].append(float(distance))
                all_data["time_limit"].append(time_limit)
                all_data["num_lambdas"].append(num_lambdas)
                all_data["num_samples"].append(num_samples)
        df = pd.DataFrame(all_data)
        df.to_csv('{}/path_length_data.csv'.format(args.root))
    df = df.groupby(['start', 'goal', 'time_limit', 'num_lambdas', 'num_samples'], as_index=False).agg({'distance': ['median', 'count']})
    df.columns = ['start', 'goal', 'time_limit', 'num_lambdas', 'num_samples', 'distance_mean', 'distance_count']

    df = df.sort_values(by=["time_limit", "start", "goal",  "distance_mean", "num_lambdas"])
    print(df)
    for time_limit, time_group in df.groupby(['time_limit']):
        print("------------- time_limit {} ----------------".format(time_limit))
        average_percentage_performance = defaultdict(list)
        for grp_name, group in time_group.groupby(['start', 'goal']):
            print("Start and goal", grp_name)
            min_distance_mean = min(group["distance_mean"])
            printable_grp = group.reset_index()[["num_lambdas", "num_samples", "distance_mean", "distance_count"]]
            print(printable_grp)
            try:
                grp_max = max([k for k in group["distance_mean"] if k < float('inf')])
            except ValueError:
                grp_max = 0
            for num_lambdas, num_samples, distance_mean, distance_count in printable_grp.values:
                d = distance_mean
                if d < float('inf'):
                    average_percentage_performance[num_lambdas].append(d)
                else:
                    average_percentage_performance[num_lambdas].append(grp_max)
        for k, v in average_percentage_performance.items():
            print("num lambdas:", k, " performance: ", len(v), np.sum(list(v)))
            average_percentage_performance[k] = np.mean(v)
        print(average_percentage_performance)

def image_processing_2d():
    for name in glob.glob("./{}/*".format(args.root)):
        if not os.path.isdir(name): continue
        #shutil.copytree("../resources", "{}/resource".format(name))
        copy_tree("../resources", "{}/resources".format(name))
        f = open("{}/index.html".format(name), 'w')
        message = """
        <html>
        <head></head>
        <body>
        """
        for img in sorted(glob.glob("{}/png_2d_demo_output-*.svg".format(name))):
            img_name = os.path.split(img)[-1]
            print(img_name)
            message += '<embed src={} />'.format(img_name)
        print("----")

        message += """
        </body></html>
        """
        f.write(message)
        f.close()



if __name__=="__main__":
    image_processing_2d()
    #graph_size_processing()
    #path_length_processing()
    #exit(0)

