import glob, re, argparse, os, shutil, subprocess
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
CURRENT_TIME = re.compile(".*Current time limit: (.*)")

LAMBDA_TOTAL_NUM_EDGES = re.compile(".*Total num edges connected (.*)")
LAMBDA_TOTAL_SAMPLES_GENERATED = re.compile(".*Total samples generated (.*)")
LAMBDA_TOTAL_VALID_SAMPLES_GENERATED = re.compile(".*Total valid samples generated (.*)")

LAMBDA_TOTAL_VERTICES_SENT = re.compile(".*Total vertices sent (.*)")
LAMBDA_TOTAL_VERTICES_RECVD = re.compile(".*Total vertices recvd (.*)")
LAMBDA_TOTAL_SAMPLES_TAKEN = re.compile(".*Total samples taken (.*)")

NEW_LAMBDA_STARTED = re.compile(".*New lambda ([0-9]*) .* duration ([0-9]*) milliseconds")
LAMBDA_COMPLETED = re.compile(".*Lambda completed ([0-9]*) .* duration ([0-9]*) milliseconds")

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument("--root", required=True)
parser.add_argument("--time_limit_experiments", default=1, type=int)
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


def truncate_files_to_last(fil, n=100000):
    dirname = os.path.dirname(fil)
    old_file_path = "{}/full_out.txt".format(dirname)
    subprocess.call("mv {} {}".format(fil, old_file_path), shell=True)
    subprocess.call("tail -n {} {} > {}".format(n, old_file_path, fil), shell=True)
    subprocess.call("rm {}".format(old_file_path), shell=True)

def truncate_all_files():
    for fil in glob.glob(args.root + "/*/*/*/out.txt"):
        truncate_files_to_last(fil)

def path_length_processing_new():
    all_data = {
            "start": [],
            "goal": [],
            "distance": [],
            "time_limit": [],
            "num_lambdas": [],
            "num_samples": [],
            "scenario": [],
            "algorithm": []
            }
    for fil in glob.glob(args.root + "/*/*/*/out.txt"):
        split_fil = fil.split("/")
        label = split_fil[-2]
        scenario = split_fil[-3]
        algorithm = split_fil[-4]
        _, num_lambdas_str, time_limit_str, _, num_samples_str = label.split("__")
        num_lambdas = int(num_lambdas_str.split("=")[1])
        num_samples = int(num_samples_str.split("=")[1])
        time_limit = float(time_limit_str.split("=")[1])
        curr_time = None
        with open(fil, 'r') as f:
            for line in f.readlines():
                time_match = CURRENT_TIME.match(line)
                if time_match is not None:
                    curr_time = float(time_match.group(1))
                    continue

                ret = STARTS_GOALS_DISTANCE_REGEX.match(line)
                if ret is not None: 
                    if curr_time is None: raise Exception("no time available!!")
                    start, goal, dist = ret.group(1), ret.group(2), float(ret.group(3))
                    all_data["start"].append(start)
                    all_data["goal"].append(goal)
                    all_data["distance"].append(dist)
                    all_data["time_limit"].append(curr_time / 1000)
                    all_data["num_lambdas"].append(num_lambdas)
                    all_data["num_samples"].append(num_samples)
                    all_data["scenario"].append(scenario)
                    all_data["algorithm"].append(algorithm)

    df = pd.DataFrame(all_data)
    import ipdb; ipdb.set_trace()
    df.to_csv('{}/path_length_data.csv'.format(args.root))

def lambda_start_end_time_processing():
    all_data = {
            "num_lambdas": [],
            "scenario": [],
            "algorithm": [],
            "lambda_start_time": [],
            "lambda_end_time": [],
            "lambda_duration": []
            }
    if args.time_limit_experiments:
        all_data["time_limit"] = []
    else:
        all_data["graph_size"] = []
    for fil in glob.glob(args.root + "*/*/*/out.txt"):
        split_fil = fil.split("/")
        label = split_fil[-2]
        scenario = split_fil[-3]
        algorithm = split_fil[-4]
        _, num_lambdas_str, time_limit_or_graph_size_str, _, num_samples_str = label.split("__")
        num_lambdas = int(num_lambdas_str.split("=")[1])
        num_samples = int(num_samples_str.split("=")[1])
        data_by_lambda = {} # lambda_id -> (start_time, end_time)

        if num_samples != 10: continue
        if args.time_limit_experiments:
            time_limit = float(time_limit_or_graph_size_str.split("=")[1])
        else:
            graph_size = int(time_limit_or_graph_size_str.split("=")[1])

        with open(fil, 'r') as f:
            for line in f.readlines():
                new_lambda_match = NEW_LAMBDA_STARTED.match(line)
                if new_lambda_match is not None:
                    lambda_id = int(new_lambda_match.group(1))
                    duration = int(new_lambda_match.group(2))
                    data_by_lambda[lambda_id] = [0, 0]
                    data_by_lambda[lambda_id][0] = duration
                    continue

                lambda_completed_match = LAMBDA_COMPLETED.match(line)
                if lambda_completed_match is not None:
                    lambda_id = int(lambda_completed_match.group(1))
                    duration = int(lambda_completed_match.group(2))
                    data_by_lambda[lambda_id][1] = duration
                    continue
        for lambda_id, (start_time, end_time) in data_by_lambda.items():
            all_data["num_lambdas"].append(num_lambdas)
            all_data["scenario"].append(scenario)
            all_data["algorithm"].append(algorithm)
            all_data["lambda_start_time"].append(start_time)
            all_data["lambda_end_time"].append(end_time)
            all_data["lambda_duration"].append(end_time - start_time)
            if args.time_limit_experiments:
                all_data["time_limit"].append(time_limit)
            else:
                all_data["graph_size"].append(graph_size)

    import ipdb; ipdb.set_trace()
    df = pd.DataFrame(all_data)
    df.to_csv('{}/lambda_start_end_times.csv'.format(args.root))

def graph_size_processing_new():
    
    all_data = {
            "num_vertices": [],
            "num_edges": [],
            "total_size": [],
            "time_limit": [],
            "num_lambdas": [],
            "num_samples": [],
            "max_global_samples": [],
            "scenario": [],
            "algorithm": []
            }
    for fil in glob.glob(args.root + "/*/*/*/out.txt"):
        split_fil = fil.split("/")
        label = split_fil[-2]
        scenario = split_fil[-3]
        algorithm = split_fil[-4]
        _, num_lambdas_str, time_limit_str, _, num_samples_str = label.split("__")
        num_lambdas = int(num_lambdas_str.split("=")[1])
        num_samples = int(num_samples_str.split("=")[1])
        time_limit = float(time_limit_str.split("=")[1])
        curr_time, curr_vertices, curr_edges, curr_global_samples = None, None, None, None
        with open(fil, 'r') as f:
            for line in f.readlines():
            
                #if "Current time" in line: import ipdb; ipdb.set_trace()
                time_match = CURRENT_TIME.match(line)
                if time_match is not None:
                    curr_time = float(time_match.group(1))
                    continue
                if curr_time is None: continue

                ret = VERTICES_REGEX.match(line)
                if ret is not None:
                    curr_vertices = int(ret.group(1))

                ret = EDGES_REGEX.match(line)
                if ret is not None:
                    curr_edges = int(ret.group(1))

                ret = MAX_GLOBAL_SAMPLES.match(line)
                if ret is not None:
                    curr_global_samples = int(ret.group(1))
                    if None in [curr_time, curr_vertices, curr_edges, curr_global_samples]: raise Exception("Should not happen")
                    all_data["num_vertices"].append(curr_vertices)
                    all_data["num_edges"].append(curr_edges)
                    all_data["total_size"].append(curr_vertices + curr_edges)
                    all_data["time_limit"].append(curr_time / 1000) # ms to s
                    all_data["scenario"].append(scenario)
                    all_data["algorithm"].append(algorithm)
                    all_data["num_lambdas"].append(num_lambdas)
                    all_data["num_samples"].append(num_samples)
                    all_data["max_global_samples"].append(curr_global_samples)

                    curr_time, curr_vertices, curr_edges, curr_global_samples = None, None, None, None

    df = pd.DataFrame(all_data)
    df.to_csv('{}/graph_size_data.csv'.format(args.root))

def common_seed_work_distribution():
    all_data = {
            "total_num_samples": [],
            "total_num_valid_samples": [],
            "total_num_edges": [],
            "num_lambdas": [],
            "algorithm": []
            }
    for fil in glob.glob(args.root + "/*/lambda-*.out"):
        split_fil = fil.split("/")
        label = split_fil[-2]
        algorithm = split_fil[-3]
        num_lambdas = int(label.split("=")[1])
        #curr_vertices, curr_edges, curr_global_samples = None, None, None
        with open(fil, 'r') as f:
            for line in f.readlines():
            
                ret = LAMBDA_TOTAL_NUM_EDGES.match(line)
                if ret is not None:
                    num_edges = int(ret.group(1))
                    continue

                ret = LAMBDA_TOTAL_SAMPLES_GENERATED.match(line)
                if ret is not None:
                    num_samples = int(ret.group(1))
                    continue

                ret = LAMBDA_TOTAL_VALID_SAMPLES_GENERATED.match(line)
                if ret is not None:
                    num_valid_samples = int(ret.group(1))
                    continue
        all_data["num_lambdas"].append(num_lambdas)
        all_data["algorithm"].append(algorithm)
        all_data["total_num_edges"].append(num_edges)
        all_data["total_num_samples"].append(num_samples)
        all_data["total_num_valid_samples"].append(num_valid_samples)

    df = pd.DataFrame(all_data)
    df.to_csv('{}/work_distribution_out.csv'.format(args.root))


def fixed_graph_work_distribution():
    all_data = {
            "total_num_samples": [],
            "num_vertices_sent": [],
            "num_vertices_recvd": [],
            "num_lambdas": [],
            "algorithm": []
            }
    for fil in glob.glob(args.root + "/*/lambda-*.out"):
        split_fil = fil.split("/")
        label = split_fil[-2]
        algorithm = split_fil[-3]
        num_lambdas = int(label.split("=")[1])
        curr_vertices, curr_edges, curr_global_samples = None, None, None
        with open(fil, 'r') as f:
            for line in f.readlines():
            
                ret = LAMBDA_TOTAL_VERTICES_SENT.match(line)
                if ret is not None:
                    num_vertices_sent = int(ret.group(1))
                    continue

                ret = LAMBDA_TOTAL_SAMPLES_TAKEN.match(line)
                if ret is not None:
                    num_samples = int(ret.group(1))
                    continue

                ret = LAMBDA_TOTAL_VERTICES_RECVD.match(line)
                if ret is not None:
                    num_vertices_recvd = int(ret.group(1))
                    continue

        all_data["num_lambdas"].append(num_lambdas)
        all_data["algorithm"].append(algorithm)
        all_data["total_num_samples"].append(num_samples)
        all_data["num_vertices_recvd"].append(num_vertices_recvd)
        all_data["num_vertices_sent"].append(num_vertices_sent)

    df = pd.DataFrame(all_data)
    df.to_csv('{}/work_distribution_out.csv'.format(args.root))

if __name__=="__main__":
    #image_processing_2d()
    #graph_size_processing()
    #path_length_processing()
    # above are old style functions, new ones below
    #truncate_all_files()

    lambda_start_end_time_processing()
    #graph_size_processing_new()
    #path_length_processing_new()

    #common_seed_work_distribution()
    #fixed_graph_work_distribution()
    #exit(0)

