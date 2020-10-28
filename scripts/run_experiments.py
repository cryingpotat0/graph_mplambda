import os, time, subprocess, argparse, glob
from functools import reduce
parser = argparse.ArgumentParser(description='Process some integers.')

parser.add_argument('--num_trials', type=int, required=True)
parser.add_argument('--num_samples', default="1")
parser.add_argument('--time_limits', default="")
parser.add_argument('--graph_sizes', default="")
parser.add_argument('--random_seeds', default="0")
parser.add_argument('--num_lambdas', default="4")
parser.add_argument('--algorithm', action='append', required=True)
parser.add_argument('--scenario', action='append', required=True)
parser.add_argument('--aws', dest='local', action='store_false', default=True)
parser.add_argument('--load_graph', dest='load_graph', action='store_true', default=False)

args = parser.parse_args()

if len(args.time_limits) == 0 and len(args.graph_sizes) == 0:
    raise ValueError("Time limit or graph size should be provided")

if len(args.time_limits) > 0:
    TIME_BASED = True
    args.time_limits = [float(time_limit) for time_limit in args.time_limits.split(",")]
if len(args.graph_sizes) > 0:
    TIME_BASED = False
    args.graph_sizes = [int(graph_size) for graph_size in args.graph_sizes.split(",")]
args.num_samples = [int(num_sample) for num_sample in args.num_samples.split(",")]
args.random_seeds = [int(random_seed) for random_seed in args.random_seeds.split(",")]
args.num_lambdas = [int(num_lambda) for num_lambda in args.num_lambdas.split(",")]


if len(args.time_limits) != 0 and len(args.graph_sizes) != 0:
    raise ValueError("Time limit and graph size cannot be provided")

if len(args.graph_sizes) > 0: TIME_BASED = False

def run_fetch_env_frame1(args, time_limit, graph_size, num_sample, folder_name, num_division, algorithm, random_seed):
    # env-frame: 0.48...
    starts = [[0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0]]
    goals = [
[0.283115211646117, 1.22028927373113, -0.648826405047283, -1.5365920442559, -0.0959178170482211, 2.45739839641551, -1.0035247223742, 0.796220232488176],
    ]
    command = "PI=3.141592653589793 ;PI_2=1.570796326794897; make -j8; ./mpl_coordinator --scenario fetch --goal-radius 0.01,0.01,0.01,0.01,0.01,$PI --env-frame 0.48,1.09,0.00,0,0,-$PI_2  --env resources/AUTOLAB.dae --global_min 0,-1.6056,-1.221,-$PI,-2.251,-$PI,-2.16,-$PI --global_max 0.38615,1.6056,1.518,$PI,2.251,$PI,2.16,$PI " 
    for start in starts:
        command += " --start "
        for i, s in enumerate(start):
            command += str(s)
            if i != len(start) - 1: command += ","
    for goal in goals:
        command += " --goal "
        for i, g in enumerate(goal):
            command += str(g)
            if i != len(goal) - 1: command += ","
    if args.local:
        lambda_type = "local"
        coordinator = "localhost"
    else:
        lambda_type = "aws"
        coordinator = "54.202.173.51"

    if args.load_graph:
        graph_out = folder_name + "/graph-out.txt"
        assert os.path.exists(graph_out), "Graph does not exist"
        load_graph = graph_out
    else:
        if os.path.exists(folder_name):
            print(folder_name, "already done")
            return
        os.makedirs(folder_name)
        load_graph = '""' # empty string
        
    command += " --lambda_type {lambda_type} --coordinator {coordinator} --num_samples {num_samples} --jobs {num_divisions} --algorithm {algorithm} --load_graph {load_graph}".format(num_samples=num_sample, num_divisions=num_division, lambda_type=lambda_type, coordinator=coordinator, load_graph=load_graph, algorithm="prm_common_seed",)

    if TIME_BASED:
        command += " --time-limit {}".format(time_limit)
    else:
        command += " --graph-size {}".format(graph_size)

    command += " --random_seed {}".format(random_seed)


    print(command)
    out_path = "{}/out.txt".format(folder_name)
    if args.load_graph:
        old_out_path = "{}/old_out.txt".format(folder_name)
        if os.path.exists(old_out_path):
            print("Old out already exists, not moving")
        else:
            os.rename(out_path, old_out_path)
    # latest results are always in out.txt
    #vnstat_out_path = "{}/vnstat_out.txt".format(folder_name)
    #subprocess.Popen(['vnstat',  '-tr', str(time_limit)], stdout=open(vnstat_out_path, 'w'))
    subprocess.call(command, shell=True, stderr=open(out_path, 'w')) 
    print(out_path)
    if not args.load_graph:
        if args.local:
            subprocess.call("mv lambda-* {folder_name}".format(folder_name=folder_name), shell=True)
        #subprocess.call("mv graph-out.txt {folder_name}".format(folder_name=folder_name), shell=True)

def run_fetch_env_frame2(args, time_limit, graph_size, num_sample, folder_name, num_division, algorithm, random_seed):
    # env-frame: 0.48...
    starts = [[0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0]]
    goals = [
[0.283115211646117, 1.22028927373113, -0.648826405047283, -1.5365920442559, -0.0959178170482211, 2.45739839641551, -1.0035247223742, 0.796220232488176],
    ]
    command = "PI=3.141592653589793 ;PI_2=1.570796326794897; make -j8; ./mpl_coordinator --scenario fetch --goal-radius 0.01,0.01,0.01,0.01,0.01,$PI --env-frame 0.57,-0.90,0.00,0,0,-$PI_2 --env resources/AUTOLAB.dae --global_min 0,-1.6056,-1.221,-$PI,-2.251,-$PI,-2.16,-$PI --global_max 0.38615,1.6056,1.518,$PI,2.251,$PI,2.16,$PI " 
    for start in starts:
        command += " --start "
        for i, s in enumerate(start):
            command += str(s)
            if i != len(start) - 1: command += ","
    for goal in goals:
        command += " --goal "
        for i, g in enumerate(goal):
            command += str(g)
            if i != len(goal) - 1: command += ","
    if args.local:
        lambda_type = "local"
        coordinator = "localhost"
    else:
        lambda_type = "aws"
        coordinator = "54.202.173.51"

    if args.load_graph:
        graph_out = folder_name + "/graph-out.txt"
        assert os.path.exists(graph_out), "Graph does not exist"
        load_graph = graph_out
    else:
        if os.path.exists(folder_name):
            print(folder_name, "already done")
            return
        os.makedirs(folder_name)
        load_graph = '""' # empty string
        
    command += " --lambda_type {lambda_type} --coordinator {coordinator} --num_samples {num_samples} --jobs {num_divisions} --algorithm {algorithm} --load_graph {load_graph}".format(num_samples=num_sample, num_divisions=num_division, lambda_type=lambda_type, coordinator=coordinator, load_graph=load_graph, algorithm="prm_common_seed",)

    if TIME_BASED:
        command += " --time-limit {}".format(time_limit)
    else:
        command += " --graph-size {}".format(graph_size)

    command += " --random_seed {}".format(random_seed)


    print(command)
    out_path = "{}/out.txt".format(folder_name)
    if args.load_graph:
        old_out_path = "{}/old_out.txt".format(folder_name)
        if os.path.exists(old_out_path):
            print("Old out already exists, not moving")
        else:
            os.rename(out_path, old_out_path)
    # latest results are always in out.txt
    #vnstat_out_path = "{}/vnstat_out.txt".format(folder_name)
    #subprocess.Popen(['vnstat',  '-tr', str(time_limit)], stdout=open(vnstat_out_path, 'w'))
    subprocess.call(command, shell=True, stderr=open(out_path, 'w')) 
    print(out_path)
    if not args.load_graph:
        if args.local:
            subprocess.call("mv lambda-* {folder_name}".format(folder_name=folder_name), shell=True)
        #subprocess.call("mv graph-out.txt {folder_name}".format(folder_name=folder_name), shell=True)

def run_png(args, time_limit, graph_size, num_sample, folder_name, num_division, algorithm):

    command = "make -j8; ./mpl_coordinator --scenario png --global_min 0,0 --global_max 1403,1404 --env resources/house_layout.png --start 1301,332 --goal 600,1034 --goal 275,87 --goal 789,709 --goal 533,822 --goal 357,142 --goal 884,1173 --goal 446,898 --goal 82,478 --goal 952,339 --goal 6,394"
    if args.local:
        lambda_type = "local"
        coordinator = "localhost"
    else:
        lambda_type = "aws"
        coordinator = "54.188.233.199"

    if args.load_graph:
        graph_out = folder_name + "/graph-out.txt"
        assert os.path.exists(graph_out), "Graph does not exist"
        load_graph = graph_out
    else:
        if os.path.exists(folder_name):
            print(folder_name, "already done")
            return
        os.makedirs(folder_name)
        load_graph = '""' # empty string
        

    if algorithm == "prm_fixed_graph":
        command += " --lambda_type {lambda_type} --coordinator {coordinator} --num_samples {num_samples} --num_divisions {num_divisions} --time-limit {time_limit} --algorithm {algorithm} --load_graph {load_graph}".format(num_samples=num_sample, num_divisions=num_division, time_limit=time_limit, lambda_type=lambda_type, coordinator=coordinator, load_graph=load_graph, algorithm=algorithm, )
    else:
        command += " --lambda_type {lambda_type} --coordinator {coordinator} --num_samples {num_samples} --jobs {num_divisions} --time-limit {time_limit} --algorithm {algorithm} --load_graph {load_graph}".format(num_samples=num_sample, num_divisions=num_division, time_limit=time_limit, lambda_type=lambda_type, coordinator=coordinator, load_graph=load_graph, algorithm=algorithm,)

    if TIME_BASED:
        command += " --time-limit {}".format(time_limit)
    else:
        command += " --graph-size {}".format(graph_size)

    print(command)
    out_path = "{}/out.txt".format(folder_name)
    if args.load_graph:
        old_out_path = "{}/old_out.txt".format(folder_name)
        if os.path.exists(old_out_path):
            print("Old out already exists, not moving")
        else:
            os.rename(out_path, old_out_path)
    # latest results are always in out.txt
    subprocess.call(command, shell=True, stderr=open(out_path, 'w')) 
    print(out_path)
    if not args.load_graph:
        if args.local:
            subprocess.call("mv lambda-* {folder_name}; ".format(folder_name=folder_name), shell=True)
        #subprocess.call("mv graph-out.txt {folder_name};".format(folder_name=folder_name), shell=True)
    subprocess.call("mv png_* {folder_name};".format(folder_name=folder_name), shell=True)

def run_se3(args, time_limit, graph_size, num_sample, folder_name, num_division, algorithm, random_seed, se3_data):
    # env-frame: 0.48...
    min_val, max_val, env_val, robot_val, starts, goals = se3_data["min"], se3_data["max"], se3_data["env"], se3_data["robot"], se3_data["starts"], se3_data["goals"]
    min_val = ",".join([str(val) for val in min_val])
    max_val = ",".join([str(val) for val in max_val])
    command = "PI=3.141592653589793 ;PI_2=1.570796326794897; make -j8; ./mpl_coordinator --scenario se3 --env {env_val} --robot {robot_val} --global_min {min_val} --global_max {max_val} ".format(env_val=env_val, robot_val=robot_val, min_val=min_val, max_val=max_val)
    for start in starts:
        command += " --start "
        for i, s in enumerate(start):
            command += str(s)
            if i != len(start) - 1: command += ","
    for goal in goals:
        command += " --goal "
        for i, g in enumerate(goal):
            command += str(g)
            if i != len(goal) - 1: command += ","
    if args.local:
        lambda_type = "local"
        coordinator = "localhost"
    else:
        lambda_type = "aws"
        coordinator = "54.202.173.51"

    if args.load_graph:
        graph_out = folder_name + "/graph-out.txt"
        assert os.path.exists(graph_out), "Graph does not exist"
        load_graph = graph_out
    else:
        if os.path.exists(folder_name):
            print(folder_name, "already done")
            return
        os.makedirs(folder_name)
        load_graph = '""' # empty string
        
    #if algorithm == "prm_fixed_graph":
    #    command += " --lambda_type {lambda_type} --coordinator {coordinator} --num_samples {num_samples} --num_divisions {num_divisions} --algorithm {algorithm} --load_graph {load_graph}".format(num_samples=num_sample, num_divisions=num_division, lambda_type=lambda_type, coordinator=coordinator, load_graph=load_graph, algorithm=algorithm, )
    #else:
    command += " --lambda_type {lambda_type} --coordinator {coordinator} --num_samples {num_samples} --jobs {num_divisions} --algorithm {algorithm} --load_graph {load_graph}".format(num_samples=num_sample, num_divisions=num_division, lambda_type=lambda_type, coordinator=coordinator, load_graph=load_graph, algorithm="prm_common_seed",)

    if TIME_BASED:
        command += " --time-limit {}".format(time_limit)
    else:
        command += " --graph-size {}".format(graph_size)

    command += " --random_seed {}".format(random_seed)


    print(command)
    out_path = "{}/out.txt".format(folder_name)
    if args.load_graph:
        old_out_path = "{}/old_out.txt".format(folder_name)
        if os.path.exists(old_out_path):
            print("Old out already exists, not moving")
        else:
            os.rename(out_path, old_out_path)
    # latest results are always in out.txt
    #vnstat_out_path = "{}/vnstat_out.txt".format(folder_name)
    #subprocess.Popen(['vnstat',  '-tr', str(time_limit)], stdout=open(vnstat_out_path, 'w'))
    subprocess.call(command, shell=True, stderr=open(out_path, 'w')) 
    print(out_path)
    if not args.load_graph:
        if args.local:
            subprocess.call("mv lambda-* {folder_name}".format(folder_name=folder_name), shell=True)
        #subprocess.call("mv graph-out.txt {folder_name}".format(folder_name=folder_name), shell=True)

if __name__ == "__main__":
    #time_limits = [5, 10, 20] #, 2, 3]
    #time_limits = [1, 5, 10, 20, 30] #[0.2, 0.5, ]#1]
    #num_samples = [1, ] #10, 20, 40]
    #num_trials = 30
    for algorithm in args.algorithm:
        for scenario in args.scenario:
            if scenario == "png":
                num_divisions = [
                    (0,0),
                    (1,1),
                    (2,2),
                ]
            else:
                num_divisions = [
                    #(0,0,0,0,0,0,0,0),  #1
                    #(1,0,0,0,0,0,0,0),  #2
                    #(1,1,0,0,0,0,0,0),  #4
                    #(2,1,0,0,0,0,0,0),  #6
                    #(1,1,1,0,0,0,0,0),  #8
                    #(1,1,1,1,0,0,0,0),  #16
                    #(1,1,1,1,1,0,0,0),  #32
                    #(1,1,1,1,1,1,0,0),  #64
                    (1,1,1,1,1,1,1,0),  #128

                ]
            root = "outputs/{}/{}".format(algorithm, scenario)
            if not os.path.exists(root):
                os.makedirs(root)
            if TIME_BASED:
                graph_size = 0 # placeholder
                for time_limit in args.time_limits:
                    for num_sample in args.num_samples:
                        for trial in range(args.num_trials):
                            for num_division_tup in num_divisions:
                                num_lambda = reduce(lambda a,b: (a) * (b), [val + 1 for val in num_division_tup])
                                num_division = reduce(lambda a,b: "{},{}".format(a,b), num_division_tup)
                                file_name = "{root}/num_divisions={num_divisions}__num_lambda={num_lambda}__time_limit={time_limit}__trial={trial}__num_samples={num_samples}".format(num_divisions=num_division, trial=trial, time_limit=time_limit, num_lambda=num_lambda, num_samples=num_sample, root=root)
                                #if os.path.exists(file_name): continue
                                if algorithm == "prm_fixed_graph":
                                    num_division_val = num_division
                                else:
                                    num_division_val = num_lambda
                                if scenario == "fetch1":
                                    run_fetch_env_frame1(args, time_limit, graph_size, num_sample, file_name, num_division_val, algorithm)
                                if scenario == "fetch2":
                                    run_fetch_env_frame2(args, time_limit, graph_size, num_sample, file_name, num_division_val, algorithm)
                                elif scenario == "png":
                                    run_png(args, time_limit, graph_size, num_sample, file_name, num_division_val, algorithm)

                                print("-------------------------------------------------------------------------------------")
                                time.sleep(2) 
            else:
                time_limit = 0 # placeholder
                for graph_size in args.graph_sizes:
                    for random_seed in args.random_seeds:
                        for num_sample in args.num_samples:
                            for trial in range(args.num_trials):
                                for num_lambda in args.num_lambdas:
                                    #num_lambda = reduce(lambda a,b: (a) * (b), [val + 1 for val in num_division_tup])
                                    #num_division = reduce(lambda a,b: "{},{}".format(a,b), num_division_tup)
                                    file_name = "{root}/num_divisions={num_divisions}__num_lambda={num_lambda}__graph_size={graph_size}__trial={trial}__num_samples={num_samples}__random_seed={random_seed}".format(num_divisions="NA", trial=trial, graph_size=graph_size, num_lambda=num_lambda, num_samples=num_sample, root=root, random_seed=random_seed)
                                    #if os.path.exists(file_name): continue
                                    #if algorithm == "prm_fixed_graph":
                                    #    num_division_val = num_division
                                    #else:
                                    #    num_division_val = num_lambda
                                    if scenario == "fetch1":
                                        run_fetch_env_frame1(args, time_limit, graph_size, num_sample, file_name, num_lambda, algorithm, random_seed)
                                    if scenario == "fetch2":
                                        run_fetch_env_frame2(args, time_limit, graph_size, num_sample, file_name, num_lambda, algorithm, random_seed)
                                    elif scenario == "png":
                                        run_png(args, time_limit, graph_size, num_sample, file_name, num_division_val, algorithm, random_seed)
                                    elif scenario == "se3_twistycool":
                                        se3_data = {
                                                "min": [53.46, -21.25, -476.86],
                                                "max": [402.96,269.25,-91.0],
                                                "env": "resources/Twistycool_env.dae",
                                                "robot": "resources/Twistycool_robot.dae",
                                                "starts": [[0,1,0,0,270,160,-200]],
                                                "goals": [[0,1,0,0,270,160,-400]]
                                                }
                                        run_se3(args, time_limit, graph_size, num_sample, file_name, num_lambda, algorithm, random_seed, se3_data)
                                    elif scenario == "se3_apartment":
                                        se3_data = {
                                                "min": [-73.76, -179.59, -0.03],
                                                "max": [295.77,168.26,90.39],
                                                "env": "resources/Apartment_env.dae",
                                                "robot": "resources/Apartment_robot.dae",
                                                "starts": [[0,0,0,-1,241.81,106.15,36.46]],
                                                "goals": [[0,0,0,-1,241.81,106.15,36.46]]
                                                }
                                        run_se3(args, time_limit, graph_size, num_sample, file_name, num_lambda, algorithm, random_seed, se3_data)
                                    elif scenario == "se3_cubicles":
                                        se3_data = {
                                                "min": [-508.88,-230.13,-123.75],
                                                "max": [319.62,531.87,101.0],
                                                "env": "resources/cubicles_env.dae",
                                                "robot": "resources/cubicles_robot.dae",
                                                "starts": [[0,1,0,0,-4.96,-40.62,70.57]],
                                                "goals": [[0,1,0,0,200.0,-40.62,70.57]]
                                                }
                                        run_se3(args, time_limit, graph_size, num_sample, file_name, num_lambda, algorithm, random_seed, se3_data)

                                    print("-------------------------------------------------------------------------------------")
                                    time.sleep(1) 
