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
#[0.0983504811945285, 0.371139833595756, -0.098912505058359, 2.16016491628702, 0.0332419251933787, -0.606802915036735, 1.43598007985539, -0.746039432665265],        
#[0.280426294579452, 0.207870102537663, -0.677022557840152, -2.66507328254128, -1.19726314893229, 1.71050076396794, -0.938068852281796, 2.05368158053659],           
#[0.346844999377988, 1.00893818682988, 0.0599395667351088, -2.46966765846436, -0.159741545640778, 2.48705054468978, -0.597827270961737, -0.381170238660199],         
#[0.268567475991556, 0.481422992527075, -0.110968939511113, -1.59728728636574, -0.963345108461369, -1.85624618847597, 1.43806974035516, 1.87702781505313],           
#[0.198391724087267, 0.934713221977167, -0.669023678294034, 0.560055774770338, -0.489143598755469, 0.902040471447032, -0.692149960863958, 1.24623166112778],         
#[0.346828384868573, 0.73688296660926, -0.739473672597282, -3.02962496997701, 0.517640927623853, 1.45907353335973, 1.94413824408156, -2.5804044054128],              
#[0.338340882170138, 0.818273454043368, -0.340390809642314, 2.5036897802403, 0.529462851493236, 2.12525723496108, 0.226611950896739, 2.4253841778172],               
#[0.168071647456087, 0.959626299992165, -0.279713632861953, 2.46855439089457, -0.700452903422931, 1.22711586227717, -0.173314234455115, -0.406685006887569],         
#[0.1568436035113, 1.19895756659759, 0.829697785636604, -2.84930431055483, 0.674602368679606, 2.11008799493205, 0.0752815010074581, -1.88460453948176],              
#[0.245063711352205, 0.614608670277712, -0.0774145048261063, -1.69961050981998, -0.931207276753278, -0.745572730101257, 1.24719851712327, -2.69715544388542],        
#[0.178941730916787, -0.176161564615621, -0.716932322133654, -1.39046293808036, -0.398077880449301, 0.339342425352945, 0.652683160311948, -3.11471054816451],        
#[0.316844665731168, 1.08097833071457, 0.559027317301401, -2.29452110740476, 0.0142555452253905, -0.522149377045005, 0.868643254214866, 0.934653306532621],          
#[0.326407677899494, 0.807757894380973, 0.148886555291553, -2.7529547272089, 0.718751545933813, 2.92412370735398, 0.304993852735226, 2.46784955924214],              
#[0.331247195787754, 0.695060725291606, -0.837133093869496, 2.73820490001116, -0.45795464073493, -1.9724500799039, 0.281586590730573, 1.31844301326072],             
#[0.247067499811424, 1.04295234039465, -0.103290005769258, -1.80254958441498, 1.15598862033335, -2.63439202268348, 1.71765185400765, -1.65365030748743],             
#[0.2848815269413, -0.851525964469983, -0.8511324222155, -2.37654269352486, -0.591030001482461, 1.59455712280699, -1.21523109493473, 2.40643149265041],              
#[0.310038683500445, -0.633862255804689, -0.716092648803919, 2.8529453939486, 1.35667640071255, -2.14146453801434, 1.05734037004526, 2.59163627132793],              
#[0.131680258892035, 0.812839207815541, 0.255875200142494, 0.707339443758727, -0.616171927193411, 2.18901012644891, -0.00686250183994463, -1.25893194838955],        
#[0.299465669410204, 0.952815275345327, -0.150500899462238, -1.64080642227197, 0.574896105114106, -2.95180402788792, 0.298739644868461, 0.737795293123225],          
        
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
        subprocess.call("mv graph-out.txt {folder_name}".format(folder_name=folder_name), shell=True)


def run_fetch_env_frame2(args, time_limit, graph_size, num_sample, folder_name, num_division, algorithm):
    # env-frame: 0.57...
    starts = [[0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0]]
    goals = [
[0.367175004808194, -0.336817492104651, -0.391634274165568, 0.708501920235296, 0.507737871694537, 3.13649185511538, 1.51062763422595, 1.33367302580728],
[0.276496847999366, 0.844671761511253, 0.152920455394991, 2.82601771366396, 1.82692320679307, 3.11597172036452, 1.19071222150992, -0.724825716003941],
[0.1067435694241, 0.790796820790578, -0.8245256549624, -1.03329387802568, 0.814517314999501, 2.88234916769481, 1.20970035665946, -1.44285808231875],
[0.0853221349994171, 0.490202426161743, 0.457938828766687, -1.14907174981083, -1.38065861128726, -0.446978430361054, 0.424792932994916, 3.00604786892286],
[0.274730267201831, -0.062634843641159, -0.165425037117345, 0.107608374394809, -0.514228147448774, -1.10401514844328, -1.21080921545689, 2.14816411094728],
[0.250301577623014, 1.13193705203315, 0.278851915772635, -2.64096004563241, 0.498370689921704, -3.02025841803874, 0.704394342291386, 1.95103421146443],
[0.207929696276963, -0.312640719182955, -0.772738455400442, 0.14564065964697, 0.376265234877095, -1.56620105610461, 0.789736748959811, 2.573240747629],
[0.298990185588533, 0.457198604518657, -0.362495277875745, -2.36469645553211, 1.22139204114733, 1.30469204989268, -0.790568066564561, 1.99500284778213],
[0.155735694261899, 1.07585003663557, -0.675527457838972, 0.199814866422495, 0.0931873976804511, 0.200442369104002, 1.60431054038815, 2.86004617199485],
[0.334013769643122, -0.239107232935291, -0.78048437609489, -3.08083882145058, 0.0350732354092536, -2.90072090996061, -0.233109440016336, 0.543661718533654],
[0.10504792206997, 1.2674176450021, 0.135131138500237, 1.75148513350984, -0.759590015489867, -0.535377542706939, 1.34590963098267, -0.526548318442299],
[0.302353854949647, 0.514387837447625, 0.242728904354575, 0.490615843598667, -0.520051146980405, 0.875284828245783, -0.0577218168093498, 1.38116666368659],
[0.173040760158342, -0.312990227866573, -0.613565311604815, -1.51892157390795, -0.0911067860018564, 1.96038575682262, 0.300978465902724, -0.300089196390469],
[0.169149123062161, 0.511516881437284, -0.366659327991421, -1.28441342574603, -1.0144015313669, -0.867548719061741, -0.126810817292239, -0.629238207197338],
[0.108460313241588, 1.28999532853229, -0.681472332982484, -1.18088927966868, 1.21915451024425, -2.6187457768506, 0.909579526401965, -2.40560742567189],
[0.328657503886421, 0.289346280560791, 0.615624045015143, 3.04169854543036, 1.23107120164063, -2.92227769968274, 0.32264128340302, -1.51420818353399],
[0.0992772070319301, 0.44690894176287, -0.285368121150095, -0.291269020933125, -0.0261740486062703, 3.00627209805372, -0.440135710786528, -2.64664245555594],
[0.290794844911196, -0.959049789166593, -0.821488683278577, -2.65240529400079, -0.145035933887483, -0.146772749516904, 0.438057505126566, 1.55685912452086],
[0.199206333913144, -0.0223101575865798, -0.707994998167683, 0.0760651695164518, 0.0364337872107372, 2.12345765162353, 1.27190619869257, 2.77757743458114],
[0.317355516938745, 1.07260992326052, -0.00253143744313378, -0.509784084501791, 0.281259967378397, 0.462047833181495, -2.14534727655838, -1.25169724111757],
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
        

    #if algorithm == "prm_fixed_graph":
    #    command += " --lambda_type {lambda_type} --coordinator {coordinator} --num_samples {num_samples} --num_divisions {num_divisions} --algorithm {algorithm} --load_graph {load_graph}".format(num_samples=num_sample, num_divisions=num_division, lambda_type=lambda_type, coordinator=coordinator, load_graph=load_graph, algorithm=algorithm, )
    #else:
    command += " --lambda_type {lambda_type} --coordinator {coordinator} --num_samples {num_samples} --jobs {num_divisions} --algorithm {algorithm} --load_graph {load_graph}".format(num_samples=num_sample, num_divisions=num_division, lambda_type=lambda_type, coordinator=coordinator, load_graph=load_graph, algorithm="prm_common_seed",)

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
            subprocess.call("mv lambda-* {folder_name}".format(folder_name=folder_name), shell=True)
        subprocess.call("mv graph-out.txt {folder_name}".format(folder_name=folder_name), shell=True)

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
        subprocess.call("mv graph-out.txt {folder_name};".format(folder_name=folder_name), shell=True)
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
        subprocess.call("mv graph-out.txt {folder_name}".format(folder_name=folder_name), shell=True)

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

                                    print("-------------------------------------------------------------------------------------")
                                    time.sleep(2) 
