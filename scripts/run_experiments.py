import os, time, subprocess, argparse, glob
from functools import reduce
parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--num_trials', type=int, required=True)
parser.add_argument('--num_samples', default="1")
parser.add_argument('--time_limits', default="1,5,10")
parser.add_argument('--scenario', action='append', required=True)
parser.add_argument('--local', type=bool, default=True)
parser.add_argument('--load_graph', dest='load_graph', action='store_true', default=False)

args = parser.parse_args()
args.time_limits = [float(time_limit) for time_limit in args.time_limits.split(",")]
args.num_samples = [int(num_sample) for num_sample in args.num_samples.split(",")]


def run_fetch_env_frame1(args, time_limit, num_sample, folder_name, num_division):
    # env-frame: 0.48...
    starts = [[0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0]]
    goals = [
        #[0.201046354657216, -0.498794712940752, -0.46997820806098, 0.383476005765817, -1.62054231450741, 0.275555806646242, 0.0946758712721159, 2.24358280398006],
        [0.139831556721525, 0.759271929076082, 0.201556195547314, -0.484104715725937, -2.12029373509578, 2.57134155795376, -1.67538426226056, 3.01697448830681],
        [0.321441237503027, 1.46708311573917, 0.503881132531543, -0.906503034349056, -0.795844068853335, -2.74375353968961, -0.948066363021766, -1.52691201354791],
        #[0.348251912818693, 0.857482139396653, -0.868182415362813, -1.38659795236808, -1.67306880274813, 2.44314001299847, 1.32103302615282, -0.623311300998885],
        #[0.258689500240738, -0.638741313290659, -0.340352449066325, -2.42704455444847, 0.859617967852464, -1.06694972172585, 1.29067054311535, 1.7767285245185],
        #[0.152665020458081, 0.289434428970899, 0.366096258792135, 2.74997944369863, 1.08232221444816, -1.61963871941389, 0.0843414754844334, -2.42602847565961],
        #[0.0775970605172425, 0.998439137947507, 0.208005843146859, 0.366034598233787, -0.867845714574326, -0.815653903008816, 0.29494203740015, 1.43653954964438],
        #[0.286093687972178, -1.44369824762342, -0.0481009251598654, -0.474462539866498, -0.912188383857106, 2.66636538877806, -0.455061349427888, 0.483050895913263],
        #[0.146939610764921, 0.136223879197724, -0.605540903587355, -1.65889325401567, -0.0608858058427932, -2.99208069385891, -1.64823616692078, -0.257763787207701],
        #[0.319832730072171, -0.485637626688431, -0.7277581708759, 0.738216090096285, 0.562851073879691, 0.376542591357429, 1.68734500211665, 2.96444030105175],
        #[0.223613127677614, -1.28400661254718, -0.765716809062011, 1.84281078759862, 1.48895295902402, 1.84063379716598, 1.19705290081946, 1.05792512702494],
        #[0.121809175009345, -0.119510620507773, -0.787405722850186, -1.70891687180038, 1.5246007015228, -0.270972997728531, -0.80486354927765, 2.15395689241382],
        #[0.280114953587402, 0.98619819723558, 0.0993662299206755, 2.00289266153983, -0.918202945478596, -2.51410265984737, -1.12843259457345, 1.61727526236],
        #[0.122509863117158, 1.05719833557325, 1.03340643466207, 2.53719559690601, 1.15820419599424, 0.0219828325738032, -0.239347453293771, -0.792570638067625],
        #[0.373383981773609, 1.56476944849679, 0.796779336423969, 0.614937686796739, -0.597715891533927, 1.00476593197605, 0.84886195368422, -0.236604489999054],
        #[0.0630504634345727, 0.517170492677152, -0.678158114808455, -2.34592050554156, 1.37386892751582, -2.80772512877203, 1.54337705502732, -2.87196427602299],
        #[0.0642929354149548, 0.637988098034972, -0.458248312769571, -0.8064084711036, 1.04604295495824, -2.08292203732094, 0.544322422165981, -0.33893691151586],
        #[0.339234923932843, -1.1881816811633, -0.0670581779118808, -1.56492359171492, 0.388227686205589, -2.53485071105795, -0.275282773947172, 1.73778452630881],
        #[0.147048915187466, -1.00894651285725, -0.785359762527153, -2.40885885108489, 0.565318303011733, 1.53342162469936, -0.403834666361805, 0.363596681927497],
        #[0.109045356416125, -1.14282135999458, -0.431245219978354, 2.50429434839354, 0.205365994897611, -1.70683467252075, 1.3482288557662, 2.56625255355814],
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
        

    command += " --lambda_type {lambda_type} --coordinator {coordinator} --num_samples {num_samples} --num_divisions {num_divisions} --time-limit {time_limit} --load_graph {load_graph}".format(num_samples=num_sample, num_divisions=num_division, time_limit=time_limit, lambda_type=lambda_type, coordinator=coordinator, load_graph=load_graph)
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

def run_png(args, time_limit, num_sample, folder_name, num_division):

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
        

    command += " --lambda_type {lambda_type} --coordinator {coordinator} --num_samples {num_samples} --num_divisions {num_divisions} --time-limit {time_limit} --load_graph {load_graph}".format(num_samples=num_sample, num_divisions=num_division, time_limit=time_limit, lambda_type=lambda_type, coordinator=coordinator, load_graph=load_graph)
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

if __name__ == "__main__":
    #time_limits = [5, 10, 20] #, 2, 3]
    #time_limits = [1, 5, 10, 20, 30] #[0.2, 0.5, ]#1]
    #num_samples = [1, ] #10, 20, 40]
    #num_trials = 30
    for scenario in args.scenario:
        if scenario == "png":
            num_divisions = [
                (0,0),
                (1,1),
                (2,2),
            ]
        else:
            num_divisions = [
                (0,0,0,0,0,0,0,0),  #1
                #(1,0,0,0,0,0,0,0),  #2
                (1,1,0,0,0,0,0,0),  #4
                #(2,1,0,0,0,0,0,0),  #6
                #(1,1,1,0,0,0,0,0),  #8
                #(1,1,1,1,0,0,0,0),  #16
                #(1,1,1,1,1,0,0,0),  #32
                #(1,1,1,1,1,1,0,0),  #64
                #(1,1,1,1,1,1,1,0),  #128

            ]
        root = "outputs/{}".format(scenario)
        if not os.path.exists(root):
            os.makedirs(root)
        for time_limit in args.time_limits:
            for num_sample in args.num_samples:
                for num_division_tup in num_divisions:
                    for trial in range(args.num_trials):
                        
                        num_lambdas = reduce(lambda a,b: (a) * (b), [val + 1 for val in num_division_tup])
                        num_division = reduce(lambda a,b: "{},{}".format(a,b), num_division_tup)
                        file_name = "{root}/num_divisions={num_divisions}__num_lambdas={num_lambdas}__time_limit={time_limit}__trial={trial}__num_samples={num_samples}".format(num_divisions=num_division, trial=trial, time_limit=time_limit, num_lambdas=num_lambdas, num_samples=num_sample, root=root)
                        #if os.path.exists(file_name): continue
                        if scenario == "fetch1":
                            run_fetch_env_frame1(args, time_limit, num_sample, file_name, num_division)
                        elif scenario == "png":
                            run_png(args, time_limit, num_sample, file_name, num_division)

                        print("-------------------------------------------------------------------------------------")
                        time.sleep(2) 
                        #command = "rm -f png_2d_demo_output*; rm -f lambda-*; make -j8; ./mpl_coordinator --scenario png --global_min 0,0 --global_max 1403,1404 --env resources/house_layout.png --start 1301,332 --goal 600,1034 --goal 275,87 --goal 789,709 --goal 533,822 --goal 357,142 --goal 884,1173 --goal 446,898 --goal 82,478 --goal 952,339 --goal 6,394 --lambda_type local --coordinator localhost --num_samples {num_samples} --num_divisions {num_divisions} --time-limit {time_limit}".format(num_samples=num_sample, num_divisions=num_division, time_limit=time_limit, )
                        #subprocess.call("mkdir {folder_name}; mv lambda-* {folder_name}".format(folder_name=file_name[:-4]), shell=True)
                        #subprocess.call("mkdir {folder_name}; mv png* {folder_name}; mv lambda-* {folder_name}".format(folder_name=file_name[:-4]), shell=True)

#### fetch stuff
