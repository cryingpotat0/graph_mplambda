import os, time, subprocess
from functools import reduce

starts = [[0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0]]
#[0.201046354657216, -0.498794712940752, -0.46997820806098, 0.383476005765817, -1.62054231450741, 0.275555806646242, 0.0946758712721159, 2.24358280398006],
#[0.115368692562692, -0.445314228868808, 0.0963735806867418, -1.27016578588354, -0.967086631735602, 2.66528726389368, 0.556858793495702, 1.60240973636245],
#[0.139831556721525, 0.759271929076082, 0.201556195547314, -0.484104715725937, -2.12029373509578, 2.57134155795376, -1.67538426226056, 3.01697448830681],
#[0.265605054307725, -0.0406835434103203, 0.794652977451368, 0.557126184096162, 0.128090244287742, -0.278693634784349, 1.41827116528597, 2.32724198527326],
#[0.23381020844656, 1.45648483488815, 1.4929787456749, 2.3063739960737, 0.64878909206156, 0.657563607266227, 1.78281764532597, -0.20735587150957],
#[0.321441237503027, 1.46708311573917, 0.503881132531543, -0.906503034349056, -0.795844068853335, -2.74375353968961, -0.948066363021766, -1.52691201354791],
#[0.34529600622738, -1.23026860284713, 0.571034997136581, -2.97475900119163, -0.96454711244822, 0.702692815925102, -0.58016987053878, -1.3749651066112],
#[0.348251912818693, 0.857482139396653, -0.868182415362813, -1.38659795236808, -1.67306880274813, 2.44314001299847, 1.32103302615282, -0.623311300998885],
#[0.127106408451682, 0.296232230941623, 1.04180281931571, -0.207161983361287, -1.05093069141864, 1.93207830145142, -1.60737483189966, -0.831110912689584],
#[0.143329632165137, -0.845592815411368, -0.785670533257555, -2.90991127435503, -1.80331904462047, 1.15575380297583, 1.95318185620889, 1.80393085782257],
#[0.267259836704116, -1.3290661469169, -0.239493471101844, -1.3425147752827, -1.82637111302761, 2.78726246375308, -0.120136757782221, 0.124171653514422],
#[0.258689500240738, -0.638741313290659, -0.340352449066325, -2.42704455444847, 0.859617967852464, -1.06694972172585, 1.29067054311535, 1.7767285245185],
#[0.141854837771701, -1.00494000985811, 0.174876648883185, -0.451941967433952, 1.23981136430659, -3.04579963246683, 0.0948442296776806, -2.78883846616237],
#[0.152665020458081, 0.289434428970899, 0.366096258792135, 2.74997944369863, 1.08232221444816, -1.61963871941389, 0.0843414754844334, -2.42602847565961],
#[0.339826762723352, -1.30895296794471, -0.791704366566936, 3.01585290457046, -1.67428897876619, -1.13049672479368, 1.76633735067612, -0.158584020748358],
#[0.0775970605172425, 0.998439137947507, 0.208005843146859, 0.366034598233787, -0.867845714574326, -0.815653903008816, 0.29494203740015, 1.43653954964438],
#[0.252837015294025, -0.606251092237807, 0.845027114214161, 2.58374368689819, 2.2448268517385, 2.47134652207135, 1.20798529001607, 1.2156791409268],
#[0.0984416507460425, 0.902578953040868, 1.44026345390896, -0.597036413241875, -1.35665388248141, 2.86210577896741, 1.43610384769603, -1.52394683731949],
#[0.286093687972178, -1.44369824762342, -0.0481009251598654, -0.474462539866498, -0.912188383857106, 2.66636538877806, -0.455061349427888, 0.483050895913263],
#[0.37724685896517, 0.162246270449604, 0.341650770565467, 2.53395855564833, -2.0152528112263, -0.114407284204447, 1.52012922738177, -2.56992411186859]
#]

goals = [
[0.201046354657216, -0.498794712940752, -0.46997820806098, 0.383476005765817, -1.62054231450741, 0.275555806646242, 0.0946758712721159, 2.24358280398006],
[0.139831556721525, 0.759271929076082, 0.201556195547314, -0.484104715725937, -2.12029373509578, 2.57134155795376, -1.67538426226056, 3.01697448830681],
[0.321441237503027, 1.46708311573917, 0.503881132531543, -0.906503034349056, -0.795844068853335, -2.74375353968961, -0.948066363021766, -1.52691201354791],
[0.348251912818693, 0.857482139396653, -0.868182415362813, -1.38659795236808, -1.67306880274813, 2.44314001299847, 1.32103302615282, -0.623311300998885],
[0.258689500240738, -0.638741313290659, -0.340352449066325, -2.42704455444847, 0.859617967852464, -1.06694972172585, 1.29067054311535, 1.7767285245185],
[0.152665020458081, 0.289434428970899, 0.366096258792135, 2.74997944369863, 1.08232221444816, -1.61963871941389, 0.0843414754844334, -2.42602847565961],
[0.0775970605172425, 0.998439137947507, 0.208005843146859, 0.366034598233787, -0.867845714574326, -0.815653903008816, 0.29494203740015, 1.43653954964438],
[0.286093687972178, -1.44369824762342, -0.0481009251598654, -0.474462539866498, -0.912188383857106, 2.66636538877806, -0.455061349427888, 0.483050895913263],
[0.146939610764921, 0.136223879197724, -0.605540903587355, -1.65889325401567, -0.0608858058427932, -2.99208069385891, -1.64823616692078, -0.257763787207701],
[0.319832730072171, -0.485637626688431, -0.7277581708759, 0.738216090096285, 0.562851073879691, 0.376542591357429, 1.68734500211665, 2.96444030105175],
[0.223613127677614, -1.28400661254718, -0.765716809062011, 1.84281078759862, 1.48895295902402, 1.84063379716598, 1.19705290081946, 1.05792512702494],
[0.121809175009345, -0.119510620507773, -0.787405722850186, -1.70891687180038, 1.5246007015228, -0.270972997728531, -0.80486354927765, 2.15395689241382],
[0.280114953587402, 0.98619819723558, 0.0993662299206755, 2.00289266153983, -0.918202945478596, -2.51410265984737, -1.12843259457345, 1.61727526236],
[0.122509863117158, 1.05719833557325, 1.03340643466207, 2.53719559690601, 1.15820419599424, 0.0219828325738032, -0.239347453293771, -0.792570638067625],
[0.373383981773609, 1.56476944849679, 0.796779336423969, 0.614937686796739, -0.597715891533927, 1.00476593197605, 0.84886195368422, -0.236604489999054],
[0.0630504634345727, 0.517170492677152, -0.678158114808455, -2.34592050554156, 1.37386892751582, -2.80772512877203, 1.54337705502732, -2.87196427602299],
[0.0642929354149548, 0.637988098034972, -0.458248312769571, -0.8064084711036, 1.04604295495824, -2.08292203732094, 0.544322422165981, -0.33893691151586],
[0.339234923932843, -1.1881816811633, -0.0670581779118808, -1.56492359171492, 0.388227686205589, -2.53485071105795, -0.275282773947172, 1.73778452630881],
[0.147048915187466, -1.00894651285725, -0.785359762527153, -2.40885885108489, 0.565318303011733, 1.53342162469936, -0.403834666361805, 0.363596681927497],
[0.109045356416125, -1.14282135999458, -0.431245219978354, 2.50429434839354, 0.205365994897611, -1.70683467252075, 1.3482288557662, 2.56625255355814],
]


if __name__ == "__main__":
    #time_limits = [5, 10, 20] #, 2, 3]
    time_limits = [1, 5, 10, 20, 30] #[0.2, 0.5, ]#1]
    num_samples = [1, ] #10, 20, 40]
    num_divisions = [
        #(0,0),
        #(1,1),
        #(2,2),
        (0,0,0,0,0,0,0,0),  #1
        #(1,0,0,0,0,0,0,0),  #2
        (1,1,0,0,0,0,0,0),  #4
        #(2,1,0,0,0,0,0,0),  #6
        (1,1,1,0,0,0,0,0),  #8
        (1,1,1,1,0,0,0,0),  #16
        (1,1,1,1,1,0,0,0),  #32
        (1,1,1,1,1,1,0,0),  #64
        #(1,1,1,1,1,1,1,0),  #128

    ]
    num_trials = 30
    for time_limit in time_limits:
        for num_sample in num_samples:
            for num_division_tup in num_divisions:
                for trial in range(num_trials):
                    
                    num_lambdas = reduce(lambda a,b: (a) * (b), [val + 1 for val in num_division_tup])
                    num_division = reduce(lambda a,b: "{},{}".format(a,b), num_division_tup)
                    #num_sample = num_sample_for_one // num_lambdas
                    file_name = "outputs/num_divisions={num_divisions}__num_lambdas={num_lambdas}__time_limit={time_limit}__trial={trial}__num_samples={num_samples}.txt".format(num_divisions=num_division, trial=trial, time_limit=time_limit, num_lambdas=num_lambdas, num_samples=num_sample)
                    if os.path.exists(file_name): continue
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
                    command += " --lambda_type aws --coordinator 54.188.233.199 --num_samples {num_samples} --num_divisions {num_divisions} --time-limit {time_limit}".format(num_samples=num_sample, num_divisions=num_division, time_limit=time_limit)
                    #command += " --lambda_type local --coordinator localhost --num_samples {num_samples} --num_divisions {num_divisions} --time-limit {time_limit} ".format(num_samples=num_sample, num_divisions=num_division, time_limit=time_limit, )

                    #command = "rm -f png_2d_demo_output*; rm -f lambda-*; make -j8; ./mpl_coordinator --scenario png --global_min 0,0 --global_max 1403,1404 --env resources/house_layout.png --start 1301,332 --goal 600,1034 --goal 275,87 --goal 789,709 --goal 533,822 --goal 357,142 --goal 884,1173 --goal 446,898 --goal 82,478 --goal 952,339 --goal 6,394 --lambda_type local --coordinator localhost --num_samples {num_samples} --num_divisions {num_divisions} --time-limit {time_limit}".format(num_samples=num_sample, num_divisions=num_division, time_limit=time_limit, )
                    print(command)
                    print(file_name)
                    subprocess.call(command, shell=True, stderr=open(file_name, 'w'))
                    #subprocess.call("mkdir {folder_name}; mv lambda-* {folder_name}".format(folder_name=file_name[:-4]), shell=True)
                    #subprocess.call("mkdir {folder_name}; mv png* {folder_name}; mv lambda-* {folder_name}".format(folder_name=file_name[:-4]), shell=True)
                    print("-------------------------------------------------------------------------------------")
                    time.sleep(2) 

#### fetch stuff
