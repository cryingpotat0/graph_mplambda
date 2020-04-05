import os, time

if __name__ == "__main__":
    time_limits = [10]
    num_samples_for_one_set = [40]
    num_divisions = [(0,0), (1,0), (1,1), (2,1), (2,2)]
    num_trials = 2
    for (time_limit, num_samples_for_one) in zip(time_limits, num_samples_for_one_set):
        for num_division_tup in num_divisions:
            for trial in range(num_trials):
                num_lambdas = reduce(lambda a,b: (a+1) * (b+1), num_division_tup)
                num_samples = int(num_samples_for_one // num_lambdas)
                num_division = reduce(lambda a,b: "{},{}".format(a,b), num_division_tup)
                file_name = "outputs/num_divisions={num_divisions}__num_lambdas={num_lambdas}__time_limit={time_limit}__trial={trial}__num_samples={num_samples}.txt".format(num_divisions=num_division, trial=trial, time_limit=time_limit, num_lambdas=num_lambdas, num_samples=num_samples)
                if os.path.exists(file_name): continue
                command = "rm -f png_2d_demo_output*; make -j8; ./mpl_coordinator --scenario png --global_min 0,0 --global_max 1403,1404 --env resources/house_layout.png --start 1301,332 --goal 600,1034 --start 977,1140 --goal 275,87 --start 249,683 --goal 789,709 --start 1328,438 --goal 533,822 --start 695,919 --goal 357,142 --start 797,196 --goal 884,1173 --start 828,327 --goal 446,898 --start 962,1140 --goal 82,478 --start 275,789 --goal 952,339 --start 772,1307 --goal 6,394 --lambda_type aws --coordinator 54.188.233.199 --num_samples {num_samples} --num_divisions {num_divisions} --time-limit {time_limit} &> {file_name}".format(num_samples=num_samples, num_divisions=num_division, time_limit=time_limit, file_name=file_name)
                print(command)
                os.system(command)
                print("-------------------------------------------------------------------------------------")
                time.sleep(2) 

