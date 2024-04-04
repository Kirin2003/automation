import itertools
import json
import os
import random
import subprocess
import time
from deap import base
from deap import creator
from deap import tools
from map_info_parser import *
from feature_generator import *
from automation.auxiliary.map import map_tools
import automation.predictor_fuzzer.random_gen as random_gen

obs_folder = "/apollo/modules/tools/perception/obstacles/"
dest = "/apollo/automation/scenario_generator"
features_file = "mut_features.csv"
ga_file = "ga_output.csv"
timer_file = "execution_time.csv"

#return a mapping from x,y coor to laneIds and viceVersa (info extracted from routing_map of sunnyvale_loop)
ptl_dict, ltp_dict, diGraph = initialize()
obstacle_type = ["PEDESTRIAN", "BICYCLE", "VEHICLE"]


def check_trajectory(p_index1, p_index2):
    valid_path = False
    while not valid_path:
        valid_path = validatePath(p_index1, p_index2, ptl_dict, ltp_dict, diGraph) and longerTrace(
            list(ptl_dict.keys())[p_index1], list(ptl_dict.keys())[p_index2], ptl_dict, ltp_dict, diGraph)
        if not valid_path:
            p_index1 = random.randint(0, len(ptl_dict.keys()) - 1)
            p_index2 = random.randint(0, len(ptl_dict.keys()) - 1)
    return p_index1, p_index2


def check_obs_type(length, width, height, speed, type_index):
    obs_type = obstacle_type[type_index]
    if obs_type is "VEHICLE":
        if length < 4.0 or length > 14.5:
            length = random.uniform(4.0, 14.5)
        if height < 1.5 or height > 4.7:
            height = random.uniform(1.5, 4.7)
        if width < 1.5 or width > 2.5:
            width = random.uniform(1.5, 2.5)
        if speed < 2.5 or speed > 20:
            speed = random.uniform(2.5, 20)
        diversity_counter["V"] += 1
    if obs_type is "PEDESTRIAN":
        if length < 0.2 or length > 0.5:
            length = random.uniform(0.2, 0.5)
        if height < 0.97 or height > 1.87:
            height = random.uniform(0.97, 1.87)
        if width < 0.3 or width > 0.8:
            width = random.uniform(0.3, 0.8)
        if speed < 1.25 or speed > 3:
            speed = random.uniform(1.25, 3)
        diversity_counter["P"] += 1
    if obs_type is "BICYCLE":
        if length < 1 or length > 2.5:
            length = random.uniform(1, 2.5)
        if height < 1 or height > 2.5:
            height = random.uniform(1, 2.5)
        if width < 0.5 or width > 1:
            width = random.uniform(0.5, 1)
        if speed < 1.75 or speed > 7:
            speed = random.uniform(1.75, 7)
        diversity_counter["B"] += 1
    return length, width, height, speed


def runScenario(deme, record_name):
    # YIFAN added
    random_gen.fuzz_config()
    #to start with a fresh set of obstacles for the current scnerio
    os.system("rm -f /apollo/modules/tools/perception/obstacles/*")
    if(record_name == "Generation0_Scenario1"):
        os.system("rm /apollo/cyber/haoran_test.txt")
    global diversity_counter
    diversity_counter = {"V": 0, "P": 0, "B": 0}
    for ind in deme:
        p_index1, p_index2 = check_trajectory(ind[1], ind[2])
        ind[1] = p_index1
        ind[2] = p_index2
        #get the x,y coor coorespoding to index i1 and i2
        p1 = list(ptl_dict.keys())[p_index1]
        p2 = list(ptl_dict.keys())[p_index2]
        #get the correspodning lane id where the points reside
        lane1 = ptl_dict[p1]
        lane2 = ptl_dict[p2]
        #find the path between two lanes
        path = nx.shortest_path(diGraph, source=lane1, target=lane2)
        #verify that obstacle type and size is realistic
        ind[4], ind[5], ind[6], ind[7] = check_obs_type(
            ind[4], ind[5], ind[6], ind[7], ind[8])
        #ensure there are no two obstacles with similar id
        unique_obs_id = False
        while not unique_obs_id:
            if os.path.exists(os.path.join(obs_folder, "sunnyvale_loop_obs{}.json".format(ind[0]))):
                ind[0] = random.randint(0, 30000)
            else:
                unique_obs_id = True
        #generate the desc files (each desc file corresponds to one individual/obstacle)
        desc = generateObsDescFile(
            ind[0], ind[3], ind[4], ind[5], ind[6], ind[7], obstacle_type[ind[8]])
        desc = produceTrace(p1, p2, path, ltp_dict, desc)
        filename = "sunnyvale_loop_obs" + str(desc["id"]) + ".json"
        with open(os.path.join(obs_folder, filename), 'w') as outfile:
            json.dump(desc, outfile)

    failed = True
    num_runs = 0
    while failed:
        #if scenario has been restarted x times, restart the moodules and sim control
        if num_runs % 10 == 0 and num_runs != 0:
            os.system("bash /apollo/scripts/bootstrap.sh stop")
            time.sleep(10)
            os.system("bash /apollo/scripts/bootstrap.sh start")
            time.sleep(10)
            os.system(
                "bash /apollo/automation/auxiliary/modules/start_modules.sh")
            time.sleep(10)
            os.system("source /apollo/cyber/setup.bash")
            time.sleep(2)
            print("attempted %s run" % num_runs)
        # ------- sending valid adc routing -------
        valid_path = False
        while not valid_path:
            p_index1 = random.randint(0, len(ptl_dict.keys()) - 1)
            p_index2 = random.randint(0, len(ptl_dict.keys()) - 1)
            start_point = tuple(
                map(float, list(ptl_dict.keys())[p_index1].split('-')))
            if not map_tools.all_points_not_in_junctions(start_point):
                p1 = list(ptl_dict.keys())[p_index1]
                p2 = list(ptl_dict.keys())[p_index2]
                continue
            valid_path = validatePath(
                p_index1, p_index2, ptl_dict, ltp_dict, diGraph)
        p1 = list(ptl_dict.keys())[p_index1]
        p2 = list(ptl_dict.keys())[p_index2]
        adc_routing = p1.replace('-', ',') + "," + p2.replace('-', ',')
        # ------- running the scneario -------
        # 通过运行结果来计算适应度函数
        scenario_player_cmd = 'bazel run --experimental_ui_limit_console_output=1 //automation/scenario_player:run_automation -- -rv ' + \
            adc_routing + ' -o ' + record_name
        scenario_player_output = subprocess.check_output(
            scenario_player_cmd, shell=True)
        scenario_player_output = str(scenario_player_output)[2:-3]
        num_runs = num_runs + 1
        #print(scenario_player_output)
        #if the adc didn't move or the adc was travelling outside the map boundaries, then re-run scenrio with new routing info
        if scenario_player_output == 'None':
            continue
        scenario_player_output = scenario_player_output.split('\\n')
        min_distance = eval(scenario_player_output[0])
        #the return number of obstacles must match the ones in the individual
        if len(min_distance) != len(deme):
            continue
        else:
            failed = False
    #scenario run successfully
    sim_time = float(scenario_player_output[8]) * num_runs
    orcle_time = float(scenario_player_output[9]) * num_runs
    lanes, min_distance, speeding_min, uslc_min, fastAccl_min, hardBrake_min = runOracles(
        scenario_player_output, record_name, deme)
    os.system(
        "mv /apollo/cyber/haoran_test.txt /apollo/evaluators_cov/evaluators_" + record_name + ".txt")
    return lanes, min_distance, speeding_min, uslc_min, fastAccl_min, hardBrake_min, sim_time, orcle_time, num_runs

# ------- Main Function -------


def main():
    # ------- GA Definitions -------
    # Fitness and Individual generator
    # Shi: hard brake may give extremely large values, disable it for now
    # Shi: and add model coverage
    #creator.create("MultiFitness", base.Fitness, weights=(-1.0,-1.0,-1.0,1.0,-1.0))
    # 创建适应度类，继承自base.Fitness类，
    creator.create("MultiFitness", base.Fitness,
                   weights=(-1.0, -1.0, -1.0, 1.0, 1000))
    # 创建了一个名为Individual的个体类，它是以list为基础，并有一个MultiFitness类型的适应度属性
    creator.create("Individual", list, fitness=creator.MultiFitness)
    # 创建工具箱，工具箱是DEAP中用于注册各种操作（如初始化、交叉、变异等）的地方。
    toolbox = base.Toolbox()
    # Attribute generator (9 obstacle attributes)
    # 使用random.randint函数生成一个在0到30000之间的随机整数作为id属性
    toolbox.register("id", random.randint, 0, 30000)
    toolbox.register("start_pos", random.randint, 0, len(ptl_dict.keys()) - 1)
    toolbox.register("end_pos", random.randint, 0, len(ptl_dict.keys()) - 1)
    toolbox.register("theta", random.uniform, -3.14, 3.14)
    toolbox.register("length", random.uniform, 0.2, 14.5)
    toolbox.register("width", random.uniform, 0.3, 2.5)
    toolbox.register("height", random.uniform, 0.97, 4.7)
    toolbox.register("speed", random.uniform, 1, 20)
    toolbox.register("type", random.randint, 0, 2)
    # Structure initializers
    # 这里使用tools.initCycle方法注册了一个初始化个体的函数。该函数使用之前注册的属性生成器来初始化Individual类的一个实例。
    # tools.initCycle函数用于根据给定的生成器列表和重复次数来初始化一个个体
    toolbox.register("individual", tools.initCycle, creator.Individual,
                     (toolbox.id, toolbox.start_pos, toolbox.end_pos , toolbox.theta, toolbox.length, toolbox.width, toolbox.height, toolbox.speed, toolbox.type), n=1)
    # define the deme to be a list of individuals (obstacles)
    # 这行代码注册了一个种群初始化器，使用tools.initRepeat方法创建一个包含多个个体的列表（即种群）。
    # deme是障碍物种群
    toolbox.register("deme", tools.initRepeat, list, toolbox.individual)
    # 交叉算子：两点交叉
    toolbox.register("mate", tools.cxTwoPoint)
    # 变异算子：均匀整数变异
    toolbox.register("mutate", tools.mutUniformInt, low=(0, 0, 0, -3, 1, 1, 1, 1, 0), up=(
        30000, len(ptl_dict.keys()) - 1, len(ptl_dict.keys()) - 1, 3, 15, 3, 5, 20, 2), indpb=0.05)
    # 选择算子：NGSA2
    toolbox.register("select", tools.selNSGA2)

    NP = 10
    OBS_MAX = 15
    OBS_MIN = 10
    TOTAL_LANES = 60
    ETIME = 6000  # execution time end (in seconds) after 100mins
    #ETIME=1800 # execution time end (in seconds) after 30mins
    GLOBAL_LANE_COVERAGE = set()
    # 初始化每一代的个体数，每一代的个体数在10-15
    DEME_SIZES = [random.randint(OBS_MIN, OBS_MAX) for p in range(0, NP)]
    CXPB, MUTPB, ADDPB, DELPB = 0.8, 0.2, 0.1, 0.1
    # 按照每一代的个体数来初始化每一代的障碍物种群
    # 假设DEME_SIZES=[10,10,10,10,10,15,15,15,15,15]，则pop是十个子种群，子种群的个体数分别为10,10,10,10,10,15,15,15,15,15
    pop = [toolbox.deme(n=i) for i in DEME_SIZES]
    # 在DEAP库中，tools.HallOfFame是一个用于存储最佳个体的数据结构。当你运行进化算法时，例如遗传算法或差分进化，种群中的个体经过一系列的遗传操作（如选择、交叉、变异）后，会产生新的个体，并且通常会根据某种适应度函数来评估这些个体的优劣。HallOfFame用于跟踪并记录这些个体中表现最好的一个或多个个体。
    hof = tools.HallOfFame(NP)  # best ind in each scenario
    lane_coverage = {scenario_num: set() for scenario_num in range(1, NP + 1)}
    scenario_counter = 1
    g = 0

    #store features output and evolution output
    labels = "record_name,c_x,c_y,c_type,adc_heading,adc_speed,obs_id,obs_heading,obs_speed,obs_type,obs_len,obs_wid,obs_height,"\
        "speeding_x,speeding_y,speeding_value,speeding_duration,speeding_heading,lanes_speed_limit,uslc_x,uslc_y,uslc_duration,uslc_heading,"\
        "fastAccl_x,fastAccl_y,fastAccl_value,fastAccl_duration,fastAccl_heading,hardBrake_x,hardBrake_y,hardBrake_value,hardBrake_duration,hardBrake_heading,"\
        "c_counter,speeding_counter,uslc_counter,fastAccl_counter,hardBrake_counter,totalV\n"
    with open(os.path.join(dest, features_file), 'w') as ffile:
      ffile.write(labels)
    labels = "RecordName,ObsNum,P,B,V,AVG_OBS2ADC_Distance,Speed_Below_Limit,ADC2LaneBound_Distance,FastAccl,HardBrake\n"
    with open(os.path.join(dest, ga_file), 'a+') as gfile:
        gfile.write(labels)
    labels = "RecordName,Simulation,Oracles,MISC,E2E,RetryNo\n"
    with open(os.path.join(dest, timer_file), 'a+') as tfile:
        tfile.write(labels)

    os.system("rm -rf /apollo/automation/grading_metrics/Safety_Violations/*")
    dump_file = '/apollo/cyber/auto_t_scenario_dump.txt'
    if os.path.exists(dump_file):
        os.remove(dump_file)
    print("Start of evolution")
    evaluators = {
        'CyclistKeepLaneEvaluator',
        'PedestrianInteractionEvaluator',
        'CruiseMLPEvaluator',
        'JunctionMapEvaluator',
        'JunctionMLPWvaluator',
        'LaneAggregatingEvaluator',
        'LaneScanningEvaluator',
        'MLPEvaluator',
        'SemanticLSTMEvaluator',
    }
    # 上面的9个evaluator两两组合
    pairwise_combinations = list(itertools.combinations(evaluators, 2))

    global_visited = set()
    global_pairwise_visited = set()
    start_time = time.time()
    for deme in pop:
        e2e_time = time.time()
        record_name = "Generation{}_Scenario{}".format(g, scenario_counter)

        print("YIFAN:", record_name, "start")
        lanes, min_distance, speeding_min, uslc_min, fastAccl_min, hardBrake_min, sim_time, orcle_time, num_runs = runScenario(
            deme, record_name)
        print("YIFAN:", record_name, "end")

        coverage_file = os.path.join(
            "/apollo/evaluators_cov/evaluators_{}.txt".format(record_name))
        visited = set()
        pairwise_visited = set()
        if os.path.exists(coverage_file):
            with open(coverage_file) as f:
                for line in f.readlines():
                    evaluator_name = line[10:-1]
                    visited.add(evaluator_name)
                    global_visited.add(evaluator_name)
            for combination in pairwise_combinations:
                e1, e2 = combination
                if e1 in visited and e2 in visited:
                    pairwise_visited.add("{}{}".format(e1, e2))
                    global_pairwise_visited.add("{}{}".format(e1, e2))
        else:
           raise RuntimeError("expected evaluator file")

        scenario_coverage = len(visited) / len(evaluators)
        scenario_pairwise_coverage = len(
            pairwise_visited) / len(pairwise_combinations)

        print("cnav: {} {} {}".format(record_name,
              scenario_coverage, scenario_pairwise_coverage))
        global_coverage = len(global_visited) / len(evaluators)
        global_pairwise_coverage = len(
            global_pairwise_visited) / len(pairwise_combinations)
        print("cnav: global {} {}".format(
            global_coverage, global_pairwise_coverage))

        lanes.remove('')
        GLOBAL_LANE_COVERAGE.update(lanes)
        lane_coverage[scenario_counter] = lane_coverage[scenario_counter].union(
            lanes)
        sum = 0
        print("shi: obs_min_dist, speeding_min, uslc_min, fastAccl_min, scenario_coverage")
        for ind in deme:
            obs_min_dist = min_distance[str(ind[0])]
            print(
                f"shi: {obs_min_dist}, {speeding_min}, {uslc_min}, {fastAccl_min}, {scenario_coverage}")
            # Shi: hard brake may give extremely large values, disable it for now
            # Shi: and add model coverage
            # ind.fitness.values = (obs_min_dist,speeding_min,uslc_min,fastAccl_min,hardBrake_min)
            ind.fitness.values = (obs_min_dist, speeding_min,
                                  uslc_min, fastAccl_min, scenario_coverage)
            sum += obs_min_dist
        with open(os.path.join(dest, ga_file), 'a+') as gfile:
            gfile.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n"
                        % (record_name, len(deme), diversity_counter["P"], diversity_counter["B"], diversity_counter["V"], sum / len(deme), speeding_min, uslc_min, fastAccl_min, hardBrake_min))
        e2e_time = time.time() - e2e_time
        misc_time = e2e_time - sim_time - orcle_time
        with open(os.path.join(dest, timer_file), 'a+') as tfile:
            tfile.write("{},{:.2f},{:.2f},{:.2f},{:.2f},{}\n".format(
                record_name, sim_time, orcle_time, misc_time, e2e_time, num_runs))
        scenario_counter += 1

    while len(GLOBAL_LANE_COVERAGE) < TOTAL_LANES and (time.time() - start_time) <= ETIME :
        g = g + 1
        scenario_counter = 1
        print("-- Generation %i --" % g)
        for deme in pop:
            e2e_time = time.time()
            offspring = list(map(toolbox.clone, deme))
            # Apply crossover and mutation on the offspring
            for child1, child2 in zip(offspring[::2], offspring[1::2]):
                if random.random() < CXPB:
                    toolbox.mate(child1, child2)
                    del child1.fitness.values
                    del child2.fitness.values
            for mutant in offspring:
                if random.random() < MUTPB:
                    toolbox.mutate(mutant)
                    del mutant.fitness.values
                if random.random() < DELPB and len(offspring) != 1:
                    worst_ind = tools.selWorst(offspring, 1)[0]
                    offspring.remove(worst_ind)
                    del mutant.fitness.values
                if random.random() < ADDPB and len(hof) != 0:
                    best_ind = hof[random.randint(0, int(len(hof) / 2))]
                    offspring.append(best_ind)
                    del mutant.fitness.values

            record_name = "Generation{}_Scenario{}".format(g, scenario_counter)
            print("YIFAN: phase2", record_name, "start")
            lanes, min_distance, speeding_min, uslc_min, fastAccl_min, hardBrake_min, sim_time, orcle_time, num_runs = runScenario(
                offspring, record_name)
            print("YIFAN: phase2", record_name, "end")

            visited = set()
            pairwise_visited = set()
            if os.path.exists(coverage_file):
                with open(coverage_file) as f:
                    for line in f.readlines():
                        evaluator_name = line[10:-1]
                        visited.add(evaluator_name)
                        global_visited.add(evaluator_name)
                    for combination in pairwise_combinations:
                        e1, e2 = combination
                        if e1 in visited and e2 in visited:
                            pairwise_visited.add("{}{}".format(e1, e2))
                            global_pairwise_visited.add("{}{}".format(e1, e2))
            else:
                raise RuntimeError("expected evaluator file")

            scenario_coverage = len(visited) / len(evaluators)
            scenario_pairwise_coverage = len(
                pairwise_visited) / len(pairwise_combinations)

            print("cnav: {} {} {}".format(record_name,
                  scenario_coverage, scenario_pairwise_coverage))

            lanes.remove('')
            GLOBAL_LANE_COVERAGE.update(lanes)
            lane_coverage[scenario_counter] = lane_coverage[scenario_counter].union(
                lanes)
            sum = 0
            print(
                "shi: obs_min_dist, speeding_min, uslc_min, fastAccl_min, scenario_coverage")
            for ind in offspring:
                obs_min_dist = min_distance[str(ind[0])]
                print(
                    f"shi: {obs_min_dist}, {speeding_min}, {uslc_min}, {fastAccl_min}, {scenario_coverage}")
                # Shi: hard brake may give extremely large values, disable it for now
                # Shi: and add model coverage
                #ind.fitness.values = (obs_min_dist,speeding_min,uslc_min,fastAccl_min,hardBrake_min)
                ind.fitness.values = (
                    obs_min_dist, speeding_min, uslc_min, fastAccl_min, scenario_coverage)
                sum += obs_min_dist
            with open(os.path.join(dest, ga_file), 'a+') as gfile:
                gfile.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n"
                            % (record_name, len(deme), diversity_counter["P"], diversity_counter["B"], diversity_counter["V"], sum / len(deme), speeding_min, uslc_min, fastAccl_min, hardBrake_min))
            hof.insert(tools.selBest(offspring, 1)[0])
            deme = toolbox.select(deme + offspring, len(offspring))

            e2e_time = time.time() - e2e_time
            misc_time = e2e_time - sim_time - orcle_time
            with open(os.path.join(dest, timer_file), 'a+') as tfile:
                tfile.write("{},{:.2f},{:.2f},{:.2f},{:.2f},{}\n".format(
                    record_name, sim_time, orcle_time, misc_time, e2e_time, num_runs))
            scenario_counter += 1

            if (time.time() - start_time) >= ETIME:
                break

    global_coverage = len(global_visited) / len(evaluators)
    global_pairwise_coverage = len(
        global_pairwise_visited) / len(pairwise_combinations)

    print("cnav: global {} {}".format(global_coverage, global_pairwise_coverage))
    print("-- End of (successful) evolution --")

    # ------- Final Results -------
    end_time = time.time()
    print("-- Execution Time: %.2f  seconds --\n" % (end_time - start_time))
    print("*** Total Num. of Lanes Covered:%s out of %s ***\n" %
          (len(GLOBAL_LANE_COVERAGE), TOTAL_LANES))


if __name__ == "__main__":
    main()
