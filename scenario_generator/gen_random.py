import argparse
import json
import os
import random
import math
import time
from feature_gen_helper import *

map = "sunnyvale"
park = []
routing = {}
config_folder = "/apollo/automation/config"
obs_folder = "/apollo/modules/tools/perception/obstacles"
dest="/apollo/automation/scenario_generator"
features_file="rand_features.csv"
ga_file="ga_output.csv"
timer_file="execution_time.csv"
park_obs_type = ["STATIC", "PARKING_IN", "PARKING_OUT"]

def load_config(map):
    park_config_path = os.path.join(config_folder, map, 'park_config.json')
    with open(park_config_path, 'r') as f:
        json.load(park, f)

    routing_config_path = os.path.join(config_folder, map, 'routing_config.json')
    with open(routing_config_path, 'r') as f:
        json.load(routing, f)  

def generate_park_obs(num):
    for i in range(num):
        obs_id=random.randint(0, 30000)
        #ensure there are no two obstacles with similar id
        unique_obs_id=False
        while not unique_obs_id:
            if os.path.exists(os.path.join(obs_folder,"sunnyvale_loop_obs{}.json".format(obs_id))):
                obs_id=random.randint(0,30000)
            else:
                unique_obs_id=True
        
        park_desc_file_path = os.path.join(obs_folder,"sunnyvale_loop_obs{}.json".format(obs_id))
        
        # park_obs = {park_type, id, skill}
        park_type_idx = random.randint(0,2)
        skill = random.random()
        id = random.randint(0, len(park) - 1)
        theta = random.uniform(-3.14, 3.14)
        length,width,height,speed=check_obs_type("VEHICLE") 


        park_id = park[id]["park_id"]
        points = park[id]["points"]
        park_type = park_obs_type[park_type_idx]

        park_desc = {}

        
        if park_type is "STATIC":
            park_desc = generate_obs_desc(unique_obs_id, points[0], theta, length, width, height, 0.0, 1.0, "VEHICLE", None)
        if park_type is "PARKING_IN":
            park_desc = generate_obs_desc(unique_obs_id, points[-1], theta, length, width, height, speed, 1.0, "VEHICLE", points.reverse())   
        if park_type is "PARKING_OUT":
            park_desc = generate_obs_desc(unique_obs_id, points[0], theta, length, width, height, speed, 1.0, "VEHICLE", points)

        with open(park_desc_file_path, 'w') as f:
            json.dump(park_desc, f) 

        scenario.append([obs_id, theta, length, width, height, speed, "PARKING"])

def check_obs_type(obs_type):
    if obs_type is "VEHICLE":
        length=random.uniform(4.0,14.5)
        height=random.uniform(1.5,4.7)
        width=random.uniform(1.5,2.5)
        speed=random.uniform(1,5.5)
    if obs_type is "PEDESTRIAN":
        length=random.uniform(0.2,0.5)
        height=random.uniform(0.97,1.87)
        width=random.uniform(0.3,0.8)
        speed=random.uniform(0,3)
    return length,width,height,speed

def generate_obs_desc(unique_obs_id, position, theta, length, width, height, speed, tracking_time, obs_type, trace):
    desc = {}
    desc["id"] = unique_obs_id
    desc["position"] = position
    desc["theta"] = theta
    desc["length"] = length
    desc["width"] = width
    desc["height"] = height
    desc["speed"] = speed
    desc["tracking_time"] = tracking_time
    desc["type"] = obs_type
    if trace != None:
        desc["trace"] = trace
    return desc

        
def generate_straight_obs(num):
    for i in range(num):
        obs_id=random.randint(0, 30000)
        #ensure there are no two obstacles with similar id
        unique_obs_id=False
        while not unique_obs_id:
            if os.path.exists(os.path.join(obs_folder,"sunnyvale_loop_obs{}.json".format(obs_id))):
                obs_id=random.randint(0,30000)
            else:
                unique_obs_id=True
        
        theta = random.uniform(-3.14, 3.14)
        length,width,height,speed=check_obs_type("VEHICLE") 
        position = [routing["start"]["x"], routing["start"]["y"], 0.0]
        trace = []
        trace.append([routing["start"]["x"], routing["start"]["y"], 0.0])
        trace.append([routing["end"]["x"], routing["end"]["y"], 0.0])
        straight_desc = generate_obs_desc(unique_obs_id, position, theta, length, width, height, speed, 1.0, "VEHICLE", trace)
        
        
        straight_desc_file_path = os.path.join(obs_folder, "sunnyvale_loop_obs{}.json".format(obs_id))
        
        with open(straight_desc_file_path, 'w') as f:
            json.dump(straight_desc, f) 

        scenario.append([obs_id, theta, length, width, height, speed, "STRAIGHT"])


def generate_pedestrian_obs(num):

    TRACE_MAX = 15
    TRACE_MIN = 3
    for i in range(num):
        obs_id=random.randint(0, 30000)
        #ensure there are no two obstacles with similar id
        unique_obs_id=False
        while not unique_obs_id:
            if os.path.exists(os.path.join(obs_folder,"sunnyvale_loop_obs{}.json".format(obs_id))):
                obs_id=random.randint(0,30000)
            else:
                unique_obs_id=True
        
        theta = random.uniform(-3.14, 3.14)
        length,width,height,speed=check_obs_type("PEDESTRIAN") 
        
        pos_x = random.uniform(routing["park_start"]["x"], routing["park_end"]["x"])
        pos_y = random.uniform(routing["park_start"]["y"], routing["park_end"]["y"])
        position = [pos_x, pos_y, 0.0]

        if speed == 0.0:
            trace = None
        else:
            trace_num = random.randint(TRACE_MIN, TRACE_MAX)
            trace = []
            trace.append(position)
            cur_x = pos_x
            cur_y = pos_y
            for j in range(trace_num):
                alpha = random.uniform(-3.14, 3.14)
                cur_x = cur_x + speed * math.cos(alpha)
                cur_y = cur_y + speed * math.sin(alpha)
                trace.append([cur_x, cur_y, 0.0])

        

        pedestrian_desc = generate_obs_desc(unique_obs_id, position, theta, length, width, height, speed, 1.0, "PEDESTRIAN", trace)

        pedestrian_desc_file = os.path.join(obs_folder, "sunnyvale_loop_obs{}.json".format(obs_id))

        with open(pedestrian_desc_file, 'w') as f:
            json.dump(pedestrian_desc, f)

        scenario.append([obs_id, theta, length, width, height, speed, "PEDESTRIAN"])

def runScenario(parking_obs_num, straight_obs_num, pedestrian_obs_num, record_name):
    #to start with a fresh set of obstacles for the current scnerio
    os.system("rm -f /apollo/modules/tools/perception/obstacles/*")
    global diversity_counter
    diversity_counter={"PV":0,"SV":0,"P":0}
    global scenario
    scenario=[]

    
    generate_park_obs(parking_obs_num)
    generate_straight_obs(straight_obs_num)
    generate_pedestrian_obs(pedestrian_obs_num)
    obs_num = parking_obs_num + straight_obs_num + pedestrian_obs_num

    failed=True
    num_runs=0
    while failed:
        #if scenario has been restarted x times, restart the moodules and sim control 
        if num_runs % 10 == 0 and num_runs != 0:
            os.system("bash /apollo/scripts/bootstrap.sh stop")
            time.sleep(10)
            os.system("bash /apollo/scripts/bootstrap.sh start")
            time.sleep(10)
            os.system("bash /apollo/automation/auxiliary/modules/start_modules.sh")
            time.sleep(10)
            os.system("source /apollo/cyber/setup.bash")
            time.sleep(2)
            print("attempted %s run" % num_runs)

        # ------- sending adc routing --------
        ego_routing_file = os.path.join(config_folder, map, 'ego_routing_config.json')

        # ------- running the scneario -------
        scenario_player_cmd='bazel run --experimental_ui_limit_console_output=1 //automation/scenario_player:run_auto -- -f '+ ego_routing_file + ' -o '+ record_name
        scenario_player_output = subprocess.check_output(scenario_player_cmd, shell=True)
        scenario_player_output = str(scenario_player_output)[2:-3]
        num_runs = num_runs+1
        #print(scenario_player_output)
        #if the adc didn't move or the adc was travelling outside the map boundaries, then re-run scenrio with new routing info  
        if scenario_player_output == 'None':
            continue
        scenario_player_output = scenario_player_output.split('\\n')
        min_distance = eval(scenario_player_output[0])
        #the return number of obstacles must match the ones in the individual 
        if len(min_distance) != obs_num:
            continue
        else:
            failed=False
    #scenario run successfully
    sim_time = float(scenario_player_output[4])*num_runs
    orcle_time = float(scenario_player_output[5])*num_runs
    min_distance, fastAccl_min, hardBrake_min = runOracles(scenario_player_output, record_name, scenario)
    return min_distance, fastAccl_min, hardBrake_min, sim_time, orcle_time, num_runs
   

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-m', '--map',
        help='The map chosen by the scenario generator',
        type=str
    )
    args = parser.parse_args()

    if args.map:
        map = args.map
    
    load_config(map)

    PARKING_OBS_MAX = 3
    PARKING_OBS_MIN = 0
    STRAIGHT_OBS_MAX = 3
    STRAIGHT_OBS_MIN = 1
    PEDESTRIAN_OBS_MAX = 5
    PEDESTRIAN_OBS_MIN = 1
    ETIME=43200 # execution time end (in seconds) after 12 hours 

    #store features output and evolution output
    labels="record_name,c_x,c_y,c_type,adc_heading,adc_speed,obs_id,obs_heading,obs_speed,obs_type,obs_len,obs_wid,obs_height,"\
    "speeding_x,speeding_y,speeding_value,speeding_duration,speeding_heading,lanes_speed_limit,uslc_x,uslc_y,uslc_duration,uslc_heading,"\
    "fastAccl_x,fastAccl_y,fastAccl_value,fastAccl_duration,fastAccl_heading,hardBrake_x,hardBrake_y,hardBrake_value,hardBrake_duration,hardBrake_heading,"\
    "c_counter,speeding_counter,uslc_counter,fastAccl_counter,hardBrake_counter,totalV\n"
    with open(os.path.join(dest,features_file),'w') as ffile:
      ffile.write(labels)
    labels="RecordName,ObsNum,P,B,V,AVG_OBS2ADC_Distance,Speed_Below_Limit,ADC2LaneBound_Distance,FastAccl,HardBrake\n"
    with open(os.path.join(dest,ga_file),'a+') as gfile:
      gfile.write(labels)
    labels="RecordName,Simulation,Oracles,MISC,E2E,RetryNo\n"
    with open(os.path.join(dest,timer_file),'a+') as tfile:
        tfile.write(labels)

    os.system("rm -rf /apollo/automation/grading_metrics/Safety_Violations/*")
    print("Start of Random Generation:")
    start_time=time.time()
    scenario_counter=1

    while (time.time()-start_time) <= ETIME :
        e2e_time=time.time()
        record_name="Scenario{}".format(scenario_counter)

        #generate number of obstacles per current scenario 
        parking_obs_num = random.randint(PARKING_OBS_MIN, PARKING_OBS_MAX)
        straight_obs_num = random.random(STRAIGHT_OBS_MIN, STRAIGHT_OBS_MAX)
        pedestrian_obs_num = random.random(PEDESTRIAN_OBS_MIN, PEDESTRIAN_OBS_MAX)

        #populate the description files of obstacles and run scenario
        min_distance,fastAccl_min,hardBrake_min,sim_time,orcle_time,num_runs=runScenario(parking_obs_num, straight_obs_num, pedestrian_obs_num, record_name)
        obs_num = parking_obs_num + straight_obs_num + pedestrian_obs_num
    
        sum=0
        for obs_id in min_distance:
            obs_min_dist=min_distance[obs_id]    
            sum+=obs_min_dist
        with open(os.path.join(dest,ga_file),'a+') as gfile:
            gfile.write("%s,%s,%s,%s,%s,%s,%s,%s\n"
            % (record_name,obs_num,diversity_counter["PV"],diversity_counter["SV"],diversity_counter["P"],sum/obs_num,fastAccl_min,hardBrake_min))
        e2e_time=time.time()-e2e_time
        misc_time=e2e_time-sim_time-orcle_time
        with open(os.path.join(dest,timer_file),'a+') as tfile:
            tfile.write("{},{:.2f},{:.2f},{:.2f},{:.2f},{}\n".format(record_name,sim_time,orcle_time,misc_time,e2e_time,num_runs))
        scenario_counter+=1
    print("-- End of (successful) evolution --")
     
    # ------- Final Results -------
    end_time = time.time()
    print("-- Execution Time: %.2f  seconds --\n" % (end_time-start_time))
    
    

if __name__ == '__mani__':
    main()