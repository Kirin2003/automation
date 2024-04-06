import os
import time
import json
import subprocess
import ast

dest="/apollo/automation/scenario_generator"
report_name="mut_features.csv"
#report_name="immut_features.csv"
#report_name="rand_features.csv"

def runOracles(scenario_player_output,record_name,scenario):
  collision=eval(scenario_player_output[3]) # collision info
  accl=eval(scenario_player_output[1]) # maximum accl of adc 
  hardbrake=eval(scenario_player_output[2]) # maximum braking 
  result=""
  
  #collision values
  min_distance=ast.literal_eval(scenario_player_output[0]) # a list of obs_id and their minimum dist to the adc, this list is needed for ga 
  c_type={"rear":0, "front":1, "right":2, "left":3}
  if collision == None:
    c_counter=0
    c_x=0
    c_y=0
    c_type=0
    adc_heading=0
    adc_speed=0
    obs_id=0
    obs_speed=0
    obs_heading=0
    obs_type=0
    obs_len=0
    obs_wid=0
    obs_height=0
  else:
    c_counter=1
    c_x=collision[1]
    c_y=collision[2]
    c_type=c_type[collision[3][0]]
    adc_heading=collision[4]
    adc_speed=collision[5]
    obs_id=collision[0]
    obs_speed=collision[6]
    obs_heading=collision[7]
    for obs in scenario:
      if obs[0]==obs_id:
        obs_type=obs[6]
        obs_len=obs[2]
        obs_wid=obs[3]
        obs_height=obs[4]
  temp="{},{},{},{},{},{},{},{},{},{},{},{},{}".format(record_name,c_x,c_y,c_type,adc_heading,adc_speed,obs_id,obs_heading,obs_speed,obs_type,obs_len,obs_wid,obs_height)
  result=temp

  #fast_accl values
  fastAccl_min=accl[0] #only value needed for ga 
  if accl[1] == None:
    fastAccl_counter=0
    fastAccl_value=0
    fastAccl_x=0
    fastAccl_y=0
    fastAccl_duration=0
    fastAccl_heading=0
  else:
    fastAccl_counter=1
    fastAccl_value=accl[1]
    fastAccl_x=accl[2][0]
    fastAccl_y=accl[2][1]
    fastAccl_duration=accl[4]
    fastAccl_heading=accl[3]
  temp=",{},{},{},{},{}".format(fastAccl_x,fastAccl_y,fastAccl_value,fastAccl_duration,fastAccl_heading)
  result+=temp

  #hard_braking values
  hardBrake_min=hardbrake[0] #only value needed for ga 
  if hardbrake[1] == None:
    hardBrake_counter=0
    hardBrake_value=0
    hardBrake_x=0
    hardBrake_y=0
    hardBrake_duration=0
    hardBrake_heading=0
  else:
    hardBrake_counter=1
    hardBrake_value=hardbrake[1]
    hardBrake_x=hardbrake[2][0]
    hardBrake_y=hardbrake[2][1]
    hardBrake_duration=hardbrake[4]
    hardBrake_heading=hardbrake[3]
  temp=",{},{},{},{},{}".format(hardBrake_x,hardBrake_y,hardBrake_value,hardBrake_duration,hardBrake_heading)
  result+=temp
  
  total_vio=c_counter+fastAccl_counter+hardBrake_counter
  viol=",{},{},{},{}\n".format(c_counter,fastAccl_counter,hardBrake_counter,total_vio)
  result+=viol
  
  #if there's no violations, delete record file to save space
  if total_vio==0:
      del_cmd="rm /apollo/automation/temp_record/{}.00000".format(record_name)
      os.system(del_cmd)
  else:
    with open(os.path.join(dest,report_name),'a+') as file:
        file.write(result)
    
  return min_distance,fastAccl_min,hardBrake_min

