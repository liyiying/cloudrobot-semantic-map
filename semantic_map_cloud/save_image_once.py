import subprocess
import sys, os
import time
import datetime

def command_run(command,timeout=1):
    proc = subprocess.Popen(command,bufsize=0,stdout=subprocess.PIPE,stderr=subprocess.PIPE,shell=True)
    time.sleep(timeout)
    proc.kill()

def save_image_once():
    #time.sleep(5)
    #command = "rosrun image_view image_saver image:=/camera/rgb/image_raw"
    command = "rosservice call /image_saver/save"
    command_run(command,1)
    #print('ok')
    time.sleep(5)
    pre_image_file = os.path.join(os.path.dirname(sys.argv[0]), 'foo.jpg')
    lat_image_file = os.path.join(os.path.dirname(sys.argv[0]), str(datetime.datetime.now()).replace(" ","_")+'.jpg')
   
    while os.path.exists(pre_image_file) == False:
        time.sleep(0.1)
    os.rename(pre_image_file, lat_image_file)
    #print("OK 1")
    #print(lat_image_file)
    return lat_image_file

if __name__=="__main__":
    save_image_once()
