import csv
import subprocess
import os
import glob
import shutil
import math
import numpy as np
import cv2
import yaml


#yaml文件处理
def changeTRYamlConfig(yamlPath, key, value):
    # 修改yaml配置
    f = open(yamlPath, 'r', encoding='utf-8')
    result = f.read()
    content = yaml.safe_load(result)
    content[key]['data'] = value

    w=open(yamlPath, 'w', encoding='utf-8')
    yaml.safe_dump(content,w)
    f.close()
    w.close()

#yaml文件处理
def changeYamlConfig(path, key, trans,rot):
    with open(path, 'r', encoding='utf-8') as f:
        lines = []  # 创建了一个空列表，里面没有元素
        for line in f.readlines():
            if line != '\n':
                lines.append(line)
        f.close()
    with open(path, 'w', encoding='utf-8') as f:
        flag = 0
        for num,line in enumerate(lines):
            if key in line:
                leftstr = line.split(":")[0]
                if "extrinsicRotation:" in lines[num - 4]:
                    newline = "{0}: {1}".format(leftstr, rot)
                elif "extrinsicTranslation:" in lines[num - 4]:
                    newline = "{0}: {1}".format(leftstr, trans)
                else:
                    f.write('%s\n' % line)
                    continue
                line = newline
                f.write('%s\n' % line)
                flag = 1
            else:
                f.write('%s' % line)
        f.close()
        return flag


def copy_and_clear_result(path,newpath,filename):
    isExist = os.path.isdir(newpath)
    # if isExist:
    #     shutil.rmtree(newpath)
    if not isExist:
        os.makedirs(newpath)

    for file in glob.glob(os.path.join(path , "*.csv")):
        shutil.copyfile(os.path.join(path, file), os.path.join(newpath, filename))
        os.remove(file)


def generate_random_3Dvector(length):
    random_vector = np.random.rand(3) - 0.5 #随机三维向量
    norm = np.linalg.norm(random_vector)
    return random_vector/norm*length

def simNoisyTranslation(vector,error):
    vector = vector +generate_random_3Dvector(error)
    return vector.tolist()


def simNoisyRotation(rotation,angle):
    Rotation=np.zeros((3, 3))
    vector = generate_random_3Dvector(1)*angle
    cv2.Rodrigues(vector,Rotation)
    rotation = np.matmul(Rotation, rotation)
    rotation = np.reshape(rotation,[9,])
    return rotation.tolist()

def append_csv(csv_path, append_word):
    with open(csv_path,'a+') as f:
        csv_write = csv.writer(f)
        csv_write.writerow(append_word)


def change_yaml():
    #修改yaml
    yamlpath = '/home/wuhan2020/yqc/catkin_ws/src/VINS-Mono/config/euroc/euroc_config.yaml'
    rotError = 0.5 * math.pi / 180
    transError = 0.02
    rotation = np.array([[0.0148655429818, -0.999880929698, 0.00414029679422],
           [0.999557249008, 0.0149672133247, 0.025715529948],
           [-0.0257744366974, 0.00375618835797, 0.999660727178]])
    translation = np.array([-0.0216401454975,-0.064676986768, 0.00981073058949])

    rot_with_noise = simNoisyRotation(rotation,rotError)
    trans_with_noise = simNoisyTranslation(translation,transError)
    changeYamlConfig(yamlpath, "data", trans_with_noise, rot_with_noise)

if __name__ == '__main__':
    type = "noiseEsti"   # noise baseline  noiseEsti
    csv_path="/home/wuhan2020/yqc/vins_mono_output/vins_result_no_loop.csv"
    data = ["MH_01_easy","MH_02_easy","MH_03_medium","MH_04_difficult","MH_05_difficult",
            "V1_01_easy","V1_02_medium","V1_03_difficult",
            "V2_01_easy","V2_02_medium","V2_03_difficult" ]
    data_name = ["MH_01","MH_02","MH_03","MH_04","MH_05",
                 "V1_01","V1_02","V1_03",
                 "V2_01","V2_02","V2_03" ]
    out_data_path= '/home/wuhan2020/yqc/vins_mono_output'
    sortd_data_path = '/home/wuhan2020/yqc/rpg_trajectory_evaluation/results/euroc_vins/laptop/'+type+'/'


    #控制循环
    for round in range(0,50):
        if type == 'noise' or type == 'noiseEsti':
            change_yaml()     #注意设置一开始的euroc_yaml
        # for data_num,inputdata in enumerate(data):
            # bagfile_path = "/home/wuhan2020/yqc/catkin_ws/dataset/"+inputdata+".bag"
            # os.system("roslaunch vins_estimator euroc.launch bag_file:="+bagfile_path)
            # copy_and_clear_result(out_data_path,sortd_data_path+'laptop_'+type+"_"+data_name[data_num],"stamped_traj_estimate"+str(round)+".txt")


            # source ../../devel/setup.bash

            # os.system("rosbag play "+bagfile_path)

