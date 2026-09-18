#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# license removed for brevity
import rospy
from std_msgs.msg import String,Int8,Bool
import json
import requests
from autoware_msgs.msg import SiteStatus
from autoware_msgs.msg import SitePoseArray,SitePose

from xml.dom.minidom import parse
import xml.dom.minidom
import time

# 发送字典
post_dict = {'carNO': '11', 'carType': '1', 'prestation': '','curStation':'','nextstation':''  }
# post_dict = {'carNO': '11', 'carType': '1', 'curStation': '2','nextstation':'2' }

# # 发送json
# post_json = json.dumps(post_dict)  
# print post_dict                    // 单引号: {'curStation': '2', 'carNO': '11', 'carType': '1', 'nextstation': '2'}
# print post_json                    // 双引号: {"curStation": "2", "carNO": "11", "carType": "1", "nextstation": "2"}
# 编解码: https://blog.51cto.com/u_15290914/3022962
pub_goal=''
pub_on =''

site_poses = []                           # 站点数据
task_list = {}
old_time_ = ''
current_site_ = -1
next_site_ = -1
data_flag = False
new_goal = False
site_names = []
pose_file = ''
server_url = ''
charge_site = ''
end_site = ''
dev_id=''

def dictFromXml(path):
    global site_poses
    DOMTree = xml.dom.minidom.parse(path)
    collection = DOMTree.documentElement

    testtables = collection.getElementsByTagName("testtable")
    rows = collection.getElementsByTagName("row")
    
    for row in rows:
        site_pose = {}
        site_pose['name'] = row.getElementsByTagName('col')[0].childNodes[0].data.encode("utf-8")
        if(site_pose['name'] == end_site):
            site_pose['attr'] = '0'
        elif(site_pose['name'] == charge_site):
            site_pose['attr'] = '2'
        else:
            site_pose['attr'] = '1'
            
        site_pose['x'] = row.getElementsByTagName('col')[1].childNodes[0].data.encode("utf-8")
        site_pose['y'] = row.getElementsByTagName('col')[2].childNodes[0].data.encode("utf-8")
        site_pose['z'] = row.getElementsByTagName('col')[3].childNodes[0].data.encode("utf-8")
        site_pose['xorie'] = row.getElementsByTagName('orie_X')[0].childNodes[0].data.encode("utf-8")
        site_pose['yorie'] = row.getElementsByTagName('orie_Y')[0].childNodes[0].data.encode("utf-8")
        site_pose['zorie'] = row.getElementsByTagName('orie_Z')[0].childNodes[0].data.encode("utf-8")
        site_pose['worie'] = row.getElementsByTagName('orie_W')[0].childNodes[0].data.encode("utf-8")
        site_poses.append(site_pose)

def dictFromJson(path):# 数据路径
    # 读取文件数据
    with open(path, 'r')  as f:
        row_data = json.load(f)
    
    return row_data
            
def plannerStatusCallback(msg):
    global next_site_,current_site_,data_flag,site_names
    
    data_flag = True               # 规划有话题发布才会发布路径数据
    
    current_site_ = msg.current_index
    next_site_ = msg.next_index
    
    # print("site status: ",msg.max_index,msg.pre_index,msg.current_index,msg.next_index)
    # for name in site_names:
    #     print(name)
    
    if(len(site_names) == 0):
        return
    
    
    status_ = msg
    post_dict['carType'] = str(msg.status)
        
    if(msg.pre_index == -1):
        post_dict['prestation'] = ''
    else:
        post_dict['prestation'] = str(site_names[msg.pre_index])
        
    if(msg.current_index == -1):
        post_dict['curStation'] = ''
    else:
        post_dict['curStation'] = str(site_names[msg.current_index])
        
    if(msg.next_index == -1):
        post_dict['nextstation'] = ''
    else:
        post_dict['nextstation'] = str(site_names[msg.next_index])
        

def pub_goal_site(goal_info):
    global site_poses,current_site_,pub_goal,data_flag,new_goal,site_names
    locals = goal_info
    
    if(data_flag == False):
        return 
    
    # if(next_site_ == -1 and current_site_ == -1):
        
    multi_site = SitePoseArray()   
    multi_site.header.stamp = rospy.get_rostime()
    multi_site.header.frame_id = 'map'  
    
    if(site_names == [] or new_goal == True):
        # 把充电与站点任务添加到最后
        charge_site_info = {"location":charge_site.decode('utf-8'),"orderStatue":[{"orderNO": "98","orderType": "0"}],"outTime": "1"}
        end_site_info = {"location":end_site.decode('utf-8'),"orderStatue":[{"orderNO": "99","orderType": "0"}],"outTime": "1"}
        locals.append(end_site_info)
        locals.append(charge_site_info)
        site_names = []
        
        for local in locals:
            for pose in site_poses:
                name_ = local['location'].encode("utf-8")
                if(name_ == pose['name']):
                    site_ = SitePose()
                    site_.site_name = pose['name']
                    site_.time = int(local['outTime']) * 60.
                    site_.Attributes = int(pose['attr'])
                    site_.pose.position.x = float(pose['x'])
                    site_.pose.position.y = float(pose['y'])
                    site_.pose.position.z = float(pose['z'])
                    site_.pose.orientation.x = float(pose['xorie'])
                    site_.pose.orientation.y = float(pose['yorie'])
                    site_.pose.orientation.z = float(pose['zorie'])
                    site_.pose.orientation.w = float(pose['worie'])
                    multi_site.sites.append(site_)
                    
                    site_names.append(pose['name'])
                    break
    # 添加充电，end
    if(new_goal == True ):
        pub_goal.publish(multi_site)
        
    if(next_site_ >= 0):
        new_goal = False

def param_data(msg):
    global old_time_,current_site_,task_list,new_goal,pub_on
    try:
        if(msg['Success'] == True):
            if(msg['ErrorCode'] == 0):
                # print(msg['Data'])
                
                num_ = int(msg['Data']['locationsNO'].encode("utf-8"))
                len_ = len(msg['Data']['locations'])
                if((len_ == num_) and num_ > 0):
                    new_time_ = msg['Data']['dateTime'].encode("utf-8")
                    
                    if(old_time_ == ''):
                        old_time_ = new_time_

                    elif((new_time_ != old_time_) or (next_site_ == -1 and current_site_ == -1)):
                        old_time_ = new_time_
                        new_goal = True
                        
                    # 发布多目标点
                    pub_goal_site(msg['Data']['locations'])
        
                    # 该站点所有快递已经领取,发布使能
                    if(current_site_ != -1):
                        status_is = True
                        for loc in msg['Data']['locations']:
                            name_ = loc['location'].encode("utf-8")
                            if(site_names[current_site_] == name_):
                                for sta in loc['orderStatue']:
                                    status_is = (status_is and int(sta['orderType']))
                        # if(bool(status_is)):
                        pub_on.publish(int(status_is))
                    else:
                        status_is = False
                        pub_on.publish(int(status_is))
            

        else:
            print("获取数据失败!!!")
                    
    except Exception as e:
        print("获取服务器数据异常")
 
def post_data():
    global server_url,pose_file,dev_id
    time.sleep(40)
    # 获取所有站点位置
    xml_path = pose_file
    dictFromXml(xml_path)
        
    rate = rospy.Rate(0.2) # 10hz
    post_dict['carNO'] =  str(dev_id) #str('11')

    while not rospy.is_shutdown():
        try:
            print('--------------------')
            print("所有点:")
            for name in site_names:
                print('\t'+name)
            rospy.loginfo("send data: %s",post_dict)
            for key,value in post_dict.items():
                # print('参数：'+str(value).decode('string_escape'))
                print(key+': '+value)
            
            # # server_url='http://www.bjcmxd.cn/POSAPI/updateCarInfoBJCM?'
            req = requests.post(server_url, data=post_dict)  #.content
            if(req.status_code == 200):
                msg = json.loads(req.text)          # 接受到的网络数据unicode转为dict(json格式也是dict)
                # print(msg)
                rospy.loginfo("recv data: %s",msg)
                param_data(msg)
                
            
            # 模拟服务器数据发布
            # json_path = "/home/ant/work/autoware.ai/src/remote_communication/conf/param.json"
            # req = dictFromJson(json_path)
            # param_data(req)
            
            
            rate.sleep()
        except Exception as e:
                print("程序异常退出")
 

if __name__ == '__main__':
    try:
        rospy.init_node('remote_interact', anonymous=False)
        pub_goal = rospy.Publisher('multi_goal', SitePoseArray, queue_size=1)
        pub_on = rospy.Publisher('goal_on', Bool, queue_size=1)
        rospy.Subscriber("planner_status", SiteStatus, plannerStatusCallback)
        
        dev_id = rospy.get_param("~dev_id",'11')
        pose_file = rospy.get_param("~pose_file",'pose.xml')
        server_url = rospy.get_param("~server_url",'pose.xml')
        charge_site = rospy.get_param("~charge_site",'充电站').encode("utf-8")
        end_site = rospy.get_param("~end_site",'终点站').encode("utf-8")
        post_data()
    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException")
        pass