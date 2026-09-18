/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "op_global_planner_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"

namespace GlobalPlanningNS
{

    GlobalPlanner::GlobalPlanner()
    {
        m_pCurrGoal = 0;
        m_iCurrentGoalIndex = 0;
        m_bKmlMap = false;
        m_bFirstStart = false;
        m_GlobalPathID = 1;
        m_bStartPlan = false;
        m_next_goal_on = false;
        low_battery_flag = false;
        bMakeNewPlan = false;
        m_bEndSiteFlag = false;
        m_SiteSatus.status = 1;

        UtilityHNS::UtilityH::GetTickCount(m_ReplnningTimer);
        nh.getParam("/op_global_planner/lowBatteryThreshold", m_LowBatteryThreshold);

        nh.getParam("/op_global_planner/pathDensity", m_params.pathDensity);
        nh.getParam("/op_global_planner/enableSmoothing", m_params.bEnableSmoothing);
        nh.getParam("/op_global_planner/enableLaneChange", m_params.bEnableLaneChange);
        nh.getParam("/op_global_planner/enableRvizInput", m_params.bEnableRvizInput);
        nh.getParam("/op_global_planner/enableReplan", m_params.bEnableReplanning);
        nh.getParam("/op_global_planner/onlyReplan", m_params.bOnlyReplanning);
        nh.getParam("/op_global_planner/enableDynamicMapUpdate", m_params.bEnableDynamicMapUpdate);
        nh.getParam("/op_global_planner/mapFileName", m_params.KmlMapPath);

        int iSource = 0;
        nh.getParam("/op_global_planner/mapSource", iSource);
        if (iSource == 0)
            m_params.mapSource = PlannerHNS::MAP_AUTOWARE;
        else if (iSource == 1)
            m_params.mapSource = PlannerHNS::MAP_FOLDER;
        else if (iSource == 2)
            m_params.mapSource = PlannerHNS::MAP_KML_FILE;

        tf::StampedTransform transform;
        PlannerHNS::ROSHelpers::GetTransformFromTF("map", "world", transform);
        m_OriginPos.position.x = transform.getOrigin().x();
        m_OriginPos.position.y = transform.getOrigin().y();
        m_OriginPos.position.z = transform.getOrigin().z();

        pub_Paths = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
        pub_PathsRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
        pub_MapRviz = nh.advertise<visualization_msgs::MarkerArray>("vector_map_center_lines_rviz", 1, true);
        pub_GoalsListRviz = nh.advertise<visualization_msgs::MarkerArray>("op_destinations_rviz", 1, true);
        pub_RemainingDistance = nh.advertise<std_msgs::Float32>("goal_remaining_distance", 1, true);
        pub_planner_status = nh.advertise<autoware_msgs::SiteStatus>("planner_status", 1, true);

        if (m_params.bEnableRvizInput)
        {
            sub_start_pose = nh.subscribe("/initialpose", 1, &GlobalPlanner::callbackGetStartPose, this);
            sub_goal_pose = nh.subscribe("move_base_simple/goal", 1, &GlobalPlanner::callbackGetGoalPose, this);
        }
        else
        {
            LoadSimulationData();
        }
        sub_batter_state = nh.subscribe("battery_state", 10, &GlobalPlanner::callbackGetBatteryState, this);
        sub_multi_goal_pose = nh.subscribe("multi_goal", 1, &GlobalPlanner::callbackGetMultiGoalPose, this);
        sub_goal_on = nh.subscribe("goal_on", 1, &GlobalPlanner::callbackGetGoalOn, this);

        sub_current_pose = nh.subscribe("/current_pose", 10, &GlobalPlanner::callbackGetCurrentPose, this);

        int bVelSource = 1;
        nh.getParam("/op_global_planner/velocitySource", bVelSource);
        if (bVelSource == 0)
            sub_robot_odom = nh.subscribe("/odom", 10, &GlobalPlanner::callbackGetRobotOdom, this);
        else if (bVelSource == 1)
            sub_current_velocity = nh.subscribe("/current_velocity", 10, &GlobalPlanner::callbackGetVehicleStatus, this);
        else if (bVelSource == 2)
            sub_can_info = nh.subscribe("/can_info", 10, &GlobalPlanner::callbackGetCANInfo, this);

        if (m_params.bEnableDynamicMapUpdate)
            sub_road_status_occupancy = nh.subscribe<>("/occupancy_road_status", 1, &GlobalPlanner::callbackGetRoadStatusOccupancyGrid, this);

        // Mapping Section
        sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &GlobalPlanner::callbackGetVMLanes, this);
        sub_points = nh.subscribe("/vector_map_info/point", 1, &GlobalPlanner::callbackGetVMPoints, this);
        sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &GlobalPlanner::callbackGetVMdtLanes, this);
        sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &GlobalPlanner::callbackGetVMIntersections, this);
        sup_area = nh.subscribe("/vector_map_info/area", 1, &GlobalPlanner::callbackGetVMAreas, this);
        sub_lines = nh.subscribe("/vector_map_info/line", 1, &GlobalPlanner::callbackGetVMLines, this);
        sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &GlobalPlanner::callbackGetVMStopLines, this);
        sub_signals = nh.subscribe("/vector_map_info/signal", 1, &GlobalPlanner::callbackGetVMSignal, this);
        sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &GlobalPlanner::callbackGetVMVectors, this);
        sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &GlobalPlanner::callbackGetVMCurbs, this);
        sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &GlobalPlanner::callbackGetVMRoadEdges, this);
        sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &GlobalPlanner::callbackGetVMWayAreas, this);
        sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &GlobalPlanner::callbackGetVMCrossWalks, this);
        sub_nodes = nh.subscribe("/vector_map_info/node", 1, &GlobalPlanner::callbackGetVMNodes, this);
    }

    GlobalPlanner::~GlobalPlanner()
    {
        if (m_params.bEnableRvizInput)
            SaveSimulationData();
    }

    void GlobalPlanner::callbackGetRoadStatusOccupancyGrid(const nav_msgs::OccupancyGridConstPtr &msg)
    {
        //  std::cout << "Occupancy Grid Origin (" << msg->info.origin.position.x << ", " << msg->info.origin.position.x << ") , " << msg->header.frame_id << ", Res: " << msg->info.resolution <<  std::endl;

        m_GridMapIntType.clear();

        // std::cout << "Found Map Data: Zero " <<  std::endl;
        for (unsigned int i = 0; i < msg->data.size(); i++)
        {
            if ((int8_t)msg->data.at(i) == 0)
                m_GridMapIntType.push_back(0);
            else if ((int8_t)msg->data.at(i) == 50)
                m_GridMapIntType.push_back(75);
            else if ((int8_t)msg->data.at(i) == 100)
                m_GridMapIntType.push_back(255);
            else
                m_GridMapIntType.push_back(128);

            // std::cout << msg->data.at(i) << ",";
        }
        // std::cout << std::endl << "--------------------------------------------------------" << std::endl;

        // std::cout << "Found Map Data: Zero : " << m_GridMapIntType.size() <<  std::endl;
        PlannerHNS::WayPoint center(msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z, tf::getYaw(msg->info.origin.orientation));
        PlannerHNS::OccupancyToGridMap grid(msg->info.width, msg->info.height, msg->info.resolution, center);
        std::vector<PlannerHNS::WayPoint *> modified_nodes;
        timespec t;
        UtilityHNS::UtilityH::GetTickCount(t);
        PlannerHNS::MappingHelpers::UpdateMapWithOccupancyGrid(grid, m_GridMapIntType, m_Map, modified_nodes);
        m_ModifiedMapItemsTimes.push_back(std::make_pair(modified_nodes, t));

        visualization_msgs::MarkerArray map_marker_array;
        PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);

        //  visualization_msgs::Marker mkr = PlannerHNS::ROSHelpers::CreateGenMarker(center.pos.x, center.pos.y, center.pos.z, 0, 0,0,1,0.5, 1000, "TestCenter", visualization_msgs::Marker::SPHERE);
        //
        //  map_marker_array.markers.push_back(mkr);

        pub_MapRviz.publish(map_marker_array);
    }

    void GlobalPlanner::ClearOldCostFromMap()
    {
        for (int i = 0; i < (int)m_ModifiedMapItemsTimes.size(); i++)
        {
            if (UtilityHNS::UtilityH::GetTimeDiffNow(m_ModifiedMapItemsTimes.at(i).second) > CLEAR_COSTS_TIME)
            {
                for (unsigned int j = 0; j < m_ModifiedMapItemsTimes.at(i).first.size(); j++)
                {
                    for (unsigned int i_action = 0; i_action < m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.size(); i_action++)
                    {
                        if (m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.at(i_action).first == PlannerHNS::FORWARD_ACTION)
                        {
                            m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.at(i_action).second = 0;
                        }
                    }
                }

                m_ModifiedMapItemsTimes.erase(m_ModifiedMapItemsTimes.begin() + i);
                i--;
            }
        }
    }

    void GlobalPlanner::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        PlannerHNS::WayPoint wp = PlannerHNS::WayPoint(msg->pose.position.x + m_OriginPos.position.x, msg->pose.position.y + m_OriginPos.position.y, msg->pose.position.z + m_OriginPos.position.z, tf::getYaw(msg->pose.orientation));
        m_GoalsPos.push_back(wp);
        ROS_INFO("Received Goal Pose");
    }

    void GlobalPlanner::callbackGetGoalOn(const std_msgs::BoolConstPtr &msg)
    {
        m_next_goal_on = msg->data;
    }

    void GlobalPlanner::callbackGetBatteryState(const sensor_msgs::BatteryStateConstPtr &msg)
    {
        static int low_count = 0;
        static int normal_count = 0;
        if (msg->percentage < m_LowBatteryThreshold){    // 电量低于百分之30
            if(low_count++ >= 1000){    // 连续读取1000次都是低电量执行
                low_battery_flag = true;

                m_GoalsPos.clear();
                m_iCurrentGoalIndex = 0;


                m_GoalsPos.push_back(m_ChargingSite);
                bMakeNewPlan = true;
                m_params.bEnableReplanning = false; // 不重复规划

            }
            normal_count = 0;
        }else{
            if(low_battery_flag == true){
                if(normal_count++ >= 1000){    // 连续读取1000次都是低电量执行
                    low_battery_flag = false;
                    m_params.bEnableReplanning = true;
                }
            }
            low_count = 0;
        }
    }

    void GlobalPlanner::callbackGetMultiGoalPose(const autoware_msgs::SitePoseArrayConstPtr &msg)
    {
        // bool repoint_flag = true;
        if(m_params.bEnableReplanning  == false)
            return;

        if(low_battery_flag == false){      // 低电量不允许重新规划
            m_GoalsPos.clear();
            m_iCurrentGoalIndex = 0;
            m_SiteArray.sites.clear();
            m_bEndSiteFlag = false;
            m_SiteArray.sites = msg->sites;
            for(int i=0;i < msg->sites.size();i++){
                PlannerHNS::WayPoint wp = PlannerHNS::WayPoint(msg->sites[i].pose.position.x + m_OriginPos.position.x, msg->sites[i].pose.position.y + m_OriginPos.position.y, msg->sites[i].pose.position.z + m_OriginPos.position.z, tf::getYaw(msg->sites[i].pose.orientation));
                if(m_SiteArray.sites[i].Attributes == autoware_msgs::SitePose::GARAGE__SITE){
                    m_ChargingSite = wp;
                    continue;
                }else if(m_SiteArray.sites[i].Attributes == autoware_msgs::SitePose::START_SITE){
                    m_EndSite = wp; 
                    m_bEndSiteFlag = true;
                    continue;
                }else{
                    // if (m_params.bEnableReplanning && repoint_flag)
                    // {
                    //     repoint_flag = false;
                    //     m_GoalsPos.push_back(wp);
                    // }
                    m_GoalsPos.push_back(wp);
                    m_iCurrentGoalIndex=0;
                    bMakeNewPlan = true;      // 接受到新目标,重新规划

                }
            }

            if (m_bEndSiteFlag)
            {
                m_GoalsPos.push_back(m_EndSite);
            }
        }
    }

    void GlobalPlanner::callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
    {
        m_CurrentPose = PlannerHNS::WayPoint(msg->pose.pose.position.x + m_OriginPos.position.x, msg->pose.pose.position.y + m_OriginPos.position.y, msg->pose.pose.position.z + m_OriginPos.position.z, tf::getYaw(msg->pose.pose.orientation));
        ROS_INFO("Received Start pose");
    }

    void GlobalPlanner::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        m_CurrentPose = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
    }

    void GlobalPlanner::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr &msg)
    {
        m_VehicleState.speed = msg->twist.twist.linear.x;
        m_CurrentPose.v = m_VehicleState.speed;
        if (fabs(msg->twist.twist.linear.x) > 0.25)
            m_VehicleState.steer += atan(2.7 * msg->twist.twist.angular.z / msg->twist.twist.linear.x);
    }

    void GlobalPlanner::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr &msg)
    {
        m_VehicleState.speed = msg->twist.linear.x;
        m_CurrentPose.v = m_VehicleState.speed;
        if (fabs(msg->twist.linear.x) > 0.25)
            m_VehicleState.steer = atan(2.7 * msg->twist.angular.z / msg->twist.linear.x);
        UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
    }

    void GlobalPlanner::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
    {
        m_VehicleState.speed = msg->speed / 3.6;
        m_CurrentPose.v = m_VehicleState.speed;
        m_VehicleState.steer = msg->angle * 0.45 / 660;
        UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
    }

    bool GlobalPlanner::GenerateGlobalPlan(PlannerHNS::WayPoint &startPoint, PlannerHNS::WayPoint &goalPoint, std::vector<std::vector<PlannerHNS::WayPoint>> &generatedTotalPaths)
    {
        std::vector<int> predefinedLanesIds;
        double ret = 0;

        ret = m_PlannerH.PlanUsingDP(startPoint, goalPoint, MAX_GLOBAL_PLAN_DISTANCE, m_params.bEnableLaneChange, predefinedLanesIds, m_Map, generatedTotalPaths);

        if (ret == 0)
        {
            std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString()
                      << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
            return false;
        }

        if (generatedTotalPaths.size() > 0 && generatedTotalPaths.at(0).size() > 0)
        {
            if (m_params.bEnableSmoothing)
            {
                for (unsigned int i = 0; i < generatedTotalPaths.size(); i++)
                {
                    PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_params.pathDensity);
                    PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.35, 0.01);
                }
            }
            m_GlobalPathID = 1;
            for (unsigned int i = 0; i < generatedTotalPaths.size(); i++)
            {
                PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
                if (m_GlobalPathID > 10000)
                    m_GlobalPathID = 1;

                for (unsigned int j = 0; j < generatedTotalPaths.at(i).size(); j++)
                    generatedTotalPaths.at(i).at(j).gid = m_GlobalPathID;

                m_GlobalPathID++;

                std::cout << "New DP Path -> " << generatedTotalPaths.at(i).size() << std::endl;
            }
            return true;
        }
        else
        {
            std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString() << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
        }
        return false;
    }

    void GlobalPlanner::VisualizeAndSend(const std::vector<std::vector<PlannerHNS::WayPoint>> generatedTotalPaths)
    {
        autoware_msgs::LaneArray lane_array;
        visualization_msgs::MarkerArray pathsToVisualize;

        for (unsigned int i = 0; i < generatedTotalPaths.size(); i++)
        {
            autoware_msgs::Lane lane;
            PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(generatedTotalPaths.at(i), lane);
            lane_array.lanes.push_back(lane);
        }

        std_msgs::ColorRGBA total_color;
        total_color.r = 0;
        total_color.g = 0.7;
        total_color.b = 1.0;
        total_color.a = 0.9;
        PlannerHNS::ROSHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
        PlannerHNS::ROSHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
        PlannerHNS::ROSHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);
        pub_PathsRviz.publish(pathsToVisualize);
        if ((m_bFirstStart && m_params.bEnableHMI) || !m_params.bEnableHMI)
            pub_Paths.publish(lane_array);

        for (unsigned int i = 0; i < generatedTotalPaths.size(); i++)
        {
            std::ostringstream str_out;
            str_out << UtilityHNS::UtilityH::GetHomeDirectory();
            str_out << UtilityHNS::DataRW::LoggingMainfolderName;
            str_out << UtilityHNS::DataRW::GlobalPathLogFolderName;
            str_out << "GlobalPath_";
            str_out << i;
            str_out << "_";
            PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), generatedTotalPaths.at(i));
        }
    }

    void GlobalPlanner::VisualizeDestinations(std::vector<PlannerHNS::WayPoint> &destinations, const int &iSelected)
    {
        visualization_msgs::MarkerArray goals_array;

        int num = 1;
        for (unsigned int i = 0; i < destinations.size(); i++)
        {
            // if (i == 0 && m_params.bEnableReplanning)
            // {
            //     continue;
            // }

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "HMI_Destinations";
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 10.25;
            marker.scale.y = 10.25;
            marker.scale.z = 10.25;
            marker.color.a = 0.9;
            marker.id = i;
            if (i == iSelected)
            {
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0;
            }
            else
            {
                // marker.color.r = 0.2;
                // marker.color.g = 0.8;
                // marker.color.b = 0.2;

                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0;
            }
            marker.pose.position.x = destinations.at(i).pos.x;
            marker.pose.position.y = destinations.at(i).pos.y;
            marker.pose.position.z = destinations.at(i).pos.z;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(destinations.at(i).pos.a);

            std::ostringstream str_out;
            str_out << num++;
            marker.text = str_out.str();

            goals_array.markers.push_back(marker);
        }
        pub_GoalsListRviz.publish(goals_array);
    }

    void GlobalPlanner::SaveSimulationData()
    {
        std::vector<std::string> simulationDataPoints;
        std::ostringstream startStr;
        startStr << m_CurrentPose.pos.x << "," << m_CurrentPose.pos.y << "," << m_CurrentPose.pos.z << "," << m_CurrentPose.pos.a << "," << m_CurrentPose.cost << "," << 0 << ",";
        simulationDataPoints.push_back(startStr.str());

        for (unsigned int i = 0; i < m_GoalsPos.size(); i++)
        {
            std::ostringstream goalStr;
            goalStr << m_GoalsPos.at(i).pos.x << "," << m_GoalsPos.at(i).pos.y << "," << m_GoalsPos.at(i).pos.z << "," << m_GoalsPos.at(i).pos.a << "," << 0 << "," << 0 << ",destination_" << i + 1 << ",";
            simulationDataPoints.push_back(goalStr.str());
        }

        std::string header = "X,Y,Z,A,C,V,name,";

        std::ostringstream fileName;
        fileName << UtilityHNS::UtilityH::GetHomeDirectory() + UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::SimulationFolderName;
        fileName << "EgoCar.csv";
        std::ofstream f(fileName.str().c_str());

        if (f.is_open())
        {
            if (header.size() > 0)
                f << header << "\r\n";
            for (unsigned int i = 0; i < simulationDataPoints.size(); i++)
                f << simulationDataPoints.at(i) << "\r\n";
        }

        f.close();
    }

    int GlobalPlanner::LoadSimulationData()
    {
        std::ostringstream fileName;
        fileName << "EgoCar.csv";

        std::string simuDataFileName = UtilityHNS::UtilityH::GetHomeDirectory() + UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::SimulationFolderName + fileName.str();
        UtilityHNS::SimulationFileReader sfr(simuDataFileName);
        UtilityHNS::SimulationFileReader::SimulationData data;

        int nData = sfr.ReadAllData(data);
        if (nData == 0)
            return 0;

        m_CurrentPose = PlannerHNS::WayPoint(data.startPoint.x, data.startPoint.y, data.startPoint.z, data.startPoint.a);
        m_GoalsPos.push_back(PlannerHNS::WayPoint(data.goalPoint.x, data.goalPoint.y, data.goalPoint.z, data.goalPoint.a));

        for (unsigned int i = 0; i < data.simuCars.size(); i++)
        {
            m_GoalsPos.push_back(PlannerHNS::WayPoint(data.simuCars.at(i).x, data.simuCars.at(i).y, data.simuCars.at(i).z, data.simuCars.at(i).a));
        }

        return nData;
    }
    
    

    void GlobalPlanner::MainLoop()
    {
        ros::Rate loop_rate(25);
        timespec animation_timer;
        UtilityHNS::UtilityH::GetTickCount(animation_timer);
        std_msgs::Float32 remaing_dis_data;

        _timer = nh.createTimer(
            ros::Duration(15),
            [&](const ros::TimerEvent &)
            {
                m_bStartPlan = true;
                _timer.stop();
                ROS_ERROR("+++++++ 开始规划新路径! , index: %d",m_iCurrentGoalIndex);
            },
            false, false);

        while (ros::ok())
        {
            ros::spinOnce();
            // bool bMakeNewPlan = false;

            if (m_params.mapSource == PlannerHNS::MAP_KML_FILE && !m_bKmlMap)
            {
                m_bKmlMap = true;
                PlannerHNS::MappingHelpers::LoadKML(m_params.KmlMapPath, m_Map);
                visualization_msgs::MarkerArray map_marker_array;
                PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
                pub_MapRviz.publish(map_marker_array);
            }
            else if (m_params.mapSource == PlannerHNS::MAP_FOLDER && !m_bKmlMap)
            {
                m_bKmlMap = true;
                PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_params.KmlMapPath, m_Map, true);
                visualization_msgs::MarkerArray map_marker_array;
                PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);

                pub_MapRviz.publish(map_marker_array);
            }
            else if (m_params.mapSource == PlannerHNS::MAP_AUTOWARE && !m_bKmlMap)
            {
                std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;
                ;

                if (m_MapRaw.GetVersion() == 2)
                {
                    std::cout << "Map Version 2" << endl;
                    m_bKmlMap = true;
                    PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
                                                                                     m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list, m_MapRaw.pAreas->m_data_list,
                                                                                     m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list, m_MapRaw.pSignals->m_data_list,
                                                                                     m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
                                                                                     m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
                                                                                     m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true, m_params.bEnableLaneChange, false);
                }
                else if (m_MapRaw.GetVersion() == 1)
                {
                    std::cout << "Map Version 1" << endl;
                    m_bKmlMap = true;
                    PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessage(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
                                                                                   m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list, m_MapRaw.pAreas->m_data_list,
                                                                                   m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list, m_MapRaw.pSignals->m_data_list,
                                                                                   m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
                                                                                   m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data, PlannerHNS::GPSPoint(), m_Map, true, m_params.bEnableLaneChange, false);
                }

                if (m_bKmlMap)
                {
                    visualization_msgs::MarkerArray map_marker_array;
                    PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
                    pub_MapRviz.publish(map_marker_array);
                }
            }

            ClearOldCostFromMap();
            static bool fist_planner = true;
            static int time_count =0;

            if(m_GoalsPos.size()==0 ){
                fist_planner = true;
            }
            remaing_dis_data.data = 120.0;
            if (m_GoalsPos.size() > 0)
            {
                if (fist_planner == false) // m_GeneratedTotalPaths.size() > 0  && m_GeneratedTotalPaths.at(0).size() > 3)
                {
                    if (m_params.bEnableReplanning)
                    {
                        PlannerHNS::RelativeInfo info;
                        bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GeneratedTotalPaths, m_CurrentPose, 0.75, info);
                        // if (ret == true && info.iGlobalPath >= 0 && info.iGlobalPath < m_GeneratedTotalPaths.size() && info.iFront > 0 && info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size())
                        if (ret == true && info.iGlobalPath >= 0 && info.iGlobalPath < m_GeneratedTotalPaths.size() && info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size())
                        {
                            double remaining_distance = m_GeneratedTotalPaths.at(info.iGlobalPath).at(m_GeneratedTotalPaths.at(info.iGlobalPath).size() - 1).cost - (m_GeneratedTotalPaths.at(info.iGlobalPath).at(info.iFront).cost + info.to_front_distance);
                            remaing_dis_data.data = remaining_distance;
                            if (remaining_distance <= REPLANNING_DISTANCE)
                            {
                                if (!(m_params.bOnlyReplanning == true && (m_iCurrentGoalIndex == m_GoalsPos.size() - 1) && m_GoalsPos.size()> 1)){
                                    static bool bTime=true;
                                    if (bTime == true)
                                    {
                                        if(m_SiteArray.sites.size() == 0)
                                        {
                                            _timer.setPeriod(ros::Duration(15));
                                            ROS_ERROR("准备到达目标点, %d 秒后启动下一个目标点",15);
                                        }else{
                                            _timer.setPeriod(ros::Duration(m_SiteArray.sites[m_iCurrentGoalIndex].time));
                                            ROS_ERROR("准备到达目标点, %d 秒后启动下一个目标点", m_SiteArray.sites[m_iCurrentGoalIndex].time);
                                            m_bStartPlan = false;
                                        }
                                        _timer.start();
                                        bTime = false;
                                    }
                                    ROS_ERROR("规划倒退计时中: %d,%d,%d", time_count++,m_bStartPlan,m_next_goal_on);

                                    if ((m_GoalsPos.size() > 0) && (m_bStartPlan == true || m_next_goal_on == true))
                                    {
                                        time_count =0;
                                        bTime = true;
                                        bMakeNewPlan = true;
                                        m_bStartPlan = false;
                                        m_next_goal_on = false;
                                        _timer.stop();
                                        m_iCurrentGoalIndex = (m_iCurrentGoalIndex + 1) % m_GoalsPos.size();
                                        std::cout << "Current Goal Index = " << m_iCurrentGoalIndex << std::endl
                                            << std::endl;
                                    }
                                }
                            }
                        }
                    }else{ // 实时目标点规划
                        if (m_GoalsPos.size() > 1)
                        {
                            bMakeNewPlan = true;
                            m_iCurrentGoalIndex = 0;
                            m_GoalsPos.erase(m_GoalsPos.begin());
                        }
                    } 
                }
                else{
                    ROS_ERROR("第一次规划路径");
                    fist_planner = false;
                    bMakeNewPlan = true;
                }

                ROS_DEBUG("+++++++ 全局目标点个数 %d ,离目标点的距离: %f", m_GoalsPos.size(), remaing_dis_data.data);
                if (bMakeNewPlan || (m_params.bEnableDynamicMapUpdate && UtilityHNS::UtilityH::GetTimeDiffNow(m_ReplnningTimer) > REPLANNING_TIME))  // 5s更新一次
                {
                    UtilityHNS::UtilityH::GetTickCount(m_ReplnningTimer);
                    PlannerHNS::WayPoint goalPoint = m_GoalsPos.at(m_iCurrentGoalIndex);
                    bool bNewPlan = GenerateGlobalPlan(m_CurrentPose, goalPoint, m_GeneratedTotalPaths);

                    if (bNewPlan)
                    {
                        m_SiteSatus.status = 2;
                        bMakeNewPlan = false;
                        VisualizeAndSend(m_GeneratedTotalPaths);
                        // ROS_ERROR("+++++++ 新路径规划成功:  index: %d, time: %d",m_iCurrentGoalIndex,m_SiteArray.sites[m_iCurrentGoalIndex].time);
                        ROS_ERROR("+++++++ 新路径规划成功:  index: %d",m_iCurrentGoalIndex);
                    }else
                    {
                        m_SiteSatus.status = 1;
                    }
                }
                VisualizeDestinations(m_GoalsPos, m_iCurrentGoalIndex);
                // if (m_params.bOnlyReplanning == true && (m_iCurrentGoalIndex == m_GoalsPos.size() - 1) && m_GoalsPos.size()> 1)
                // if (m_bEndSiteFlag == true && (m_iCurrentGoalIndex == m_GoalsPos.size() - 1) && m_GoalsPos.size() > 1)
                {
                    // m_bEndSiteFlag = false;
                    // m_GoalsPos.clear();
                    // m_iCurrentGoalIndex = 0;
                    // m_SiteSatus.status = 3;
                }
            }
            pub_RemainingDistance.publish(remaing_dis_data);

            // 发布当前站点的index
            m_SiteSatus.pre_index = m_iCurrentGoalIndex - 1;
            m_SiteSatus.next_index = m_iCurrentGoalIndex;
            m_SiteSatus.current_index = -1;
            if (remaing_dis_data.data < REPLANNING_DISTANCE)
            {

                    m_SiteSatus.current_index = m_iCurrentGoalIndex;
                    m_SiteSatus.next_index = m_iCurrentGoalIndex + 1;
            }

            if ((m_GoalsPos.size() > 0))
            {
                if (m_SiteSatus.current_index != -1 && m_SiteSatus.current_index >= (m_GoalsPos.size() - 1))
                {
                    m_SiteSatus.status = 3;
                }else
                {
                    m_SiteSatus.status = 2;
                }
            }
            if (m_SiteSatus.next_index >= m_GoalsPos.size()){
                m_SiteSatus.next_index = -1;
            }
            m_SiteSatus.max_index = m_GoalsPos.size() - 1;

            if(m_GoalsPos.size() == 0)
            {
                m_SiteSatus.pre_index = - 1;
                m_SiteSatus.next_index = -1;
                m_SiteSatus.current_index = -1;
                m_SiteSatus.status = 1;
            }

            if(low_battery_flag == true)        // 低电量时，状态设置为4
            {
                m_SiteSatus.status = 4;
            }
            pub_planner_status.publish(m_SiteSatus);
         
            
            loop_rate.sleep();
        }
    }

    // Mapping Section

    void GlobalPlanner::callbackGetVMLanes(const vector_map_msgs::LaneArray &msg)
    {
        std::cout << "Received Lanes" << msg.data.size() << endl;
        if (m_MapRaw.pLanes == nullptr)
            m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMPoints(const vector_map_msgs::PointArray &msg)
    {
        std::cout << "Received Points" << msg.data.size() << endl;
        if (m_MapRaw.pPoints == nullptr)
            m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray &msg)
    {
        std::cout << "Received dtLanes" << msg.data.size() << endl;
        if (m_MapRaw.pCenterLines == nullptr)
            m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray &msg)
    {
        std::cout << "Received CrossRoads" << msg.data.size() << endl;
        if (m_MapRaw.pIntersections == nullptr)
            m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMAreas(const vector_map_msgs::AreaArray &msg)
    {
        std::cout << "Received Areas" << msg.data.size() << endl;
        if (m_MapRaw.pAreas == nullptr)
            m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMLines(const vector_map_msgs::LineArray &msg)
    {
        std::cout << "Received Lines" << msg.data.size() << endl;
        if (m_MapRaw.pLines == nullptr)
            m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMStopLines(const vector_map_msgs::StopLineArray &msg)
    {
        std::cout << "Received StopLines" << msg.data.size() << endl;
        if (m_MapRaw.pStopLines == nullptr)
            m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMSignal(const vector_map_msgs::SignalArray &msg)
    {
        std::cout << "Received Signals" << msg.data.size() << endl;
        if (m_MapRaw.pSignals == nullptr)
            m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMVectors(const vector_map_msgs::VectorArray &msg)
    {
        std::cout << "Received Vectors" << msg.data.size() << endl;
        if (m_MapRaw.pVectors == nullptr)
            m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMCurbs(const vector_map_msgs::CurbArray &msg)
    {
        std::cout << "Received Curbs" << msg.data.size() << endl;
        if (m_MapRaw.pCurbs == nullptr)
            m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray &msg)
    {
        std::cout << "Received Edges" << msg.data.size() << endl;
        if (m_MapRaw.pRoadedges == nullptr)
            m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray &msg)
    {
        std::cout << "Received Wayareas" << msg.data.size() << endl;
        if (m_MapRaw.pWayAreas == nullptr)
            m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray &msg)
    {
        std::cout << "Received CrossWalks" << msg.data.size() << endl;
        if (m_MapRaw.pCrossWalks == nullptr)
            m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
    }

    void GlobalPlanner::callbackGetVMNodes(const vector_map_msgs::NodeArray &msg)
    {
        std::cout << "Received Nodes" << msg.data.size() << endl;
        if (m_MapRaw.pNodes == nullptr)
            m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
    }

}
