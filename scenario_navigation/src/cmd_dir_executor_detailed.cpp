#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int8MultiArray.h>
#include "geometry_msgs/Twist.h"
#include "scenario_navigation/Scenario.h"
#include "scenario_navigation/PassageType.h"
#include "scenario_navigation_msgs/cmd_dir_intersection.h"
#include <std_srvs/SetBool.h>
#include <unistd.h>
#include <cmath>
#include <vector>

#include <iostream>

class cmdVelController {
     public:
        cmdVelController();
        int SCENARIO_MAX = 10;
        void getRosParam(void);
       
        // <intersection>
        // bool compareScenarioAndPassageType(const std_msgs::String::ConstPtr& intersection_name);
        // void loadNextScenario(void);//
        // void updateLastNode(const std_msgs::String::ConstPtr& intersection_n);
        // bool compareLastNodeAndCurrentNode(const std_msgs::String::ConstPtr& intersection_name);
        // void passageTypeCallback(const std_msgs::String::ConstPtr& intersection_name);
        bool compareScenarioAndPassageType(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name);
        void loadNextScenario(void);//
        void updateLastNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_n);
        bool compareLastNodeAndCurrentNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name);
        void passageTypeCallback(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name);
        
        void turnFinish (bool change);
        void stopCallback(const std_msgs::Bool::ConstPtr& stop); //
        bool scenarioCallback(scenario_navigation::Scenario::Request& scenario,
                              scenario_navigation::Scenario::Response& res);
        bool nextscenarioCallback(std_srvs::SetBool::Request& next_req ,
                                  std_srvs::SetBool::Response& next_resp);
     private:
        ros::NodeHandle node_;
       

        // <intersection>
        ros::Publisher stop_pub_;
        ros::Publisher cmd_data_pub;
        ros::Subscriber intersection_sub_;
        ros::Subscriber passage_type_sub_;
        ros::Subscriber stop_sub_;
        ros::ServiceServer scenario_server_,next_scenario_srv_;


        std::list<std::string> target_type_;
        std::list<std::int16_t> target_order_;
        std::list<std::string> target_direction_;
        std::list<std::string> target_action_;

        std::list<std::string>::iterator target_type_itr_begin_;
        // std_msgs::Int8MultiArray cmd_data;
        scenario_navigation_msgs::cmd_dir_intersection cmd_data;
        std::list<std::int16_t>::iterator target_order_itr_begin_;
        std::list<std::string>::iterator target_direction_itr_begin_;
        std::list<std::string>::iterator target_action_itr_begin_;

        std::string action;
        int scenario_num_ = 0;
        int scenario_progress_cnt_ = 0;
        int scenario_order_cnt_ = 1;
        int reach_target_type_cnt_ = 0;
        int reach_different_type_cnt_ = 0;
        int reach_target_type_cnt_margin_ = 6;
        int reach_different_type_cnt_margin_ = 6;
        float rotate_rad_ = 0.0;
        bool turn_flg_ = false;
        bool change_node_flg_ = false;
        bool satisfy_conditions_flg_ = false;
        bool stop_flg_ = true;
        bool request_update_last_node_flg = true;
        std_msgs::String last_node_;
        // std::string node_name;
        int str_list[3] = {1,0,0};
        int left_list[3] = {0,1,0};
        int right_list[3] = {0,0,1};
        int stop_list[3] = {0,0,0};
};

cmdVelController::cmdVelController(){
    stop_pub_ = node_.advertise<std_msgs::Bool>("stop", 1, false);
    //passage_type_sub_ = node_.subscribe<std_msgs::String> ("passage_type", 1, &cmdVelController::passageTypeCallback, this); //intersection name
    passage_type_sub_ = node_.subscribe<scenario_navigation_msgs::cmd_dir_intersection>("passage_type", 1, &cmdVelController::passageTypeCallback, this); //intersection name
    stop_sub_ = node_.subscribe<std_msgs::Bool> ("stop", 1, &cmdVelController::stopCallback, this);
    scenario_server_ = node_.advertiseService("scenario", &cmdVelController::scenarioCallback, this);
    next_scenario_srv_ = node_.advertiseService("next_scenario", &cmdVelController::nextscenarioCallback, this);
    // cmd_data_pub = node_.advertise<std_msgs::Int8MultiArray >("cmd_dir", 1);
    cmd_data_pub = node_.advertise<scenario_navigation_msgs::cmd_dir_intersection>("cmd_dir_intersection",1);
    // cmd_data.data.resize(3);

    // updateLastNode();
    getRosParam();
}

void cmdVelController::getRosParam(void){
    SCENARIO_MAX = 10;
    node_.getParam("scenario_executor/scenario_max", SCENARIO_MAX);
}
//<intersection>

bool cmdVelController::compareScenarioAndPassageType(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& passage_type){//シナリオと現在のノードの比較
    std::string target_type = *std::next(target_type_itr_begin_, scenario_progress_cnt_); //参照渡し
    std::string target_direction = *std::next(target_direction_itr_begin_, scenario_progress_cnt_); //参照渡し 今回は考慮しない

    //check "straight_road","3_way","cross_road","corridor"
    // if(target_type == passage_type->intersection_name){

    //         return true;
    //     }
    //直線の道
    if(target_type == "straight_road"){
        if(target_type == passage_type->intersection_name)
        return true;
    }
    //三叉路　右，中央，左
    if(target_type =="3_way"){
        if( passage_type->intersection_name == "3_way_right" || 
            passage_type->intersection_name == "3_way_center" ||
            passage_type->intersection_name == "3_way_left" ||
            passage_type->intersection_name == "corner_right" ||
            passage_type->intersection_name == "corner_left"){

            return true;
            }
    }
    // if(target_type =="corner"){
    //     if( passage_type->intersection_name == "corner_left" ||
    //         passage_type->intersection_name == "corner_right"|| 
    //         passage_type->intersection_name == "3_way_right" || 
    //         passage_type->intersection_name == "3_way_center"||
    //         passage_type->intersection_name == "3_way_left")
    //         return true;
    // }
    if(target_type == "end"){
        if( passage_type->intersection_name == "3_way_center"||
            passage_type->intersection_name == "corner_left"||
            passage_type->intersection_name == "corner_right"||
            passage_type->intersection_name == "dead_end"){
            
            return true;
            }
    }
    if(target_type == "corner"){
        if(target_direction == "left"){
            if( passage_type->intersection_name == "3_way_center"||
                passage_type->intersection_name == "3_way_left"||
                passage_type->intersection_name == "corner_left"
            )
                
                return true;
            
        }
        else if(target_direction == "right"){
            if( passage_type->intersection_name == "3_way_center"||
                passage_type->intersection_name == "3_way_right"||
                passage_type->intersection_name == "corner_right"
            )
                
                return true;
            
        }

    }
    if(target_type == "corridor"){
        if(target_direction == "left"){
            if( passage_type->intersection_name == "3_way_center"||
                passage_type->intersection_name == "3_way_left"||
                passage_type->intersection_name == "corner_left"
            )
                
                return true;
            
        }
        else if(target_direction == "right"){
            if( passage_type->intersection_name == "3_way_center"||
                passage_type->intersection_name == "3_way_right"||
                passage_type->intersection_name == "corner_right"
            )
                
                return true;
            
        }
    }
    return false;
}

void cmdVelController::loadNextScenario(void){
    // std::string action = *std::next(target_action_itr_begin_, scenario_progress_cnt_);
    action = *std::next(target_action_itr_begin_, scenario_progress_cnt_);

// stop robot
    stop_flg_ = true;
    if(action == "stop"){
        ROS_INFO("Robot gets a goal");
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
        std::copy(std::begin(stop_list),std::end(stop_list),std::begin(cmd_data.cmd_dir));
    }
    else{
        ROS_INFO("Execute next action(%s)", action.c_str());//string <=> char
        stop_flg_ = false;
        change_node_flg_ = false;
        
        if(action.find("turn") != std::string::npos){//turn find
            turn_flg_ = true;//曲がる動作を開始

            //if(action.find("left")){
            if(action == "turn_left"){
                std::copy(std::begin(left_list),std::end(left_list),std::begin(cmd_data.cmd_dir));
            
            }
            //else if(action.find("right")){
            else if(action == "turn_right"){
                std::copy(std::begin(right_list),std::end(right_list),std::begin(cmd_data.cmd_dir));
                
            }
        }
        else{
            std::copy(std::begin(str_list),std::end(str_list),std::begin(cmd_data.cmd_dir));
        }
    }
}

// <intersection>
// void cmdVelController::updateLastNode(const std_msgs::String::ConstPtr& intersection_n){
void cmdVelController::updateLastNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_n){
    ROS_INFO("update last node");
    last_node_.data = intersection_n->intersection_name;
    // node_name = last_node_.data;
    ROS_INFO("last node is (%s)",intersection_n->intersection_name.c_str());
 
    
}

// bool cmdVelController::compareLastNodeAndCurrentNode(const std_msgs::String::ConstPtr& intersection_name){//前回と今回のノードの比較
bool cmdVelController::compareLastNodeAndCurrentNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name){//前回と今回のノードの比較
    if(last_node_.data == intersection_name->intersection_name){
            return true;
        }
    else{
        return false;
    }
 }

// void cmdVelController::passageTypeCallback(const std_msgs::String::ConstPtr& passage_type){//メイン処理部
void cmdVelController::passageTypeCallback(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& passage_type){//メイン処理部
//std::copy(std::begin(left_list),std::end(left_list),std::begin(cmd_data.data));
    if(! stop_flg_){
        if(request_update_last_node_flg){
            updateLastNode(passage_type);
            request_update_last_node_flg = false;
        }
    }
        if(! turn_flg_){//actionがturnではない場合
            if(change_node_flg_){//道タイプが変更されている場合
                satisfy_conditions_flg_ = compareScenarioAndPassageType(passage_type);
                if(satisfy_conditions_flg_){
                    ROS_INFO("find target node !! ");
                    scenario_order_cnt_++;
                    change_node_flg_ = false;
                    updateLastNode(passage_type);
                    int order = *std::next(target_order_itr_begin_, scenario_progress_cnt_);
                    if(order <= scenario_order_cnt_){
                        ROS_INFO("Robot reaches target_node!!");
                        scenario_order_cnt_ = 0;
                        scenario_progress_cnt_++;
                        loadNextScenario();
                    }
                }
                else{
                    ROS_INFO("Not the goal of the scenario !!");
                }
            }
            else{
                if(!compareLastNodeAndCurrentNode(passage_type)){
                    updateLastNode(passage_type);
                    change_node_flg_ = true;
                    ROS_INFO("Node changed!!");
                }
                else{
                    ROS_INFO("Same node as last time !! ");
                }
            }
        }    
        else { //turn_flg_on　action中は継続
            if (!compareLastNodeAndCurrentNode(passage_type))
            {
                turnFinish(true);  //change_flg on ターン終了まで継続
            } 
        }
    if (passage_type->intersection_name == "dead_end") {
        ROS_INFO("Dead end detected. Stopping robot.");
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
        std::copy(std::begin(stop_list),std::end(stop_list),std::begin(cmd_data.cmd_dir));
    }
    cmd_data_pub.publish(cmd_data);//
}

void cmdVelController::turnFinish(bool change){
    ROS_INFO("finish turn");
    turn_flg_ = false;
    scenario_progress_cnt_++;
    loadNextScenario();
    request_update_last_node_flg = change;
    change_node_flg_ =false;
}

void cmdVelController::stopCallback(const std_msgs::Bool::ConstPtr& stop_flg){
    stop_flg_ = stop_flg->data;
}

bool cmdVelController::scenarioCallback(scenario_navigation::Scenario::Request& scenario, //一括読み込み
                                        scenario_navigation::Scenario::Response& res){
    scenario_num_++;
    target_type_.push_back(scenario.type);
    target_order_.push_back(scenario.order);
    target_direction_.push_back(scenario.direction);
    target_action_.push_back(scenario.action);
    std::string last_action = *std::next(target_action_.begin(), scenario_num_ - 1);//先頭を取得  scenario_num方向へすすめる

// check whether scenario is loaded
    if(last_action == "stop"){
        ROS_INFO("Completed loading scenario");
        //先頭のイテレータ取得
        target_type_itr_begin_ = target_type_.begin();
        target_order_itr_begin_ = target_order_.begin();
        target_direction_itr_begin_ = target_direction_.begin();
        target_action_itr_begin_ = target_action_.begin();

        stop_flg_ = false;
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
        loadNextScenario();
    }
    else{
        stop_flg_ = true;
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
    }


//  debug
    std::cout << "####################################" << std::endl;
    std::cout << "type is " << scenario.type << std::endl;
    std::cout << "order is "  << std::hex << scenario.order << std::endl;
    std::cout << "direction is " << scenario.direction << std::endl;
    std::cout << "action is " << scenario.action << std::endl;
    std::cout << "####################################" << std::endl;

    return true;
}
bool cmdVelController::nextscenarioCallback(std_srvs::SetBool::Request& next_req,
                              std_srvs::SetBool::Response& next_res){
    if(next_req.data){
        ROS_INFO("Next scenario");
        scenario_order_cnt_ = 0;
        scenario_progress_cnt_++;
        loadNextScenario();
    }
    else
        ROS_INFO("Please send true data");
}
    
int main(int argc, char** argv){
    ros::init(argc, argv, "scenario_executor");
    cmdVelController cmd_vel_controller;
    while(ros::ok()){
        ros::spin();
    }

    return 0;
}

