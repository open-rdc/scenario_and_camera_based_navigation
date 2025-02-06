#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from typing_extensions import Self


import rospy
import rosparam
from std_msgs.msg import String,ColorRGBA
#from jsk_rviz_plugins.msg import OverlayText
from scenario_navigation.srv import Scenario
import MeCab
from jsk_rviz_plugins.msg import OverlayText

class ScenarioParser:
    def __init__(self):
        self.hz = 1
        self.loop_rate = rospy.Rate(self.hz)
        self.scenario_service_proxy_ = rospy.ServiceProxy('scenario', Scenario)
        self.text_pub = rospy.Publisher("scenario_text",OverlayText,queue_size=1)
        self.ACTIONS = ["直進", "前進", "右折", "左折", "向く", "停止", "曲がる"]
        self.DIRECTIONS = ["前", "前方", "右", "右手", "左", "左手", "後ろ", "後方"]
        self.TYPE = ["一本道", "三叉路", "行き止まり", "十字路",
                     "突き当り", "曲がり角", "角", "通路", "交差点"]
        self.ORDERS = ["つ", "番", "次"]

        self.ACTIONS_ENG = ["go_straight", "go_straight",
                            "turn_right", "turn_left", "turn_", "stop", "turn_"]
        self.DIRECTIONS_ENG = ["front", "front", "right",
                               "right", "left", "left", "back", "back"]
        # self.TYPE_ENG = ["straight_road", "3_way", "dead_end", "crossroads",
        #                  "end", "corridor", "corridor", "corridor", "corridor"]
        # self.TYPE_ENG = ["straight_road", "3_way", "dead_end", "crossroads",
        #                  "corridor", "corridor", "corridor", "corridor", "corridor"]
        self.TYPE_ENG = ["straight_road", "3_way", "dead_end", "crossroads",
                         "end", "corner", "corner", "corridor", "corridor"]

        self.action_ = []
#        self.distance_ = []
        self.order_ = []
        self.direction_ = []
        self.type_ = []
        self.condition_ = []
        self.mecab = MeCab.Tagger('-Owakati -d /usr/lib/x86_64-linux-gnu/mecab/dic/mecab-ipadic-neologd')
        #self.mecab = MeCab.Tagger('-Owakati -d /usr/share/mecab/dic/ipadic')
        # self.mecab = MeCab.Tagger('-Owakati')

# complement direction to turn if scenario-action is "turn_"
    def complement_scenario_action_turn_xxx(self, direction, action):
        if (action == "turn_"):
            if (direction == "left" or direction == "right" or direction == "back"):
                result = action + direction
        else:
            result = action

        return result

# Translate each features(for example, action and direction) of scenario from Japanese to English
# Not translate a word about "order" in scenario because order is integer
    def translation_from_ja_to_en(self, types, direction, action):
        if (types in self.TYPE):
            type_index = self.TYPE.index(types)
            return_type = self.TYPE_ENG[type_index]
        else:
            return_type = types

        if (direction in self.DIRECTIONS):
            direction_index = self.DIRECTIONS.index(direction)
            return_direction = self.DIRECTIONS_ENG[direction_index]
        else:
            return_direction = direction

        if (action in self.ACTIONS):
            action_index = self.ACTIONS.index(action)
            return_action = self.ACTIONS_ENG[action_index]
        else:
            return_action = action
        return_action = self.complement_scenario_action_turn_xxx(
            return_direction, return_action)

        return return_type, return_direction, return_action

# get a word related to "distance"
    def get_distance(self, sentence):
        try:
            tmp = [i for i, word in enumerate(sentence) if ("m" in word)]
            if (len(tmp) != 0):
                i = tmp[0]
                if ("m" in sentence[i]):
                    tmp = sentence[i]
                    distance = int(tmp.rstrip("m"))
                    return distance

        except Exception as e:
            print("############## ERROR ##############")
            print(e)
            return "nothing"

        return "nothing"

# get a word related to "order"
    def get_order(self, sentence):
        try:
            tmp = [i for i, word in enumerate(
                sentence) for order in self.ORDERS if (order in word)]
            if (len(tmp) != 0):
                i = tmp[0]
                if (sentence[i] == "次"):
                    order = 1
                    return int(order)
                elif (sentence[i+1] == "目"):
                    order = sentence[i].rstrip("つ")
                    order = order.rstrip("番")
                    order = int(order)
                    return order

        except Exception as e:
            print("############## ERROR ##############")
            print(e)
            return 1

        return 1

# get a word related to "direction"
    def get_direction(self, sentence):
        try:
            tmp = [i for i, word in enumerate(sentence) for direction in self.DIRECTIONS if ((direction in word) and
                                                                                             (("右折" not in word) and ("左折" not in word)))]
            if (len(tmp) != 0):
                i = tmp[0]
                direction = sentence[i]
                return direction

        except Exception as e:
            print("############## ERROR ##############")
            print(e)
            return "nothing"

        return "nothing"


# get a word related to "type"

    def get_type(self, sentence):
        try:
            tmp = [i for i, word in enumerate(
                sentence) for road_type in self.TYPE if (road_type in word)]
            if (len(tmp) != 0):
                i = tmp[0]
                road_type = sentence[i]
                return road_type

        except Exception as e:
            print("############## ERROR ##############")
            print(e)
            return "nothing"

        return "nothing"

# get a word related to "action"
    def get_action(self, sentence):
        try:
            tmp = [i for i, word in enumerate(
                sentence) for action in self.ACTIONS if (action in word)]
            if (len(tmp) != 0):
                i = tmp[0]
                action = sentence[i]
                return action

        except Exception as e:
            print("############## ERROR ##############")
            print(e)
            return "nothing"

        return "nothing"

# get a word related to condition
    def get_condition(self, sentence):
        try:
            tmp = [i for i, word in enumerate(sentence) if ("まで" in word)]
            if (len(tmp) != 0):
                i = tmp[0]
                condition = sentence[i]
                if ("見える" in sentence[i-1]):
                    condition = "見えるまで"

                return condition

        except Exception as e:
            print("############## ERROR ##############")
            print(e)
            return "nothing"

        return "nothing"

    def get_condition_and_action(self, scenario):
        all_sentences = scenario.split("．")
    # delete '\n'
        del all_sentences[-1]

        for sentence in all_sentences:
            splitted_sentence = self.mecab.parse(sentence)
            splitted_sentence = splitted_sentence.split(" ")
    # delete '\n'
            del splitted_sentence[-1]

#            distance = self.get_distance(splitted_sentence)
            order = self.get_order(splitted_sentence)
            direction = self.get_direction(splitted_sentence)
            road_type = self.get_type(splitted_sentence)
            condition = self.get_condition(splitted_sentence)
            action = self.get_action(splitted_sentence)

#            self.distance_.append(distance)
            self.order_.append(order)
            self.direction_.append(direction)
            self.type_.append(road_type)
            self.condition_.append(condition)
            self.action_.append(action)

# send conditions and action
    def send_scenarios(self, i):

        # debug
        #        print("type is ", self.type_[i])
        #        print("order is ", self.order_[i])
        #        print("direction is ", self.direction_[i])
        #        print("action is ", self.action_[i])

        scenario_order = self.order_[i]
        scenario_type, scenario_direction, scenario_action = self.translation_from_ja_to_en(
            self.type_[i], self.direction_[i], self.action_[i])

        try:
            self.scenario_service_proxy_(
                scenario_type, scenario_order, scenario_direction, scenario_action)
        except rospy.ServiceException as e:
            print("Service call failed: ", e)

    def show_result(self):
        for i in range(len(self.action_)):
            print("#######################################")
#            print("距離： ", self.distance_[i])
            print("順番： ", self.order_[i])
            print("方向： ", self.direction_[i])
            print("通路： ", self.type_[i])
            print("条件： ", self.condition_[i])
            print("行動： ", self.action_[i])
            print("\n")

    def overlay_text(self,data):
        text = OverlayText()
        text.width =600
        text.height =150
        text.left = 0
        text.top = 0
        color1 = ColorRGBA()
        color1.r=0
        color1.g=0
        color1.b=0
        color1.a=0.4
        text.fg_color =color1

        color2 = ColorRGBA()
        color2.r = 25.0 / 255
        color2.g = 255.0 / 255
        color2.b = 240.0 / 255
        color2.a = 0.8
        text.fg_color = color2
        text.line_width = 1
        text.text_size = 24
        text.font = "Ubuntu"
        text.text = '.'.join(data)
        self.text_pub.publish(text)

if __name__ == '__main__':
    try:
        rospy.init_node('scenario_parser', anonymous=True)
        scenario_parser = ScenarioParser()
        # text_pub = rospy.Publisher('scenariotopic',OverlayText,queue_size=1)
        # text_data = OverlayText
        rate = rospy.Rate(1)
        scenario_path = "/root/share/mecab/scenario.txt"
        try:
            scenario_path = rosparam.get_param("scenario_parser/scenario_path")
        except:
            rospy.logwarn("scenario_path is not set")

        with open(scenario_path) as f:
            data = f.readlines()
        for scenario in data:
            print("#######################################")
            print(scenario)
            scenario_parser.get_condition_and_action(scenario)
            scenario_parser.show_result()
            print("#######################################\n")
        # text_data.text =scenario
        # text_pub.publish(text_data)
        rospy.wait_for_service('scenario')
        for i in range(len(scenario_parser.type_)):
            scenario_parser.send_scenarios(i)
        print("Finish sending scenarios")
        # rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("except occur")
        pass
    while not rospy.is_shutdown():
        # print("cani")
        scenario_parser.overlay_text(data)
        # rospy.spin()
        rate.sleep()
