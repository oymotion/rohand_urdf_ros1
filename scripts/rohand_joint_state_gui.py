#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.

import argparse
import random
import signal
import sys
import threading

import rospy
from sensor_msgs.msg import JointState

# Qt GUI库
from python_qt_binding.QtCore import pyqtSlot, Qt, Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (QApplication, QFormLayout, QGridLayout, 
                                         QHBoxLayout, QLabel, QLineEdit, QMainWindow,
                                         QPushButton, QSlider, QScrollArea, QVBoxLayout, QWidget)

# 自定义布局
from python_qt_binding.QtWidgets import QVBoxLayout  # 添加QVBoxLayout导入

RANGE = 10000
LINE_EDIT_WIDTH = 45
SLIDER_WIDTH = 200
INIT_NUM_SLIDERS = 7

# ROHand关节名称过滤
ROHAND_SLIDER_LIST = [
    'if_slider_link',
    'mf_slider_link',
    'rf_slider_link',
    'lf_slider_link',
    'th_slider_link',
    'th_root_link'
]

class Slider(QWidget):
    def __init__(self, name):
        super().__init__()
        self.joint_layout = QVBoxLayout()
        self.row_layout = QHBoxLayout()

        font = QFont("Helvetica", 9, QFont.Bold)
        self.label = QLabel(name)
        self.label.setFont(font)
        self.row_layout.addWidget(self.label)

        self.display = QLineEdit("0.00")
        self.display.setAlignment(Qt.AlignRight)
        self.display.setFont(font)
        self.display.setReadOnly(True)
        self.display.setFixedWidth(LINE_EDIT_WIDTH)
        self.row_layout.addWidget(self.display)

        self.joint_layout.addLayout(self.row_layout)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFont(font)
        self.slider.setRange(0, RANGE)
        self.slider.setValue(int(RANGE / 2))
        self.slider.setFixedWidth(SLIDER_WIDTH)

        self.joint_layout.addWidget(self.slider)
        self.setLayout(self.joint_layout)

    def remove(self):
        self.joint_layout.removeWidget(self.slider)
        self.slider.setParent(None)
        self.row_layout.removeWidget(self.display)
        self.display.setParent(None)
        self.row_layout.removeWidget(self.label)
        self.label.setParent(None)
        self.row_layout.setParent(None)

class JointStatePublisherGui(QMainWindow):
    sliderUpdateTrigger = Signal()
    initialize = Signal()

    def __init__(self, title):
        super(JointStatePublisherGui, self).__init__()
        # 初始化
        rospy.init_node('joint_state_publisher_gui', anonymous=True)
        
        self.joint_map = {}
        self.setWindowTitle(title)
        
        # 关节状态发布器
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = []
        self.joint_state_msg.position = []

        # 随机化按钮
        self.rand_button = QPushButton('Randomize', self)
        self.rand_button.clicked.connect(self.randomizeEvent)

        # 居中按钮
        self.ctr_button = QPushButton('Center', self)
        self.ctr_button.clicked.connect(self.centerEvent)

        # 滚动区域布局
        self.scroll_layout = QVBoxLayout()  # 用QVBoxLayout替换FlowLayout
        self.scroll_widget = QWidget()
        self.scroll_widget.setLayout(self.scroll_layout)
        
        # 滚动区域
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.scroll_widget)

        # 主布局
        self.main_layout = QVBoxLayout()
        self.main_layout.addWidget(self.rand_button)
        self.main_layout.addWidget(self.ctr_button)
        self.main_layout.addWidget(self.scroll_area)

        # 中心部件
        self.central_widget = QWidget()
        self.central_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.central_widget)

        self.running = True
        self.sliders = {}
        
        # ROS参数服务器获取URDF
        self.robot_description = rospy.get_param('robot_description', '')
        self.free_joints = self._parse_urdf(self.robot_description)

        # 信号连接
        self.initialize.connect(self.initializeSliders)
        self.sliderUpdateTrigger.connect(self.updateSliders)
        self.initialize.emit()

    def _parse_urdf(self, urdf_string):
        """解析URDF获取关节信息"""
        from urdf_parser_py.urdf import URDF
        free_joints = {}
        
        try:
            robot = URDF.from_xml_string(urdf_string)
            for joint in robot.joints:
                if joint.type != 'fixed' and joint.type != 'continuous':
                    free_joints[joint.name] = {
                        'min': joint.limit.lower,
                        'max': joint.limit.upper,
                        'zero': 0.0
                    }
        except Exception as e:
            rospy.logerr(f"URDF解析失败: {e}")
            
        return free_joints

    def initializeSliders(self):
        self.joint_map = {}
        
        # 清理现有滑块
        for sl in list(self.sliders.keys()):
            self.scroll_layout.removeWidget(sl)
            sl.remove()
            del self.sliders[sl]
        
        # 生成新滑块
        for name, joint_info in self.free_joints.items():
            if name not in ROHAND_SLIDER_LIST:
                continue
                
            if joint_info['min'] == joint_info['max']:
                continue

            slider = Slider(name)
            self.joint_map[name] = {
                'display': slider.display,
                'slider': slider.slider,
                'joint': joint_info
            }

            self.scroll_layout.addWidget(slider)  # 用QVBoxLayout的addWidget
            slider.slider.valueChanged.connect(
                lambda value, name=name: self.onSliderValueChangedOne(name)
            )
            self.sliders[slider] = slider

        for name, joint_info in self.joint_map.items():
            zero_pos = joint_info['joint']['zero']  # 0.0
            joint_info['slider'].setValue(self.valueToSlider(zero_pos, joint_info['joint']))
            joint_info['display'].setText("%.3f" % zero_pos)  # 强制显示0.000

    def onSliderValueChangedOne(self, name):
        """滑块值变更处理"""
        joint_info = self.joint_map[name]
        slidervalue = joint_info['slider'].value()
        joint = joint_info['joint']
        position = self.sliderToValue(slidervalue, joint)
        joint_info['display'].setText("%.3f" % position)
        
        # 发布关节状态
        self.publish_joint_state()

    @pyqtSlot()
    def updateSliders(self):
        """更新滑块显示值"""
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            position = joint.get('position', joint['zero'])
            slidervalue = self.valueToSlider(position, joint)
            joint_info['slider'].setValue(slidervalue)
            joint_info['display'].setText("%.3f" % position)

    def publish_joint_state(self):
        """发布关节状态消息"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        
        for name, joint_info in self.joint_map.items():
            slidervalue = joint_info['slider'].value()
            position = self.sliderToValue(slidervalue, joint_info['joint'])
            joint_info['joint']['position'] = position  # 更新位置
            
            msg.name.append(name)
            msg.position.append(position)
            
        self.pub.publish(msg)

    def centerEvent(self, event):
        """居中所有关节"""
        rospy.loginfo("Centering joints")
        for name, joint_info in self.joint_map.items():
            zero_pos = joint_info['joint']['zero']
            joint_info['slider'].setValue(self.valueToSlider(zero_pos, joint_info['joint']))
        self.publish_joint_state()

    def randomizeEvent(self, event):
        """随机化关节位置"""
        rospy.loginfo("Randomizing joints")
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            rand_value = random.uniform(joint['min'], joint['max'])
            joint_info['slider'].setValue(self.valueToSlider(rand_value, joint))
        self.publish_joint_state()

    def valueToSlider(self, value, joint):
        """实际值转滑块值"""
        if abs(value) < 1e-5:  # 约等于0.0
            return 0
        return int((value - joint['min']) * float(RANGE) / (joint['max'] - joint['min']))

    def sliderToValue(self, slider, joint):
        """滑块值转实际值"""
        return joint['min'] + (joint['max'] - joint['min']) * (slider / float(RANGE))

    def closeEvent(self, event):
        self.running = False

    def loop(self):
        """ROS1主循环"""
        rate = rospy.Rate(30)  # 30Hz
        while self.running and not rospy.is_shutdown():
            self.publish_joint_state()
            rate.sleep()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', help='URDF文件路径（可选）', default=None)
    args = parser.parse_args(rospy.myargv()[1:])
    
    # 如果通过参数指定URDF，加载到参数服务器
    if args.urdf:
        with open(args.urdf, 'r') as f:
            rospy.set_param('robot_description', f.read())
    
    app = QApplication(sys.argv)
    jsp_gui = JointStatePublisherGui('Joint State Publisher')
    jsp_gui.show()
    
    # 启动ROS循环线程
    threading.Thread(target=jsp_gui.loop).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
