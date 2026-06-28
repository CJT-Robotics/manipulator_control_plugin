#!/usr/bin/env python3

import rospy
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                                         QPushButton, QScrollArea, QFrame, 
                                         QLineEdit, QInputDialog, QMessageBox)
from python_qt_binding.QtCore import Qt
from std_msgs.msg import Bool
from std_srvs.srv import SetBool  
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Joy

class ManipulatorPlugin(Plugin):
    def __init__(self, context):
        super(ManipulatorPlugin, self).__init__(context)
        self.setObjectName('ManipulatorPlugin')

        # Interne Variablen
        self._current_pose = None
        self._current_gripper_angle = 0.0
        self._last_buttons_state = []
        self._estop_active = False  # Trackt den aktuellen Zustand für das SpaceMouse-Toggeln
        
        # Welcher SpaceMouse-Button soll E-Stop toggeln? (0 bis 14)
        self.btn_idx_estop_toggle = 3  

        # ROS Publisher
        self.pub_ik_auto = rospy.Publisher('ik_auto_publish', Bool, queue_size=1, latch=True)
        self.pub_ik_approve = rospy.Publisher('ik_approve_enable', Bool, queue_size=1, latch=True)
        self.pub_target_pose = rospy.Publisher('ik_target_pose_set', PoseStamped, queue_size=1)
        self.pub_gripper_target = rospy.Publisher('gripper_target_state', JointState, queue_size=1)
        
        # ROS Subscriber
        self.sub_current_pose = rospy.Subscriber('ik_target_pose_get', PoseStamped, self._pose_callback)
        self.sub_joint_states = rospy.Subscriber('joint_states', JointState, self._joint_state_callback)
        # NEU: Spacenav Joystick Subscriber
        self.sub_spacenav_joy = rospy.Subscriber('spacenav/joy', Joy, self._spacenav_joy_cb)

        # Haupt-Widget erstellen
        self._widget = QWidget()
        self._widget.setWindowTitle('CJT Manipulator Control (Cartesian)')
        main_layout = QVBoxLayout(self._widget)

        # E-STOP BUTTONS
        estop_layout = QHBoxLayout()
        btn_estop_on = QPushButton("Digital E-STOP ACTIVATE")
        btn_estop_on.setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold; font-size: 13px; padding: 6px;")
        btn_estop_on.clicked.connect(lambda: self._call_estop_service(True))
        
        btn_estop_off = QPushButton("Digital E-STOP RELEASE")
        btn_estop_off.setStyleSheet("background-color: #388e3c; color: white; font-weight: bold; font-size: 13px; padding: 6px;")
        btn_estop_off.clicked.connect(lambda: self._call_estop_service(False))
        
        estop_layout.addWidget(btn_estop_on)
        estop_layout.addWidget(btn_estop_off)
        main_layout.addLayout(estop_layout)

        line_estop = QFrame()
        line_estop.setFrameShape(QFrame.HLine)
        line_estop.setFrameShadow(QFrame.Sunken)
        main_layout.addWidget(line_estop)

        # IK Buttons
        ik_layout = QHBoxLayout()
        btn_auto_true = QPushButton("IK Auto: TRUE")
        btn_auto_true.setStyleSheet("background-color: #2ecc71; font-weight: bold;")
        btn_auto_true.clicked.connect(lambda: self.pub_ik_auto.publish(Bool(data=True)))
        
        btn_auto_false = QPushButton("IK Auto: FALSE")
        btn_auto_false.setStyleSheet("background-color: #e74c3c; font-weight: bold;")
        btn_auto_false.clicked.connect(lambda: self.pub_ik_auto.publish(Bool(data=False)))
        
        btn_app_true = QPushButton("IK Approve: TRUE")
        btn_app_true.setStyleSheet("background-color: #3498db; font-weight: bold;")
        btn_app_true.clicked.connect(lambda: self.pub_ik_approve.publish(Bool(data=True)))
        
        btn_app_false = QPushButton("IK Approve: FALSE")
        btn_app_false.setStyleSheet("background-color: #f39c12; font-weight: bold;")
        btn_app_false.clicked.connect(lambda: self.pub_ik_approve.publish(Bool(data=False)))

        ik_layout.addWidget(btn_auto_true)
        ik_layout.addWidget(btn_auto_false)
        ik_layout.addWidget(btn_app_true)
        ik_layout.addWidget(btn_app_false)
        main_layout.addLayout(ik_layout)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        main_layout.addWidget(line)

        self.btn_add_position = QPushButton("+ Aktuelle Pose speichern")
        self.btn_add_position.setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px;")
        self.btn_add_position.clicked.connect(self._add_pose_button)
        main_layout.addWidget(self.btn_add_position)

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.dynamic_container = QWidget()
        self.dynamic_layout = QVBoxLayout(self.dynamic_container)
        self.dynamic_layout.setAlignment(Qt.AlignTop)
        self.scroll_area.setWidget(self.dynamic_container)
        main_layout.addWidget(self.scroll_area)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def _call_estop_service(self, state):
        """ Ruft den E-Stop Service auf und updatet den internen State """
        service_name = 'estop_bridge/set_digital_estop'
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            estop_service = rospy.ServiceProxy(service_name, SetBool)
            response = estop_service(data=state)
            if response.success:
                rospy.loginfo(f"E-Stop Service erfolgreich auf {state} gesetzt. Nachricht: {response.message}")
                self._estop_active = state  # Lokalen Zustand aktualisieren
            else:
                rospy.logwarn(f"Service wurde aufgerufen, schlug aber fehl: {response.message}")
                QMessageBox.warning(self._widget, "E-Stop Warnung", f"Service-Fehler: {response.message}")
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} nicht erreichbar!")
            QMessageBox.critical(self._widget, "Fehler", f"Service '{service_name}' antwortet nicht!\nLäuft die estop_bridge?")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service-Aufruf fehlgeschlagen: {e}")

    def _spacenav_joy_cb(self, msg):
        """ Callback für die SpaceMouse Tasten """
        # Falls die Tasten-Anzahl kürzer ist als unser zugewiesener Index, abbrechen
        if len(msg.buttons) <= self.btn_idx_estop_toggle:
            return

        # Initialisierung beim ersten empfangenen Paket
        if not self._last_buttons_state:
            self._last_buttons_state = list(msg.buttons)
            return

        # Flankenerkennung: Knopf jetzt gedrückt (1) und war vorher ungedrückt (0)
        if msg.buttons[self.btn_idx_estop_toggle] == 1 and self._last_buttons_state[self.btn_idx_estop_toggle] == 0:
            # Zustand invertieren und Service aufrufen
            new_estop_state = not self._estop_active
            rospy.loginfo(f"SpaceMouse Trigger: E-Stop Zustand wechselt auf {new_estop_state}")
            self._call_estop_service(new_estop_state)

        # Zustand speichern für den nächsten Durchlauf
        self._last_buttons_state = list(msg.buttons)

    def _pose_callback(self, msg):
        self._current_pose = msg

    def _joint_state_callback(self, msg):
        if "arm_7_gripper_joint_1_joint" in msg.name:
            idx = msg.name.index("arm_7_gripper_joint_1_joint")
            self._current_gripper_angle = msg.position[idx]

    def _add_pose_button(self):
        if self._current_pose is None:
            QMessageBox.warning(self._widget, "Fehler", "Noch keine Pose von 'ik_target_pose_get' empfangen!")
            return
        saved_pose = PoseStamped()
        saved_pose.header = self._current_pose.header
        saved_pose.pose = self._current_pose.pose
        saved_gripper_angle = self._current_gripper_angle

        name, ok = QInputDialog.getText(self._widget, 'Pose benennen', 'Name für diese Pose:')
        if not ok or not name.strip():
            name = f"Pose {self.dynamic_layout.count() + 1}"

        row_widget = QWidget()
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(0, 2, 0, 2)

        pub_btn = QPushButton(name)
        pub_btn.clicked.connect(lambda: self._publish_pose_and_gripper(saved_pose, saved_gripper_angle))
        
        rename_btn = QPushButton("✏️")
        rename_btn.setFixedWidth(35)
        rename_btn.clicked.connect(lambda: self._rename_button(pub_btn))

        delete_btn = QPushButton("🗑️")
        delete_btn.setFixedWidth(35)
        delete_btn.setStyleSheet("background-color: #c0392b; color: white;")
        delete_btn.clicked.connect(lambda: self._delete_row(row_widget))

        row_layout.addWidget(pub_btn)
        row_layout.addWidget(rename_btn)
        row_layout.addWidget(delete_btn)
        self.dynamic_layout.addWidget(row_widget)

    def _publish_pose_and_gripper(self, pose_msg, gripper_angle):
        pose_msg.header.stamp = rospy.Time.now()
        self.pub_target_pose.publish(pose_msg)
        gripper_msg = JointState()
        gripper_msg.header.stamp = rospy.Time.now()
        gripper_msg.name = ["arm_7_gripper_joint_1_joint", "arm_7_gripper_joint_2_joint"]
        gripper_msg.position = [gripper_angle, -gripper_angle]
        self.pub_gripper_target.publish(gripper_msg)
        rospy.loginfo(f"Pose '{pose_msg.header.frame_id}' und Greiferzustand erfolgreich gepublisht.")

    def _rename_button(self, button_to_rename):
        new_name, ok = QInputDialog.getText(self._widget, 'Umbenennen', 'Neuer Name:', text=button_to_rename.text())
        if ok and new_name.strip():
            button_to_rename.setText(new_name.strip())

    def _delete_row(self, row_widget):
        self.dynamic_layout.removeWidget(row_widget)
        row_widget.deleteLater()

    def shutdown_plugin(self):
        self.sub_current_pose.unregister()
        self.sub_joint_states.unregister()
        self.sub_spacenav_joy.unregister()  # Sauberes Unregister des neuen Subs
        self.pub_ik_auto.unregister()
        self.pub_ik_approve.unregister()
        self.pub_target_pose.unregister()
        self.pub_gripper_target.unregister()