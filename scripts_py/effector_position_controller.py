#!/usr/bin/env python3
#ROS
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
#Otras Librerias de Python
import numpy as np
import math
import time

class EffectorPositionController(Node):
    def __init__(self, x_desired, y_desired, z_desired, psi_desired):
        super().__init__("effector_position_controller")

        #Tiempo        
        self.ts = 0.1 

        #Punto Deseado
        self.h_d_x = x_desired
        self.h_d_y = y_desired
        self.h_d_z = z_desired
        self.psi_d = psi_desired
        print(f"Objective: {x_desired}, {y_desired}, {z_desired}")
        
        #Posicion Inicial de Actuadores
        q_1_start = 0.0
        q_2_start = 0.0
        d_3_start = 0.0
        q_4_start = 0.0

        #Posicion Actual de Actuadores
        self.q_1_current = q_1_start
        self.q_2_current = q_2_start
        self.d_3_current = d_3_start
        self.q_4_current = q_4_start

        #Matriz de Ganancias
        self.gain_k = np.array(
            [
                [0.3, 0.0, 0.0, 0.0],
                [0.0, 0.3, 0.0, 0.0],
                [0.0, 0.0, 0.3, 0.0],
                [0.0, 0.0, 0.0, 0.3],
            ],
            float
        )
       
        #Cosas de ROS2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_msg = JointState()
        self.effector_position_publisher = self.create_publisher(Pose, "/effector_position", 10)
        self.effector_position_msg = Pose()

        #Iniciar Algoritmo de Control
        self.position_timer = self.create_timer(
            self.ts, 
            self.estimate_current_effector_position
        )

    def send_transform_to_urdf(self, frame_name, joint_name, q_position):
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_msg.header.frame_id = frame_name
        self.joint_msg.name = [joint_name]
        self.joint_msg.position = [q_position]

        self.joint_publisher.publish(self.joint_msg)    
       
    def compute_inverse_jacobian_matrix(self, q_1, q_2, d_3, q_4):
        #Calcular Items de la Matriz Jacobiana
        j_11 = float(math.cos(q_1 + q_2 + q_4))
        j_12 = float(-1.0 * math.sin(q_1 + q_2 + q_4))
        j_13 = float(0.0)
        j_14 = float((425.0 * math.cos(q_1)) + (345.0 * math.cos(q_1 + q_2)))
        j_21 = float(math.sin(q_1 + q_2 + q_4))
        j_22 = float(math.cos(q_1 + q_2 + q_4))
        j_23 = float(0.0)
        j_24 = float((425.0 * math.cos(q_1)) + (345.0 * math.cos(q_1 + q_2)))
        j_31 = float(0.0)
        j_32 = float(0.0)
        j_33 = float(1.0)
        j_34 = float(490.0 - d_3)
        j_41 = float(0.0)
        j_42 = float(0.0)
        j_43 = float(0.0)
        j_44 = float(1.0)

        #Calcular Items de la Matriz Jacobiana
        j_matrix = np.array(
            [
                [j_11, j_12, j_13, j_14], 
                [j_21, j_22, j_23, j_24],
                [j_31, j_32, j_33, j_34],
                [j_41, j_42, j_43, j_44]
            ],
            float
        )

        j_matrix_inverse = np.linalg.inv(j_matrix)
        return j_matrix_inverse
   
    def compute_prediction_value(self, value_current, value_p_current):
        return float(value_current + (self.ts * value_p_current))

    def control_effector_position(self, h_x, h_y, h_z, psi): 
        #Calcular Error en la Posicion del Efector
        self.h_e_x = float(math.tanh(self.h_d_x - h_x))
        self.h_e_y = float(math.tanh(self.h_d_y - h_y))
        self.h_e_z = float(math.tanh(self.h_d_z - h_z))
        self.psi_e = float(math.tanh(self.psi_d - psi))

        #Vector de Error en la Posicion del Efector
        h_e_saturated_vector = np.array(
            [
                [
                    self.h_e_x, 
                    self.h_e_y, 
                    self.h_e_z, 
                    self.psi_e
                ]
            ],
            float
        ).reshape(4,1)
        print("Current Effector Position Error:")
        print(f"h_e_x: {round(self.h_e_x, 3)} m")
        print(f"h_e_y: {round(self.h_e_y, 3)} m")
        print(f"h_e_z: {round(self.h_e_z, 3)} m")

        #Matriz Jacobiana Inversa
        j_matrix_inverse  = self.compute_inverse_jacobian_matrix(
            self.q_1_current,
            self.q_2_current,
            self.d_3_current,
            self.q_4_current
        )
        print("Inverse Jacobian Matrix:")
        print(f"{j_matrix_inverse}")
        
        #Calcular Velocidad de los Actuadores
        q_p_vector = np.matmul(
            j_matrix_inverse, 
            np.matmul(self.gain_k, h_e_saturated_vector)
        ).reshape(4,1)

        #Predecir Siguiente Posicion de los Actuadores
        self.q_1_current = self.compute_prediction_value(self.q_1_current, q_p_vector[0])
        self.q_2_current = self.compute_prediction_value(self.q_2_current, q_p_vector[1])
        self.d_3_current = self.compute_prediction_value(self.d_3_current, q_p_vector[2])

    def estimate_current_effector_position(self):
        #Enviar Posicion de los Actuadores al URDF
        self.send_transform_to_urdf('link2','link2_joint', self.q_1_current)
        self.send_transform_to_urdf('link3','link3_joint', self.q_2_current)
        self.send_transform_to_urdf('effector', 'effector_prismatic_joint', self.d_3_current)
        print("\nCurrent Actuators Position:")
        print(f"q_1: {round(self.q_1_current, 3)} rad")
        print(f"q_2: {round(self.q_2_current, 3)} rad")
        print(f"d_3: {round(self.d_3_current, 3)} m")
        print(f"q_4: {round(self.q_4_current, 3)} rad")

        try:
            #Estimar Posicion Actual del Efector
            effector_tf = self.tf_buffer.lookup_transform(
                'effector', 
                'base_link', 
                rclpy.time.Time()
            )
            h_x = float(effector_tf.transform.translation.x)
            h_y = float(effector_tf.transform.translation.y)
            h_z = float(effector_tf.transform.translation.z)
            psi = 0.0

            #Publicar Posicion del Efector
            self.effector_position_msg.position.x = h_x
            self.effector_position_msg.position.y = h_y
            self.effector_position_msg.position.z = h_z
            self.effector_position_publisher.publish(self.effector_position_msg)

            #Controlar Posicion del Efector
            self.control_effector_position(h_x, h_y, h_z, psi)
        except TransformException as ex:
            print(ex) 

def main():
    rclpy.init()
    try:
        #Posicion deseada
        x_desired = -0.8
        y_desired = 0.45
        z_desired = 0.15
        psi_desired = 0.0

        #Nodo del Controlador
        effector_position_controller_node = EffectorPositionController(
            x_desired= x_desired,
            y_desired= y_desired,
            z_desired= z_desired,
            psi_desired= psi_desired
        )
        rclpy.spin(effector_position_controller_node)
    except:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
