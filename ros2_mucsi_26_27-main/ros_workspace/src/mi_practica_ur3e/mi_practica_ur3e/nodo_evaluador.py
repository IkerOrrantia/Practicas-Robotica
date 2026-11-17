"""
Modulo que contiene el nodo EvaluadorCinematicoUR3e para la extraccion,
procesamiento e ingesta inteligente de datos cinematicos del robot UR3e.
"""

import json
from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from mi_practica_interfaces.msg import DatosCinematicos

class EvaluadorCinematicoUR3e(Node):
    UMBRAL_MOVIMIENTO_RAD_S: float = 0.001

    def __init__(self) -> None:
        super().__init__('evaluador_cinematico_ur3e')
        self.declare_parameter('trayectoria', 'lineal')
        self._sub_articulaciones = self.create_subscription(JointState, '/joint_states', self._callback_estado_articulaciones, 10)
        self._pub_datos_procesados = self.create_publisher(DatosCinematicos, '/ur3e/datos_cinematicos_procesados', 10)
        self._tiempo_anterior: Optional[float] = None
        self._velocidades_anteriores: Optional[List[float]] = None
        self._trayectoria_activa: str = "lineal"
        self._base_datos_trayectorias: Dict[str, List[Dict[str, float]]] = {"lineal": [], "joint": [], "circular": []}
        self.get_logger().info("Nodo EvaluadorCinematicoUR3e iniciado correctamente.")

    def _callback_estado_articulaciones(self, msg: JointState) -> None:
        self._trayectoria_activa = self.get_parameter('trayectoria').get_parameter_value().string_value
        velocidades: List[float] = list(msg.velocity)
        if not self._robot_esta_en_movimiento(velocidades):
            return
        tiempo_actual: float = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        posiciones: List[float] = list(msg.position)
        nombres: List[str] = list(msg.name)
        aceleraciones: List[float] = self._calcular_aceleracion(tiempo_actual, velocidades)
        self._publicar_mensaje_personalizado(msg.header, nombres, posiciones, velocidades, aceleraciones)
        self._almacenar_muestra(tiempo_actual, posiciones, velocidades, aceleraciones)

    def _robot_esta_en_movimiento(self, velocidades: List[float]) -> bool:
        return any(abs(v) > self.UMBRAL_MOVIMIENTO_RAD_S for v in velocidades)

    def _calcular_aceleracion(self, tiempo_actual: float, velocidades_actuales: List[float]) -> List[float]:
        if self._tiempo_anterior is None or self._velocidades_anteriores is None:
            self._tiempo_anterior = tiempo_actual
            self._velocidades_anteriores = velocidades_actuales
            return [0.0] * len(velocidades_actuales)
        dt: float = tiempo_actual - self._tiempo_anterior
        if dt <= 0.0:
            return [0.0] * len(velocidades_actuales)
        aceleraciones: List[float] = [(v_act - v_ant) / dt for v_act, v_ant in zip(velocidades_actuales, self._velocidades_anteriores)]
        self._tiempo_anterior = tiempo_actual
        self._velocidades_anteriores = velocidades_actuales
        return aceleraciones

    def _publicar_mensaje_personalizado(self, header, nombres: List[str], pos: List[float], vel: List[float], acc: List[float]) -> None:
        msg_custom = DatosCinematicos()
        msg_custom.header = header
        msg_custom.nombres_articulaciones = nombres
        msg_custom.posicion = pos
        msg_custom.velocidad = vel
        msg_custom.aceleracion = acc
        self._pub_datos_procesados.publish(msg_custom)

    def _almacenar_muestra(self, timestamp: float, pos: List[float], vel: List[float], acc: List[float]) -> None:
        if self._trayectoria_activa in self._base_datos_trayectorias:
            registro = {'timestamp': timestamp, 'posicion': pos, 'velocidad': vel, 'aceleracion': acc}
            self._base_datos_trayectorias[self._trayectoria_activa].append(registro)

    def exportar_datos_a_json(self) -> None:
        for nombre_tray, registros in self._base_datos_trayectorias.items():
            nombre_archivo = f"/home/laboratorio/ros_workspace/{nombre_tray}.json"
            with open(nombre_archivo, 'w', encoding='utf-8') as f:
                json.dump(registros, f, indent=4)
            print(f"[EXITO] Guardados {len(registros)} registros en {nombre_archivo}")

def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = EvaluadorCinematicoUR3e()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        print("\n[INFO] Deteniendo nodo y exportando datos a JSON...")
        nodo.exportar_datos_a_json()
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
