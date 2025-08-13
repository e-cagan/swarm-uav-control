#!/usr/bin/env python3
import time
from typing import Dict, List, Optional, Callable

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State


class SwarmStartMissionNode(Node):
    """
    QGC'de görevi yüklü olan araçları (PX4) eşzamanlı olarak
    AUTO.MISSION moduna alıp ARM eder. Gerçek sonucu servis ACK'ına değil
    /<ns>/state mesajına bakarak teyit eder.

    Parametreler:
      - vehicle_namespaces : ['uas1','uas2','uas3']  # MAVROS --ros-args __ns ile eşleşmeli
      - mission_mode       : 'AUTO.MISSION'          # PX4 için
      - service_timeout_s  : 30.0  (servislerin gelmesi için)
      - wait_connected_s   : 30.0  (FCU 'connected' = True olana kadar)
      - sync_delay_s       : 2.0   (hepsi birlikte başlasın)
      - confirm_sleep_s    : 3.0   (son doğrulama için kısa bekleme)
    """

    def __init__(self):
        super().__init__('swarm_mission_node')

        # ---- Parametreler
        self.declare_parameter('vehicle_namespaces', ['uas1', 'uas2', 'uas3'])
        self.declare_parameter('mission_mode', 'AUTO.MISSION')
        self.declare_parameter('service_timeout_s', 30.0)
        self.declare_parameter('wait_connected_s', 30.0)
        self.declare_parameter('sync_delay_s', 2.0)
        self.declare_parameter('confirm_sleep_s', 3.0)

        vns = self.get_parameter('vehicle_namespaces').get_parameter_value().string_array_value
        self.vehicles: List[str] = list(vns) if vns else ['uas1', 'uas2', 'uas3']
        self.mission_mode: str = self.get_parameter('mission_mode').get_parameter_value().string_value or 'AUTO.MISSION'
        self.service_timeout_s = float(self.get_parameter('service_timeout_s').value)
        self.wait_connected_s = float(self.get_parameter('wait_connected_s').value)
        self.sync_delay_s = float(self.get_parameter('sync_delay_s').value)
        self.confirm_sleep_s = float(self.get_parameter('confirm_sleep_s').value)

        # ---- Araç durum tabloları
        self.uav: Dict[str, Dict] = {}
        for ns in self.vehicles:
            # DİKKAT: Senin MAVROS kurulumunda topic/service kökleri "/<ns>/*"
            # ("/<ns>/mavros/*" değil). O yüzden adlar aşağıdaki gibi.
            self.uav[ns] = {
                'arm_cli':  self.create_client(CommandBool, f'/{ns}/cmd/arming'),
                'mode_cli': self.create_client(SetMode,     f'/{ns}/set_mode'),
                'state_sub': self.create_subscription(
                    State, f'/{ns}/state', lambda msg, _ns=ns: self._on_state(_ns, msg), 10
                ),
                'connected': False,
                'armed': False,
                'mode': '',
            }

        self.executor: Optional[SingleThreadedExecutor] = None

    # ---------- Callbacks ----------
    def _on_state(self, ns: str, msg: State):
        self.uav[ns]['connected'] = bool(getattr(msg, 'connected', False))
        self.uav[ns]['armed'] = bool(getattr(msg, 'armed', False))
        self.uav[ns]['mode'] = getattr(msg, 'mode', '') or ''

    # ---------- Yardımcılar ----------
    def wait_services_and_connection(self, ns: str) -> bool:
        """Servisler ve FCU bağlantısı hazır olana kadar bekle."""
        t0 = time.monotonic()
        # Önce servisler
        while time.monotonic() - t0 < self.service_timeout_s:
            if (self.uav[ns]['arm_cli'].wait_for_service(timeout_sec=0.5) and
                self.uav[ns]['mode_cli'].wait_for_service(timeout_sec=0.5)):
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        else:
            return False

        # Sonra connected=True
        t1 = time.monotonic()
        while time.monotonic() - t1 < self.wait_connected_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.uav[ns]['connected']:
                return True
        return False

    def _call_and_wait(
        self,
        ns: str,
        client,
        request,
        want_ok: Callable[[], bool],
        label: str,
        timeout_s: float,
    ) -> bool:
        """Servisi çağır, ama başarıyı /state ile doğrula."""
        fut = client.call_async(request)
        t0 = time.monotonic()
        srv_ack = None  # True/False/None
        while time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)

            if srv_ack is None and fut.done():
                try:
                    resp = fut.result()
                    # CommandBool -> success, SetMode -> mode_sent
                    srv_ack = getattr(resp, 'success', getattr(resp, 'mode_sent', None))
                except Exception:
                    srv_ack = False

            if want_ok():
                if srv_ack is False:
                    self.get_logger().warn(f'{ns}: {label} -> state OK, fakat srv_ack=false (bilgi)')
                else:
                    self.get_logger().info(f'{ns}: {label} -> state OK')
                return True

        self.get_logger().warn(f'{ns}: {label} -> TIMEOUT (srv_ack={srv_ack})')
        return False

    def set_mode(self, ns: str, mode: str, timeout_s: float = 12.0) -> bool:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        return self._call_and_wait(
            ns,
            self.uav[ns]['mode_cli'],
            req,
            lambda: self.uav[ns]['mode'] == mode,
            f'set_mode("{mode}")',
            timeout_s,
        )

    def arm(self, ns: str, value: bool, timeout_s: float = 12.0) -> bool:
        req = CommandBool.Request()
        req.value = value
        return self._call_and_wait(
            ns,
            self.uav[ns]['arm_cli'],
            req,
            lambda: self.uav[ns]['armed'] is value,
            f'arm({value})',
            timeout_s,
        )

    # ---------- Ana akış ----------
    def orchestrate(self):
        self.get_logger().info(f'Vehicles: {self.vehicles}')

        ready: List[str] = []
        for ns in self.vehicles:
            ok = self.wait_services_and_connection(ns)
            self.get_logger().info(
                f'{ns}: services={ok}, connected={self.uav[ns]["connected"]}, mode={self.uav[ns]["mode"]}'
            )
            if ok and self.uav[ns]['connected']:
                ready.append(ns)
            else:
                self.get_logger().error(f'{ns}: skipping (not ready)')

        if not ready:
            self.get_logger().error('No vehicle is ready. Abort.')
            return

        # Senkron başlatma
        self.get_logger().info(f'Start at t0 in {self.sync_delay_s:.1f}s...')
        t0 = time.monotonic() + self.sync_delay_s
        while time.monotonic() < t0:
            rclpy.spin_once(self, timeout_sec=0.05)

        # Mod ve ARM
        for ns in ready:
            self.set_mode(ns, self.mission_mode)
            time.sleep(0.05)
        for ns in ready:
            self.arm(ns, True)
            time.sleep(0.05)

        # Son teyit
        t_conf = time.monotonic()
        while time.monotonic() - t_conf < self.confirm_sleep_s:
            rclpy.spin_once(self, timeout_sec=0.05)

        for ns in ready:
            self.get_logger().info(
                f'{ns}: final -> connected={self.uav[ns]["connected"]}, '
                f'armed={self.uav[ns]["armed"]}, mode={self.uav[ns]["mode"]}'
            )


def main():
    rclpy.init()
    node = SwarmStartMissionNode()
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)
    node.executor = exec_
    try:
        node.orchestrate()
    finally:
        exec_.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
