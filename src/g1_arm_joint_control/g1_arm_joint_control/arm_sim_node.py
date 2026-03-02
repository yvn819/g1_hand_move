import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd, LowState


class ArmSimNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_sim_node")

        self._declare_params()
        self._load_params()

        scene_path = self._resolve_scene_path(self.scene_path)
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.model.opt.timestep = self.simulate_dt
        self.data = mujoco.MjData(self.model)

        self.base_qpos_dofs = 7 if self.model.nq >= 7 else 0
        self.base_qvel_dofs = 6 if self.model.nv >= 6 else 0
        self.motor_count = int(self.model.nu)
        self.ctrl_min = self.model.actuator_ctrlrange[:, 0].copy()
        self.ctrl_max = self.model.actuator_ctrlrange[:, 1].copy()

        self.lock = threading.Lock()
        self.running = True
        self.viewer = None

        self.mode_pr = 0
        self.mode_machine = 0
        self.tick = 0
        self.msg_motor_count = len(LowCmd().motor_cmd)
        self._last_lowcmd_warning = 0.0

        self.cmd_q = self.data.qpos[
            self.base_qpos_dofs:self.base_qpos_dofs + self.motor_count
        ].copy()
        self.cmd_dq = np.zeros(self.motor_count, dtype=np.float64)
        self.cmd_kp = np.zeros(self.motor_count, dtype=np.float64)
        self.cmd_kd = np.zeros(self.motor_count, dtype=np.float64)
        self.cmd_tau = np.zeros(self.motor_count, dtype=np.float64)
        self.locked_base_qpos = (
            self.data.qpos[:self.base_qpos_dofs].copy()
            if self.base_qpos_dofs > 0
            else np.array([], dtype=np.float64)
        )

        self.imu_quat_sensor = self._pick_sensor_name(("imu_quat", "secondary_imu_quat"))
        self.imu_gyro_sensor = self._pick_sensor_name(("imu_gyro", "secondary_imu_gyro"))
        self.imu_acc_sensor = self._pick_sensor_name(("imu_acc", "secondary_imu_acc"))

        self.lowcmd_sub = self.create_subscription(
            LowCmd,
            self.lowcmd_topic,
            self.lowcmd_cb,
            10,
        )
        self.lowstate_pub = self.create_publisher(
            LowState,
            self.lowstate_topic,
            10,
        )

        self.get_logger().info(
            f"MuJoCo scene loaded: {scene_path} (nu={self.motor_count}, "
            f"simulate_dt={self.simulate_dt:.4f}, viewer_dt={self.viewer_dt:.4f})"
        )
        self.get_logger().info(
            f"Topics: lowcmd={self.lowcmd_topic}, lowstate={self.lowstate_topic}"
        )
        if self.lock_base and self.base_qpos_dofs > 0:
            self.get_logger().info(
                f"Base lock enabled: qpos[:{self.base_qpos_dofs}] fixed to initial pose."
            )
        if self.print_scene_information:
            self.get_logger().info(
                f"Model info: nq={self.model.nq}, nv={self.model.nv}, nu={self.model.nu}, "
                f"nsensor={self.model.nsensor}"
            )

        self.viewer_thread = threading.Thread(target=self.launch_viewer, daemon=True)
        self.sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.viewer_thread.start()
        self.sim_thread.start()

    def _declare_params(self) -> None:
        self.declare_parameter(
            "scene_path",
            "/home/yutong/scaledrive.ai/fincantieri/g1_control/g1_control_ws/src/"
            "g1_mujoco_nav/assets/unitree_robots/g1/scene_29dof.xml",
        )
        self.declare_parameter("simulate_dt", 0.005)
        self.declare_parameter("viewer_dt", 0.02)
        self.declare_parameter("print_scene_information", True)
        self.declare_parameter("lock_base", True)

        self.declare_parameter("lowcmd_topic", "/lowcmd")
        self.declare_parameter("lowstate_topic", "/lowstate")

        self.declare_parameter("camera_lookat", [0.0, 0.0, 0.8])
        self.declare_parameter("camera_distance", 4.0)
        self.declare_parameter("camera_elevation", -15.0)
        self.declare_parameter("camera_azimuth", 90.0)

    def _load_params(self) -> None:
        self.scene_path = str(self.get_parameter("scene_path").value)
        self.simulate_dt = float(self.get_parameter("simulate_dt").value)
        self.viewer_dt = float(self.get_parameter("viewer_dt").value)
        self.print_scene_information = bool(self.get_parameter("print_scene_information").value)
        self.lock_base = bool(self.get_parameter("lock_base").value)

        self.lowcmd_topic = str(self.get_parameter("lowcmd_topic").value)
        self.lowstate_topic = str(self.get_parameter("lowstate_topic").value)

        self.camera_lookat = np.asarray(self.get_parameter("camera_lookat").value, dtype=np.float64)
        if self.camera_lookat.size != 3:
            raise ValueError("camera_lookat must be [x, y, z].")
        self.camera_distance = float(self.get_parameter("camera_distance").value)
        self.camera_elevation = float(self.get_parameter("camera_elevation").value)
        self.camera_azimuth = float(self.get_parameter("camera_azimuth").value)

    def _resolve_scene_path(self, configured_path: str) -> Path:
        path = Path(configured_path).expanduser()
        if path.is_absolute():
            if not path.exists():
                raise FileNotFoundError(f"scene_path not found: {path}")
            return path

        package_root = Path(__file__).resolve().parents[1]
        candidates = [
            Path.cwd() / path,
            Path(__file__).resolve().parent / path,
            package_root / path,
            package_root / "assets" / path,
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()

        raise FileNotFoundError(
            f'Could not resolve scene_path "{configured_path}". '
            f"Tried: {[str(c) for c in candidates]}"
        )

    def _pick_sensor_name(self, names: tuple[str, ...]) -> str | None:
        for name in names:
            sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sensor_id >= 0:
                return name
        return None

    def _read_sensor(self, sensor_name: str | None, expected_dim: int) -> np.ndarray | None:
        if sensor_name is None:
            return None
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sensor_id < 0:
            return None
        adr = self.model.sensor_adr[sensor_id]
        dim = self.model.sensor_dim[sensor_id]
        if dim != expected_dim:
            return None
        return self.data.sensordata[adr:adr + dim].copy()

    def _configure_camera(self, viewer) -> None:
        viewer.cam.lookat[:] = self.camera_lookat
        viewer.cam.distance = self.camera_distance
        viewer.cam.elevation = self.camera_elevation
        viewer.cam.azimuth = self.camera_azimuth

    def launch_viewer(self) -> None:
        try:
            viewer_ctx = mujoco.viewer.launch_passive(self.model, self.data)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to start MuJoCo viewer: {exc}. Check DISPLAY/GL and Python env."
            )
            self.running = False
            return

        with viewer_ctx as viewer:
            self.viewer = viewer
            with self.lock:
                self._configure_camera(viewer)
            while self.running and viewer.is_running():
                with self.lock:
                    viewer.sync()
                time.sleep(self.viewer_dt)

        self.running = False

    def lowcmd_cb(self, msg: LowCmd) -> None:
        if len(msg.motor_cmd) < self.motor_count:
            now = time.time()
            if now - self._last_lowcmd_warning > 1.0:
                self.get_logger().warning(
                    f"LowCmd has {len(msg.motor_cmd)} motors but simulation needs {self.motor_count}."
                )
                self._last_lowcmd_warning = now
            return

        with self.lock:
            self.mode_pr = int(msg.mode_pr)
            self.mode_machine = int(msg.mode_machine)
            for i in range(self.motor_count):
                motor_cmd = msg.motor_cmd[i]
                self.cmd_q[i] = float(motor_cmd.q)
                self.cmd_dq[i] = float(motor_cmd.dq)
                self.cmd_kp[i] = float(motor_cmd.kp)
                self.cmd_kd[i] = float(motor_cmd.kd)
                self.cmd_tau[i] = float(motor_cmd.tau)

    def _apply_lowcmd_locked(self) -> None:
        curr_q = self.data.qpos[
            self.base_qpos_dofs:self.base_qpos_dofs + self.motor_count
        ]
        curr_dq = self.data.qvel[
            self.base_qvel_dofs:self.base_qvel_dofs + self.motor_count
        ]
        tau = (
            self.cmd_tau
            + self.cmd_kp * (self.cmd_q - curr_q)
            + self.cmd_kd * (self.cmd_dq - curr_dq)
        )
        self.data.ctrl[:] = np.clip(tau, self.ctrl_min, self.ctrl_max)

    def _enforce_locked_base_locked(self) -> None:
        if not self.lock_base:
            return
        if self.base_qpos_dofs > 0 and self.locked_base_qpos.size == self.base_qpos_dofs:
            self.data.qpos[:self.base_qpos_dofs] = self.locked_base_qpos
        if self.base_qvel_dofs > 0:
            self.data.qvel[:self.base_qvel_dofs] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def simulation_loop(self) -> None:
        next_step = time.perf_counter()

        with self.lock:
            self._publish_lowstate_locked()

        while self.running:
            if self.viewer is not None and not self.viewer.is_running():
                break

            with self.lock:
                self._apply_lowcmd_locked()
                self._enforce_locked_base_locked()
                mujoco.mj_step(self.model, self.data)
                self._enforce_locked_base_locked()
                self._publish_lowstate_locked()

            next_step += self.simulate_dt
            sleep_time = next_step - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_step = time.perf_counter()

        self.running = False

    def _publish_lowstate_locked(self) -> None:
        q = self.data.qpos[
            self.base_qpos_dofs:self.base_qpos_dofs + self.motor_count
        ].copy()
        dq = self.data.qvel[
            self.base_qvel_dofs:self.base_qvel_dofs + self.motor_count
        ].copy()

        if self.data.sensordata.size >= 3 * self.motor_count:
            tau_est = self.data.sensordata[
                2 * self.motor_count:3 * self.motor_count
            ].copy()
        else:
            tau_est = self.data.ctrl.copy()

        imu_quat = self._read_sensor(self.imu_quat_sensor, 4)
        imu_gyro = self._read_sensor(self.imu_gyro_sensor, 3)
        imu_acc = self._read_sensor(self.imu_acc_sensor, 3)

        if imu_quat is None:
            imu_quat = self.data.qpos[3:7].copy()
        if imu_gyro is None:
            imu_gyro = self.data.qvel[3:6].copy()
        if imu_acc is None:
            imu_acc = np.zeros(3, dtype=np.float64)

        msg = LowState()
        msg.version[0] = 1
        msg.version[1] = 0
        msg.mode_pr = self.mode_pr
        msg.mode_machine = self.mode_machine
        msg.tick = int(self.tick)

        msg.imu_state.quaternion[:] = [float(v) for v in imu_quat]
        msg.imu_state.gyroscope[:] = [float(v) for v in imu_gyro]
        msg.imu_state.accelerometer[:] = [float(v) for v in imu_acc]
        msg.imu_state.rpy[:] = [0.0, 0.0, 0.0]
        msg.imu_state.temperature = 0

        for i in range(min(self.motor_count, self.msg_motor_count)):
            motor = msg.motor_state[i]
            motor.mode = 0
            motor.q = float(q[i])
            motor.dq = float(dq[i])
            motor.ddq = 0.0
            motor.tau_est = float(tau_est[i])
            motor.temperature[:] = [0, 0]
            motor.vol = 0.0
            motor.sensor[:] = [0, 0]
            motor.motorstate = 0
            motor.reserve[:] = [0, 0, 0, 0]

        msg.wireless_remote[:] = [0] * len(msg.wireless_remote)
        msg.reserve[:] = [0, 0, 0, 0]
        msg.crc = 0

        self.tick = (self.tick + 1) & 0xFFFFFFFF
        self.lowstate_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        if hasattr(self, "sim_thread") and self.sim_thread.is_alive():
            self.sim_thread.join(timeout=1.0)
        if hasattr(self, "viewer_thread") and self.viewer_thread.is_alive():
            self.viewer_thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = ArmSimNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
