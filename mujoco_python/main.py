import mujoco as mj
import mujoco.viewer
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
from Controller import controller

class RobotController:
    
    def __init__(self, model_path):
        # 加载模型和场景
        self.model = mj.MjModel.from_xml_path(model_path)
        self.data = mj.MjData(self.model)
        
        # 初始化仿真状态
        mj.mj_forward(self.model, self.data)
        
        # 初始化观察器，使用与simulate命令类似的配置
        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=False,
        )
        # 获取 target_site 的 ID
        self.marker_site_id = self.model.body("target_marker").id
        self.mocap_id = self.model.body_mocapid[self.marker_site_id]
        # 获取末端位置joint7(link7)(actuator7)的id
        self.end_position_id = self.model.body("link7").id
        # 启动控制器线程
        controller.start()
        
    def solve_ik(self, target_pos, target_rot):
        if self.check_singularity():
            print("警告：奇异点检测，放弃此次 IK 计算！")
            return None

        def objective(q):
            for i in range(6):
                self.data.qpos[i] = q[i]
            mj.mj_forward(self.model, self.data)

            pos = self.data.xpos[self.end_position_id]
            pos_err = np.linalg.norm(pos - target_pos)

            current_rot = R.from_matrix(self.data.xmat[self.end_position_id].reshape(3, 3))
            rot_diff = current_rot.inv() * target_rot
            orient_err = np.linalg.norm(rot_diff.as_rotvec())

            return pos_err + 0.5 * orient_err

        constraints = []
        for i in range(6):
            qmin, qmax = self.model.jnt_range[i]
            constraints.append({"type": "ineq", "fun": lambda q, i=i, qmin=qmin: q[i] - qmin})
            constraints.append({"type": "ineq", "fun": lambda q, i=i, qmax=qmax: qmax - q[i]})

        q_init = self.data.qpos[:6].copy()
        res = minimize(objective, q_init, method='SLSQP', constraints=constraints, options={'maxiter': 100})
        
        if not res.success:
            print("IK 求解失败:", res.message)
            return None

        return res
    
    def check_singularity(self):
        """检测奇异点"""
        # 计算雅可比矩阵
        jacobian_pos = np.zeros((3, self.model.nv))
        jacobian_rot = np.zeros((3, self.model.nv))
        mj.mj_jac(self.model, self.data, jacobian_pos, jacobian_rot, self.data.xpos[self.end_position_id], self.end_position_id)

        # 计算雅可比矩阵的秩
        jacobian = np.vstack((jacobian_pos, jacobian_rot))  # 6×n 矩阵
        rank = np.linalg.matrix_rank(jacobian)

        # 如果秩不足6，说明处于奇异点
        return rank < 6
       
    def run(self):
        """主仿真循环"""
        # 记录上次渲染时间
        last_render_time = time.time()
        try:
            # 设置仿真步长
            step_time = self.model.opt.timestep
            
            while self.viewer.is_running():
                
                # 记录循环开始时间
                loop_start = time.time()
                
                # 获取 link7 的位姿（位置 + 四元数）&设置 target_marker（body）的位置和姿态
                self.data.mocap_pos[self.mocap_id] = self.data.xpos[self.end_position_id]
                self.data.mocap_quat[self.mocap_id] = self.data.xquat[self.end_position_id]
                
                current_pos = self.data.xpos[self.end_position_id].copy()
                current_rot = R.from_matrix(self.data.xmat[self.end_position_id].reshape(3, 3))
                
                delta_pos = controller.update_data()
                delta_rot = R.from_euler('xyz', [0, 0, 0])
                
                target_pos = current_pos + delta_pos
                target_rot = delta_rot * current_rot
                
                # 根据当前目标位置求解六个电机的位置
                ik_result = self.solve_ik(target_pos,target_rot)
                
                # 进行一次判断，如果逆解成功就把角度值更新给电机
                if ik_result is not None:
                    for i in range(6):
                        self.data.ctrl[i] = ik_result.x[i]
                
                # 步进仿真
                mj.mj_step(self.model, self.data)
                
                # 同步渲染，保持实时性
                now = time.time()
                elapsed = now - last_render_time
                if elapsed > 1.0/60.0:  # 约60Hz渲染频率
                    self.viewer.sync()
                    last_render_time = now
                
                # 计算并补偿仿真时间与实际时间的差异
                time_until_next_step = step_time - (time.time() - loop_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                
        except KeyboardInterrupt:
            print("Simulation interrupted by user")
        finally:
            # 关闭窗口
            self.viewer.close()
            # 关闭控制器线程
            controller.stop()

if __name__ == "__main__":
    # 定义文件路径
    model_path = "agilex_piper/scene.xml"
    
    # 创建控制器实例并运行仿真
    try:
        print("Starting simulation... (Press Ctrl+C to stop)")
        SimController = RobotController(model_path)
        SimController.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Simulation ended")