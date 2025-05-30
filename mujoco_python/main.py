import mujoco as mj
import mujoco.viewer
import time
from scipy.spatial.transform import Rotation as R

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

if __name__ == "__main__":
    # 定义文件路径
    model_path = "agilex_piper/scene.xml"
    
    # 创建控制器实例并运行仿真
    try:
        print("Starting simulation... (Press Ctrl+C to stop)")
        controller = RobotController(model_path)
        controller.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Simulation ended")