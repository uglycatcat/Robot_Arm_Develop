# 控制器线程
# 该线程负责接收用户的手柄/键盘输入
# 对输入进行滤波，平滑等处理，转换为速度，位置信息。接收模式变化相关信号。
# 该线程应当是一个触发式线程，即在接收到输入后改变关键数据trans的值
# 当外部程序调用update函数时，返回当前的trans值
# 该线程应当是一个守护线程，主线程结束后自动结束
# 传递给main函数中内容
import keyboard
import time
import numpy as np
from threading import Thread, Event, Lock

class ArmController:
    def __init__(self):
        """初始化控制器"""
        # 键盘控制参数
        self.delta_pos_step = 0.01 #最大移动速度
        self.delta_rot_step = 0.01 #最大旋转速度
        self.keyboard_filter_alpha = 0.01  # 平滑因子
        # 控制数据（在update中的返回值）
        self.trans_data=np.zeros(3)
        # 控制器线程参数
        self.thread = None
        self.lock = Lock()
        self.running = Event()
        

    def help(self):
        """打印控制信息"""
        print("""
        =============================
        机械臂控制器 基本使用说明
        =============================

        [ 键盘模式 按键控制 ]
        - 机械臂末端平移：
        W / S : 前进 / 后退
        A / D : 左移 / 右移
        Q / E : 上移 / 下移
        
        请确保 MuJoCo 界面处于激活状态，否则键盘输入可能无效。
        """)
        
    def start(self):
        """启动控制器线程"""
        if not self.running.is_set():
            self.running.set()
            self.thread = Thread(target=self._controller_loop, daemon=True)
            self.thread.start()
            print("控制器线程已启动。")

    def stop(self):
        """停止控制器线程"""
        self.running.clear()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        print("控制器线程已停止。")
        
    def _controller_loop(self):
        """主控制循环"""
        while self.running.is_set():

            trans = self.handle_keyboard_input()

            with self.lock:
                self.trans_data = trans

            time.sleep(0.01)
                    
    def handle_keyboard_input(self):
        """处理键盘输入"""
        delta_pos = np.zeros(3)
        delta_rot = np.zeros(3)

        # 处理平移输入
        if keyboard.is_pressed('RIGHT'): delta_pos[0] += self.delta_pos_step
        if keyboard.is_pressed('LEFT'): delta_pos[0] -= self.delta_pos_step
        if keyboard.is_pressed('d'): delta_pos[1] += self.delta_pos_step
        if keyboard.is_pressed('a'): delta_pos[1] -= self.delta_pos_step
        if keyboard.is_pressed('UP'): delta_pos[2] += self.delta_pos_step
        if keyboard.is_pressed('DOWN'): delta_pos[2] -= self.delta_pos_step

        # 处理旋转输入
        if keyboard.is_pressed('o'): delta_rot[0] += self.delta_rot_step
        if keyboard.is_pressed('k'): delta_rot[0] -= self.delta_rot_step
        if keyboard.is_pressed('j'): delta_rot[1] += self.delta_rot_step
        if keyboard.is_pressed('i'): delta_rot[1] -= self.delta_rot_step
        if keyboard.is_pressed('p'): delta_rot[2] += self.delta_rot_step
        if keyboard.is_pressed('l'): delta_rot[2] -= self.delta_rot_step

        return delta_pos
                
    def update_data(self):
        """获取最新的trans数据"""
        with self.lock:
            return np.copy(self.trans_data) if self.trans_data is not None else np.zeros(3)
    
controller=ArmController()