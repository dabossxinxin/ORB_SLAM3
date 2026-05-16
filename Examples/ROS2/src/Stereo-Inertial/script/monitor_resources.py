#!/usr/bin/env python3
# monitor_resources.py
# 使用psutil库监控CPU idle和内存占用

import psutil
import time
import csv
import sys
import os
from datetime import datetime
from collections import deque

class SystemMonitor:
    """系统资源监控類"""
    
    def __init__(self, process_name, output_csv="system_monitor.csv", 
                 history_size=60, interval=1.0):
        """
        初始化监控器
        
        Args:
            process_name: 要监控的进程名称
            output_csv: 输出CSV文件路径
            history_size: 历史数据保留数量
            interval: 采样间隔（秒）
        """
        self.process_name = process_name
        self.output_csv = output_csv
        self.interval = interval
        self.running = True
        
        # 历史数据（用于计算平均值等）
        self.history = {
            'cpu_percent': deque(maxlen=history_size),
            'cpu_idle': deque(maxlen=history_size),
            'memory_mb': deque(maxlen=history_size),
            'memory_percent': deque(maxlen=history_size),
        }
        
        # CSV写入器
        self.csv_file = None
        self.csv_writer = None
        
        # 初始化CSV文件
        self._init_csv()
        
        # 获取进程对象
        self.process = None
        self._update_process()
    
    def _init_csv(self):
        """初始化CSV文件"""
        try:
            self.csv_file = open(self.output_csv, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'timestamp', 'cpu_usage(%)', 'cpu_idle(%)', 'cpu_iowait(%)',
                'proc_cpu(%)', 'proc_memory(MB)', 'proc_memory(%)',
                'sys_memory_used(MB)', 'sys_memory_available(MB)', 'sys_memory_percent(%)',
                'cpu_count', 'cpu_freq(MHz)', 'threads'
            ])
            self.csv_file.flush()
        except Exception as e:
            print(f"Error initializing CSV: {e}", file=sys.stderr)
    
    def _update_process(self):
        """更新进程对象"""
        try:
            for proc in psutil.process_iter(['pid', 'name']):
                if self.process_name.lower() in proc.info['name'].lower():
                    self.process = psutil.Process(proc.info['pid'])
                    return
            self.process = None
        except Exception:
            self.process = None
    
    def get_system_cpu_info(self):
        """获取系统CPU信息"""
        try:
            # CPU使用率（不包括idle和iowait）
            cpu_times = psutil.cpu_times_percent()
            cpu_percent = 100 - cpu_times.idle
            
            # CPU idle和iowait
            cpu_idle = cpu_times.idle
            cpu_iowait = cpu_times.iowait
            
            # 存储历史数据
            self.history['cpu_percent'].append(cpu_percent)
            self.history['cpu_idle'].append(cpu_idle)
            
            return {
                'usage': cpu_percent,
                'idle': cpu_idle,
                'iowait': cpu_iowait,
                'count': psutil.cpu_count(),
                'freq': psutil.cpu_freq().current if psutil.cpu_freq() else 0
            }
        except Exception as e:
            print(f"Error getting CPU info: {e}", file=sys.stderr)
            return None
    
    def get_system_memory_info(self):
        """获取系统内存信息"""
        try:
            memory = psutil.virtual_memory()
            return {
                'total_mb': memory.total / 1024 / 1024,
                'used_mb': memory.used / 1024 / 1024,
                'available_mb': memory.available / 1024 / 1024,
                'percent': memory.percent
            }
        except Exception as e:
            print(f"Error getting memory info: {e}", file=sys.stderr)
            return None
    
    def get_process_info(self):
        """获取进程信息"""
        if not self.process:
            self._update_process()
            if not self.process:
                return None
        
        try:
            # 检查进程是否还在运行
            if not self.process.is_running():
                self.process = None
                return None
            
            # 获取进程内存信息（MB）
            memory_info = self.process.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024
            
            # 获取进程CPU使用率
            cpu_percent = self.process.cpu_percent(interval=0.1)
            
            # 获取进程内存百分比
            memory_percent = self.process.memory_percent()
            
            # 线程数
            num_threads = self.process.num_threads()
            
            # 存储历史数据
            self.history['memory_mb'].append(memory_mb)
            self.history['memory_percent'].append(memory_percent)
            
            return {
                'pid': self.process.pid,
                'name': self.process.name(),
                'cpu_percent': cpu_percent,
                'memory_mb': memory_mb,
                'memory_percent': memory_percent,
                'threads': num_threads
            }
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            self.process = None
            return None
        except Exception as e:
            print(f"Error getting process info: {e}", file=sys.stderr)
            return None
    
    def print_status(self, cpu_info, mem_info, proc_info):
        """打印当前状态"""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # ANSI颜色代码
        GREEN = '\033[92m'
        YELLOW = '\033[93m'
        RED = '\033[91m'
        BLUE = '\033[94m'
        RESET = '\033[0m'
        
        # 构建输出字符串
        output = f"\r[{GREEN}{timestamp}{RESET}] "
        
        # CPU信息
        cpu_color = GREEN if cpu_info['usage'] < 50 else (YELLOW if cpu_info['usage'] < 80 else RED)
        output += f"CPU: {cpu_color}{cpu_info['usage']:.1f}%{RESET} "
        
        # 空闲率（绿色表示还有余量）
        idle_color = RED if cpu_info['idle'] < 10 else (YELLOW if cpu_info['idle'] < 30 else GREEN)
        output += f"Idle: {idle_color}{cpu_info['idle']:.1f}%{RESET} "
        
        # I/O等待
        if cpu_info['iowait'] > 5:
            output += f"IOWait: {YELLOW}{cpu_info['iowait']:.1f}%{RESET} | "
        
        # 进程信息
        if proc_info:
            output += f"[{BLUE}PID:{proc_info['pid']}{RESET}] "
            output += f"Process CPU: {proc_info['cpu_percent']:.1f}% | "
            mem_color = GREEN if proc_info['memory_percent'] < 10 else (YELLOW if proc_info['memory_percent'] < 30 else RED)
            output += f"Mem: {mem_color}{proc_info['memory_mb']:.1f} MB ({proc_info['memory_percent']:.1f}%){RESET} | "
            output += f"Threads: {proc_info['threads']}"
        else:
            output += f"{RED}[Process not found]{RESET}"
        
        # 系统内存
        output += f" | Sys Mem: {mem_info['used_mb']:.0f}/{mem_info['total_mb']:.0f} MB ({mem_info['percent']:.1f}%)"
        
        sys.stdout.write(output)
        sys.stdout.flush()
    
    def append_csv(self, cpu_info, mem_info, proc_info):
        """追加数据到CSV文件"""
        if not self.csv_writer:
            return
        
        try:
            timestamp = datetime.now().isoformat()
            
            if proc_info:
                self.csv_writer.writerow([
                    timestamp,
                    f"{cpu_info['usage']:.2f}",
                    f"{cpu_info['idle']:.2f}",
                    f"{cpu_info['iowait']:.2f}",
                    f"{proc_info['cpu_percent']:.2f}",
                    f"{proc_info['memory_mb']:.2f}",
                    f"{proc_info['memory_percent']:.2f}",
                    f"{mem_info['used_mb']:.2f}",
                    f"{mem_info['available_mb']:.2f}",
                    f"{mem_info['percent']:.2f}",
                    cpu_info['count'],
                    f"{cpu_info['freq']:.2f}",
                    proc_info['threads']
                ])
            else:
                self.csv_writer.writerow([
                    timestamp,
                    f"{cpu_info['usage']:.2f}",
                    f"{cpu_info['idle']:.2f}",
                    f"{cpu_info['iowait']:.2f}",
                    "N/A", "N/A", "N/A",
                    f"{mem_info['used_mb']:.2f}",
                    f"{mem_info['available_mb']:.2f}",
                    f"{mem_info['percent']:.2f}",
                    cpu_info['count'],
                    f"{cpu_info['freq']:.2f}",
                    "N/A"
                ])
            
            self.csv_file.flush()
        except Exception as e:
            print(f"Error writing CSV: {e}", file=sys.stderr)
    
    def print_summary(self):
        """打印统计摘要"""
        if not self.history['cpu_percent']:
            return
        
        print("\n" + "="*60)
        print("监控统计汇总")
        print("="*60)
        
        # CPU统计
        cpu_data = list(self.history['cpu_percent'])
        print(f"\nCPU使用率:")
        print(f"  平均: {sum(cpu_data)/len(cpu_data):.1f}%")
        print(f"  最小: {min(cpu_data):.1f}%")
        print(f"  最大: {max(cpu_data):.1f}%")
        
        # 空闲率统计
        idle_data = list(self.history['cpu_idle'])
        print(f"\nCPU空闲率:")
        print(f"  平均: {sum(idle_data)/len(idle_data):.1f}%")
        print(f"  最小: {min(idle_data):.1f}%")
        print(f"  最大: {max(idle_data):.1f}%")
        
        # 内存统计
        if self.history['memory_mb']:
            mem_data = list(self.history['memory_mb'])
            print(f"\n进程内存占用:")
            print(f"  平均: {sum(mem_data)/len(mem_data):.1f} MB")
            print(f"  最小: {min(mem_data):.1f} MB")
            print(f"  最大: {max(mem_data):.1f} MB")
        
        if self.history['memory_percent']:
            mem_pct_data = list(self.history['memory_percent'])
            print(f"\n进程内存百分比:")
            print(f"  平均: {sum(mem_pct_data)/len(mem_pct_data):.2f}%")
            print(f"  最小: {min(mem_pct_data):.2f}%")
            print(f"  最大: {max(mem_pct_data):.2f}%")
        
        print("="*60)
    
    def run(self, duration=None):
        """
        运行监控循环
        
        Args:
            duration: 监控持续时间（秒），None表示持续运行
        """
        print(f"启动系统资源监控")
        print(f"监控进程: {self.process_name}")
        print(f"采样间隔: {self.interval}秒")
        print(f"输出文件: {self.output_csv}")
        print(f"按 Ctrl+C 停止监控\n")
        
        start_time = time.time()
        
        try:
            while self.running:
                # 检查持续时间
                if duration and (time.time() - start_time) > duration:
                    break
                
                # 获取信息
                cpu_info = self.get_system_cpu_info()
                mem_info = self.get_system_memory_info()
                proc_info = self.get_process_info()
                
                if cpu_info and mem_info:
                    # 打印实时状态
                    self.print_status(cpu_info, mem_info, proc_info)
                    
                    # 记录到CSV
                    self.append_csv(cpu_info, mem_info, proc_info)
                
                time.sleep(self.interval)
        
        except KeyboardInterrupt:
            print("\n监控已停止")
        
        finally:
            if self.csv_file:
                self.csv_file.close()
            
            self.print_summary()
            print(f"\n数据已保存到: {self.output_csv}")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='监控ORB_SLAM3的CPU和内存占用'
    )
    parser.add_argument(
        '-p', '--process', 
        default='stereo_inertial_ros2',
        help='要监控的进程名称 (默认: stereo_inertial_ros2)'
    )
    parser.add_argument(
        '-o', '--output',
        default='',
        help='输出CSV文件路径 (默认: 空)'
    )
    parser.add_argument(
        '-i', '--interval',
        type=float,
        default=1.0,
        help='采样间隔（秒） (默认: 1.0)'
    )
    parser.add_argument(
        '-d', '--duration',
        type=int,
        help='监控持续时间（秒），不指定则持续运行'
    )
    
    args = parser.parse_args()
    
    # 检查psutil是否已安装
    try:
        import psutil
    except ImportError:
        print("请先安装psutil库:")
        print("  pip install psutil")
        sys.exit(1)
    
    # 创建并运行监控器
    monitor = SystemMonitor(
        process_name=args.process,
        output_csv=args.output,
        interval=args.interval
    )
    monitor.run(duration=args.duration)

if __name__ == '__main__':
    main()
