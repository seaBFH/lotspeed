### lotspeed 开心版


* auto install


```bash
curl -fsSL https://raw.githubusercontent.com/uk0/lotspeed/main/install.sh | sudo bash
#   or
wget -qO- https://raw.githubusercontent.com/uk0/lotspeed/main/install.sh | sudo bash
```


* manual compile and load

```bash

# 下载代码/编译

git clone https://github.com/uk0/lotspeed.git 

cd lotspeed && make

# 加载模块
sudo insmod lotspeed.ko

# 设置为当前拥塞控制算法
sudo sysctl -w net.ipv4.tcp_congestion_control=lotspeed
sudo sysctl -w net.ipv4.tcp_no_metrics_save=1

# 查看是否生效
sysctl net.ipv4.tcp_congestion_control

```


* parameters


```bash
#1. 速率调整 (lotserver_rate)
# 1Gbps (保守)
echo 125000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate

# 5Gbps (平衡)
echo 625000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate

# 10Gbps (默认)
echo 1250000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate

# 20Gbps (激进)
echo 2500000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate

# 40Gbps (极限)
echo 5000000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate


#2. 增益调整 (lotserver_gain)
# 1.0x (无增益)
echo 10 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain

# 1.5x (保守)
echo 15 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain

# 2.5x (平衡)
echo 25 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain

# 3.0x (默认)
echo 30 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain

# 4.0x (激进)
echo 40 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain

# 5.0x (极限)
echo 50 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain

#3. 拥塞窗口限制
# 最小拥塞窗口 (lotserver_min_cwnd)
echo 10 | sudo tee /sys/module/lotspeed/parameters/lotserver_min_cwnd   # 极小
echo 50 | sudo tee /sys/module/lotspeed/parameters/lotserver_min_cwnd   # 默认
echo 100 | sudo tee /sys/module/lotspeed/parameters/lotserver_min_cwnd  # 较大
echo 200 | sudo tee /sys/module/lotspeed/parameters/lotserver_min_cwnd  # 很大

# 最大拥塞窗口 (lotserver_max_cwnd)
echo 1000 | sudo tee /sys/module/lotspeed/parameters/lotserver_max_cwnd   # 保守
echo 5000 | sudo tee /sys/module/lotspeed/parameters/lotserver_max_cwnd   # 平衡
echo 10000 | sudo tee /sys/module/lotspeed/parameters/lotserver_max_cwnd  # 默认
echo 20000 | sudo tee /sys/module/lotspeed/parameters/lotserver_max_cwnd  # 激进
echo 50000 | sudo tee /sys/module/lotspeed/parameters/lotserver_max_cwnd  # 极限
#4. 模式开关

# 自适应模式 (lotserver_adaptive)
echo 1 | sudo tee /sys/module/lotspeed/parameters/lotserver_adaptive  # 开启（默认）
echo 0 | sudo tee /sys/module/lotspeed/parameters/lotserver_adaptive  # 关闭

# 涡轮模式 (lotserver_turbo) - 谨慎使用！
echo 1 | sudo tee /sys/module/lotspeed/parameters/lotserver_turbo  # 开启（无视丢包）
echo 0 | sudo tee /sys/module/lotspeed/parameters/lotserver_turbo  # 关闭（默认）

# 详细日志 (lotserver_verbose)
echo 1 | sudo tee /sys/module/lotspeed/parameters/lotserver_verbose  # 开启（默认）
echo 0 | sudo tee /sys/module/lotspeed/parameters/lotserver_verbose  # 关闭
#5. 预设配置组合
# 保守配置（适合共享网络）
echo 125000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate   # 1Gbps
echo 15 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain          # 1.5x
echo 1 | sudo tee /sys/module/lotspeed/parameters/lotserver_adaptive       # 自适应开
echo 0 | sudo tee /sys/module/lotspeed/parameters/lotserver_turbo          # 涡轮关
echo 50 | sudo tee /sys/module/lotspeed/parameters/lotserver_min_cwnd      # 标准最小
echo 5000 | sudo tee /sys/module/lotspeed/parameters/lotserver_max_cwnd    # 较小最大

# 平衡配置（推荐）
echo 625000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate   # 5Gbps
echo 25 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain          # 2.5x
echo 1 | sudo tee /sys/module/lotspeed/parameters/lotserver_adaptive       # 自适应开
echo 0 | sudo tee /sys/module/lotspeed/parameters/lotserver_turbo          # 涡轮关
echo 50 | sudo tee /sys/module/lotspeed/parameters/lotserver_min_cwnd      # 标准最小
echo 10000 | sudo tee /sys/module/lotspeed/parameters/lotserver_max_cwnd   # 标准最大

# 激进配置（高速网络）
echo 1250000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate  # 10Gbps
echo 40 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain          # 4.0x
echo 1 | sudo tee /sys/module/lotspeed/parameters/lotserver_adaptive       # 自适应开
echo 0 | sudo tee /sys/module/lotspeed/parameters/lotserver_turbo          # 涡轮关
echo 100 | sudo tee /sys/module/lotspeed/parameters/lotserver_min_cwnd     # 较大最小
echo 20000 | sudo tee /sys/module/lotspeed/parameters/lotserver_max_cwnd   # 较大最大

# 极限配置（专线/测试）
echo 2500000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate  # 20Gbps
echo 50 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain          # 5.0x
echo 0 | sudo tee /sys/module/lotspeed/parameters/lotserver_adaptive       # 自适应关
echo 1 | sudo tee /sys/module/lotspeed/parameters/lotserver_turbo          # ⚡涡轮开⚡
echo 200 | sudo tee /sys/module/lotspeed/parameters/lotserver_min_cwnd     # 很大最小
echo 50000 | sudo tee /sys/module/lotspeed/parameters/lotserver_max_cwnd   # 极限最大


```

* 监控和调试

```bash
# 实时查看参数变更日志
sudo dmesg -w | grep lotspeed

# 查看最近的 lotspeed 日志
sudo dmesg | grep lotspeed | tail -20

# 清空 dmesg 并重新监控
sudo dmesg -C && sudo dmesg -w | grep lotspeed

# 检查当前拥塞控制算法
sysctl net.ipv4.tcp_congestion_control

# 切换回 lotspeed（如果被改变）
sudo sysctl -w net.ipv4.tcp_congestion_control=lotspeed

# 一键恢复默认配置
echo 1250000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate  # 10Gbps
echo 30 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain          # 3.0x
echo 1 | sudo tee /sys/module/lotspeed/parameters/lotserver_adaptive       # 自适应开
echo 0 | sudo tee /sys/module/lotspeed/parameters/lotserver_turbo          # 涡轮关
echo 1 | sudo tee /sys/module/lotspeed/parameters/lotserver_verbose        # 日志开
echo 50 | sudo tee /sys/module/lotspeed/parameters/lotserver_min_cwnd      # 最小50
echo 10000 | sudo tee /sys/module/lotspeed/parameters/lotserver_max_cwnd   # 最大10000

# 如果看到频繁丢包，降低激进程度
echo 20 | sudo tee /sys/module/lotspeed/parameters/lotserver_gain  # 降低增益

# 如果性能不足，提高参数
echo 1000000000 | sudo tee /sys/module/lotspeed/parameters/lotserver_rate  # 提高到8Gbps
```



* DMESG -W


```bash
dmesg -w
...
[   31.308153] ╔════════════════════════════════════════════════════════╗
[   31.308167] ║          LotSpeed v2.0 - 锐速复活版                    ║
[   31.308169] ║          Created by uk0 @ 2025-11-17 12:57:26          ║
[   31.308169] ║          Kernel: 6.8.12                                ║
[   31.308172] ╚════════════════════════════════════════════════════════╝
[   31.308175] Initial Parameters:
[   31.308176]   Rate: 10.00 Gbps
[   31.308177]   Gain: 3.0x
[   31.308178]   Min/Max CWND: 50/10000
[   31.308179]   Adaptive: ON | Turbo: OFF | Verbose: ON
[   52.968525] lotspeed: [uk0@2025-11-17 12:57:26] ⚡⚡⚡ TURBO MODE ACTIVATED ⚡⚡⚡
[   52.968534] lotspeed: WARNING: Ignoring ALL congestion signals!


[   67.969370] lotspeed: [uk0@2025-11-17 12:57:26] rate changed: 1250000000 -> 625000000 (5.00 Gbps)

```