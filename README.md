### lotspeed 开心版

<div align=center>
    <img src="https://github.com/uk0/lotspeed/blob/main/logo.png" width="400" height="400" />
</div>


### branch explanation

* `main`: lotspeed 自动优化版本最新版
* `v3.1`: lotspeed 最小优化算法版本
* `zeta-tcp`: lotspeed zeta-tcp 版本([Appex Networking zeta-tcp](https://appexnetworks.com/wp-content/uploads/2024/02/ZetaTCP-Whitepaper-V2.0.pdf))


* auto install


```bash
# 直接安装最新版
wget -qO- https://raw.githubusercontent.com/uk0/lotspeed/main/install.sh | sudo bash
# zeta-tcp 版本
wget -qO- https://raw.githubusercontent.com/uk0/lotspeed/zeta-tcp/install.sh | sudo bash
# 暴力版本
wget -qO- https://raw.githubusercontent.com/uk0/lotspeed/v3.1/install.sh | sudo bash
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

# 查看日志
dmesg -w

```

### LotSpeed 核心参数配置说明表

| 参数名称 (`sysctl`/`module`)           | 作用说明 (Description)                                        | 单位/换算 (Unit) | 默认值 | 推荐范围 (Ratio/Range) | 调整建议 |
|:-----------------------------------|:----------------------------------------------------------| :--- | :--- | :--- | :--- |
| **`lotserver_rate`**               | **全局物理带宽上限**<br>控制服务器发包的物理天花板，防止撑爆网卡或被运营商QoS。             | **Bytes/sec**<br>100Mbps ≈ 12,500,000 | 125000000<br>(1Gbps) | **物理带宽的 90% - 95%** | **必填项**。设为你的 VPS 物理端口带宽上限（如 100M 口设为 `11500000`）。不要设得比物理带宽大。 |
| **`lotserver_start_rate`**         | **zeta-tcp版本独有，软启动初始速率**<br>新连接建立时的起步速度。保护小带宽客户端不被瞬间流量淹没。 | **Bytes/sec**<br>10Mbps ≈ 1,250,000 | 6250000<br>(50Mbps) | **物理带宽的 30% - 50%** | 对于 100M 口，建议设为 `5000000` (40Mbps) 到 `7500000` (60Mbps)。设太高会导致起步丢包，设太低起步慢。 |
| **`lotserver_gain`**               | **拥塞窗口增益 (Pacing Gain)**<br>倍率因子。决定算法有多“激进”地去抢占带宽。        | **数值 / 10**<br>20 = 2.0倍 | 20 | **15 (1.5倍) - 30 (3.0倍)** | **核心激进参数**。<br>20 (2.0x) 是平衡点；<br>25-30 (2.5x-3.0x) 适合极高丢包线路，但增加重传消耗。 |
| **`lotserver_beta`**               | **丢包退让比例 (Fairness)**<br>当发生严重拥塞必须降速时，保留多少窗口。             | **数值 / 1024**<br>717 ≈ 保留70% | 717 | **614 (60%) - 921 (90%)** | 设得越高，降速越不情愿（越头铁）。<br>推荐 `819` (80%) 适合跨国线路。<br>`921` (90%) 极其激进。 |
| **`lotserver_min_cwnd`**           | **最小拥塞窗口**<br>无论网络多差，窗口绝不低于此值。                            | **Packets (包数)** | 16 | **4 - 64** | 16 是安全值。设为 `32` 或 `64` 可以提高起步速度，但在拥塞时可能加剧丢包。 |
| **`lotserver_max_cwnd`**           | **最大拥塞窗口**<br>窗口的绝对物理上限，防止 Bufferbloat。                   | **Packets (包数)** | 15000 | **5000 - 30000** | 100Mbps 建议 `5000-8000`。<br>1Gbps 建议 `15000-25000`。<br>设太大无意义，会占用内存。 |
| **`lotserver_turbo`**              | **暴力模式 (Turbo)**<br>是否无视所有丢包信号。                           | **0 (关) / 1 (开)** | 0 | **建议 0** | 除非你在进行压力测试，否则不要开。开启后容易被运营商直接断流。 |
| **`lotserver_safe_mode`**          | **zeta-tcp版本独有，安全熔断 (Safe Mode)**<br>是否在丢包率 >15% 时强制介入降速。              | **0 (关) / 1 (开)** | 1 | **建议 1** | 建议始终开启。这是防止 SSH 断连的最后一道防线。 |

### 常用带宽换算表 (Bytes/sec)

| 带宽 (Mbps) | 参数值 (Bytes/s) | 备注 |
| :--- | :--- | :--- |
| 10 Mbps | `1250000` | 适合作为 Start Rate |
| 20 Mbps | `2500000` | |
| 50 Mbps | `6250000` | 默认 Start Rate |
| 100 Mbps | `12500000` | 常见的 VPS 上限 |
| 200 Mbps | `25000000` | |
| 500 Mbps | `62500000` | |
| 1 Gbps | `125000000` | 默认 Global Rate |

### test youtube (v3.1 version) 

>测试前提，服务器1Gbps，客户端100Mbps带宽

<div align=center>
    <img src="https://github.com/uk0/lotspeed/blob/main/v3.1.png" width="1024" height="768" />
</div>


### test youtube (zeta-tcp version)

>测试前提，服务器1Gbps，客户端100Mbps带宽，丢包率 5% ～ 16%

<div align=center>
    <img src="https://github.com/uk0/lotspeed/blob/main/zeta-tcp.png" width="1024" height="768" />
</div>



### changelog
* 已经测试支持 `debian`,`ubunut` 5.x.x ,6.x.x 内核
* 对抗丢包能力提升
* 优化算法细节，提升稳定性


[![Star History Chart](https://api.star-history.com/svg?repos=uk0/lotspeed&type=Date)](https://star-history.com/#uk0/lotspeed&Date)
