import simpy
import numpy as np
import random
import math
import queue
from simulator.log import logger
from entities.packet import DataPacket
from routing.dsdv.dsdv import Dsdv
from mac.csma_ca import CsmaCa
from mobility.gauss_markov_3d import GaussMarkov3D
from energy.energy_model import EnergyModel
from allocation.channel_assignment import ChannelAssigner
from utils import config
from utils.util_function import has_intersection
from phy.large_scale_fading import sinr_calculator


class Drone:
    """
    Drone implementation

    Drones in the simulation are served as routers. Each drone can be selected as a potential source node, destination
    and relaying node. Each drone needs to install the corresponding routing module, MAC module, mobility module and
    energy module, etc. At the same time, each drone also has its own queue and can only send one packet at a time, so
    subsequent data packets need queuing for queue resources, which is used to reflect the queue delay in the drone
    network

    Attributes:
        simulator: the simulation platform that contains everything
        env: simulation environment created by simpy
        identifier: used to uniquely represent a drone
        coords: the 3-D position of the drone
        start_coords: the initial position of drone
        direction: current direction of the drone
        pitch: current pitch of the drone
        speed: current speed of the drone
        velocity: velocity components in three directions
        direction_mean: mean direction
        pitch_mean: mean pitch
        velocity_mean: mean velocity
        inbox: a "Store" in simpy, used to receive the packets from other drones (calculate SINR)
        buffer: used to describe the queuing delay of sending packet
        transmitting_queue: when the next hop node receives the packet, it should first temporarily store the packet in
                    "transmitting_queue" instead of immediately yield "packet_coming" process. It can prevent the buffer
                    resource of the previous hop node from being occupied all the time
        waiting_list: for reactive routing protocol, if there is no available next hop, it will put the data packet into
                      "waiting_list". Once the routing information bound for a destination is obtained, drone will get
                      the data packets related to this destination, and put them into "transmitting_queue"
        mac_protocol: installed mac protocol (CSMA/CA, ALOHA, etc.)
        mac_process_dict: a dictionary, used to store the mac_process that is launched each time
        mac_process_finish: a dictionary, used to indicate the completion of the process
        mac_process_count: used to distinguish between different "mac_send" processes
        enable_blocking: describe whether the process of waiting for an ACK blocks the delivery of subsequent packets
                         1: stop-and-wait protocol; 0: sliding window (need further implemented)
        routing_protocol: routing protocol installed (GPSR, DSDV, etc.)
        mobility_model: mobility model installed (3-D Gauss-markov, 3-D random waypoint, etc.)
        energy_model: energy consumption model installed
        residual_energy: the residual energy of drone in Joule
        sleep: if the drone is in a "sleep" state, it cannot perform packet sending and receiving operations
        channel_assigner: used to assign sub-channel for transmitting

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2025/4/16
    """

    def __init__(self,  #各个无人机自己    初始化
                 env,   #环境/共享时钟
                 node_id,  #网络--id
                 coords,   #物理--三维矢量坐标
                 speed,    #物理--速率
                 inbox,    #网络--收件箱
                 simulator):   #模拟器，老大
        self.simulator = simulator
        self.env = env
        self.identifier = node_id
        self.coords = coords               #存入当前位置
        self.start_coords = coords         #存入初始位置
        #def 发奖金(金额):  self.我的奖金 = 金额
        """""
        Python                                    Math
        Class                                     集合U  如：高一3班全体学生
        实例self (self.是成员成员访问运算符)        元素u  如：张三
        实例属性 e.g: self.coords                 该元素的特征  如：张三的年龄，张三的肌肉含量
        变量/参数                                 未知数x
        __init__
        """
        self.rng_drone = random.Random(self.identifier + self.simulator.seed)
        #init外的初始化。初始化加入随机数，确保实验的可重复性
        #!!!为什么括号里没有rng_drone？实参从外部传进来的要写在括号里，内部自己计算就可以得出的可以不用写在括号里

        self.direction = self.rng_drone.uniform(0, 2 * np.pi)    #调用NumPy 库里的pi
        self.pitch = self.rng_drone.uniform(-0.05, 0.05)         
        """"为什么要self.rng_drone.？
        python里，由于指针的存在，随机抽样约等于不放回的。为了相互独立互不影响，需要对每个无人机进行单独随机抽样
        使得共享指针变为独立指针
        如果是共享指针就会出现：飞机1抽走了0.123，飞机二就抽不到0.123
        """
        self.speed = speed  # constant speed throughout the simulation  速率恒定，意味着无人机引擎提供的推力大小保持一致。
        self.velocity = [self.speed * math.cos(self.direction) * math.cos(self.pitch),   #Vx
                         self.speed * math.sin(self.direction) * math.cos(self.pitch),   #Vy
                         self.speed * math.sin(self.pitch)]                              #Vz
        #self.velocity是个矩阵列表 v=[Vx,Vy,Vz]
        self.direction_mean = self.direction
        self.pitch_mean = self.pitch
        self.velocity_mean = self.speed
        #在后续飞行中，无人机会左右晃动（波动），但它需要记住它“本来”想往哪个方向飞。mean 就是它的航向参考基准。

        self.inbox = inbox

        self.buffer = simpy.Resource(env, capacity=1)    
        #信息通道设置为1，同一时刻仅一个飞机可以发出/接收信号。模拟竞争和排队
        #buffer：排队缓冲等待发送信号
        self.max_queue_size = config.MAX_QUEUE_SIZE  #从 config 文件读进来的常量，定义了这架飞机“内存”的上限
        self.transmitting_queue = queue.Queue()  
        # 存的是准备发送的数据包，先产生的信号先发出，先收到的信号先执行。Queue 对象的本质定义就是 FIFO（First-In-First-Out）。
        self.waiting_list = []   #尚未进入正式发送排队队列的数据
        """""
        产生数据：无人机把自己的 coords 封成包，通过 transmitting_queue 排队发出去。
        地面站处理：地面站（或另一个无人机）从网络收到这个包。
        计算决策：地面站判断：“1号机太偏右了，往左打 5 度。”
        指令回传：地面站生成一个 指令包，通过网络发回给 1 号机。
        指令解析：1 号机从 inbox 拿到包，执行 self.direction -= 5 * np.pi / 180。
        物理变动：在下一次 update 时，velocity 重新计算，无人机完成物理上的转弯。
        """

        self.mac_protocol = CsmaCa(self)    #类似交警，模拟了无线电物理特性——“先听后说”
        self.mac_process_dict = dict()      #记录已发出
        self.mac_process_finish = dict()    #记录已完成
        self.mac_process_count = 0
        self.enable_blocking = 1  # enable "stop-and-wait" protocol
        #具体流程图见“流程图1”

        self.routing_protocol = Dsdv(self.simulator, self)   #决定了数据包在空中几个无人机间传递跳跃的路径

        self.mobility_model = GaussMarkov3D(self)     #模拟平稳巡航的运动轨迹
        # self.motion_controller = VfMotionController(self)
        """""
        随机分布 (Uniform) vs. 高斯-马尔可夫 (Gauss-Markov)对比详见
        "D:\Git--Ivy\AI-Learning-Log\UavNetSim-master\Ivy_studynote\随机分布 (Uniform) vs. 高斯-马尔可夫 (Gauss-Markov).pdf"
        """
        self.energy_model = EnergyModel(self)    #EnergyModel：能量模型。它会计算：飞 1 秒扣多少电？发一个包扣多少电？
        self.residual_energy = config.INITIAL_ENERGY
        self.sleep = False    #默认为 False。如果电量耗尽，这个状态变 True

        self.channel_assigner = ChannelAssigner(self.simulator, self)
        #ChannelAssigner：信道分配器。如果空域中有多个频率，它负责告诉无人机：
        # “你去用 1 号频道，他去用 2 号频道”，以减少干扰。
        #channel是指有多少条马路，capacity=1指的一条马路同一时间下只能有一条信息传递

        self.env.process(self.generate_data_packet())   #启动“产生数据”进程。这架飞机会按照一定的频率（比如每秒 1 个）源源不断地产生坐标包或业务包
        self.env.process(self.feed_packet())    #启动“喂包（处理）”进程。它会不停地检查 transmitting_queue，一旦里面有东西，就去敲 CsmaCa 的门请求发送
        self.env.process(self.receive())    #启动“监听接收”进程。这架飞机会一直守着自己的天线，一旦有信号飞过来，就把它抓进 inbox 供以后解析

    def generate_data_packet(self, traffic_pattern='Poisson'):  #Possion是形参默认值
        #无人机通信默认为泊松分布：你像个正常人，有时候半天不说话，有时候突然连发五六条。这种“突发性、随机性”的消息产生方式，就是泊松分布。
        """
        Generate one data packet, it should be noted that only when the current packet has been sent can the next
        packet be started. When the drone generates a data packet, it will first put it into the "transmitting_queue",
        the drone reads a data packet from the head of the queue every very short time through "feed_packet()" function.

        Parameters:
            traffic_pattern: characterize the time interval between generating data packets
        通过分析 generate_data_packet 的逻辑，我理解了系统是如何通过 transmitting_queue 
        实现流量平滑(Traffic Shaping)的。这种‘生产’与‘发送’分离的机制，
        使我们能够定量地分析在非平稳随机流（如泊松流）冲击下，
        无人机的端到端时延(End-to-End Latency)。
        这对于低空飞行中实时控制指令的有效性评估具有决定性意义。
        """

        while True:   #构成（死）循环，以下程序并非一次性任务
         """""
         在任何 Python 代码中，while 后面的表达式只要为“真”，循环就会执行。
         由于 True 永远为真，所以这是一个永不停止的循环。
         它的唯一出口： 只有遇到 break、return、yeild或者抛出异常时，循环才会停止。
         如果你在一段普通代码里写 while True 而没有退出条件，你的电脑 CPU 会瞬间飙升到 100%，程序会“卡死”。
         """
         if not self.sleep:   #还有电
                #准备发包的物理时长
                if traffic_pattern == 'Uniform':
                    # the drone generates a data packet every 0.5s with jitter
                    yield self.env.timeout(self.rng_drone.randint(500000, 505000))  
                    #randint 是 Random Integer 的缩写，意思是“随机整数”
                    # 语法： randint(a, b) 会从 [a, b] 这个闭区间内，随机挑一个整数
                    # 出处： 它来自 Python 自带的 random 模块
                    #额外封装self.rng_drone.用以保证每架无人机的独立性
                    #yield self.env.timeout：进程延迟，即暂停多少秒
                elif traffic_pattern == 'Poisson':
                    """
                    The process of generating data packets by nodes follows Poisson distribution, thus the generation 
                    interval of data packets follows exponential distribution
                    翻译：节点（无人机）生成数据包的过程服从泊松分布，因此，数据包生成的时间间隔服从指数分布。
                    泊松分布 (Poisson Distribution)： 描述的是在一段固定时间内，事件发生了多少次（比如 1 秒内发了 5 个包）。
                    指数分布 (Exponential Distribution)： 描述的是两次事件之间经过了多长时间（比如 发完第一个包，要等多久才发第二个包）。
                    在仿真中，我们无法预知未来 1 秒发多少包，我们只能控制“下一次发包要等多久”。所以，代码里使用的是指数分布。
                    """

                    rate = 5  # on average, how many packets are generated in 1s  $\lambda$。意味着平均每秒发 5 个包。
                    yield self.env.timeout(round(self.rng_drone.expovariate(rate) * 1e6))  #*1e6用于将秒转化为无人机物理执行层（仿真器）的微秒
                    #random库中明确：expovariate(lambd) 用于生成指数分布。
                    #round用于取整。微秒必须是整数时间步。

                #数据包已准备好，记录好id
                config.GL_ID_DATA_PACKET += 1  # data packet id  每完成一次上述的timeout就数据包编号+1（数据包记录员），用于追踪数据流
                #用的是global全局变量而非实体变量，不会出现id重合

                # randomly choose a destination
                all_candidate_list = [i for i in range(config.NUMBER_OF_DRONES)]   #创建全集 U = {0, 1, 2, ..., n-1\}，其中n是无人机的总数。
                all_candidate_list.remove(self.identifier)   #从清单中删掉“我（当前这架无人机）”的 ID，构成所有数据包潜在接收者的补集
                dst_id = self.rng_drone.choice(all_candidate_list)         #从刚才那个“非我”清单中，随机抽取一个 ID。
                destination = self.simulator.drones[dst_id]  # obtain the destination drone  根据编号找机器

                # data packet length
                if config.VARIABLE_PAYLOAD_LENGTH:
                #if config.VARIABLE_PAYLOAD_LENGTH: 等同于 if config.VARIABLE_PAYLOAD_LENGTH == True:对错在config文件中已设置
                #1为True，0为False。判断数据包容量是否可变，可在config中调整

                    fluctuation = self.rng_drone.randint(-config.MAXIMUM_PAYLOAD_VARIATION, config.MAXIMUM_PAYLOAD_VARIATION)
                    payload_length = config.AVERAGE_PAYLOAD_LENGTH + fluctuation
                else:
                    payload_length = config.AVERAGE_PAYLOAD_LENGTH  # in bit, 1024 bytes  容量不可变为定值，不考虑波动

                data_packet_length = (config.IP_HEADER_LENGTH + config.MAC_HEADER_LENGTH +
                                      config.PHY_HEADER_LENGTH + payload_length)
                #另外三成包装也消耗容量：网络层的包装：源IP地址和目的IP地址（收寄件地址）+数据链路层的包装：Medium Access Control Header地址（通行证）+物理层最外层的包装
 
                # channel assignment
                channel_id = self.channel_assigner.channel_assign()   #self.channel_assigner用channel_assign方法分发信道资源，并记为channel_id

                pkd = DataPacket(self,
                                 dst_drone=destination,
                                 creation_time=self.env.now,
                                 data_packet_id=config.GL_ID_DATA_PACKET,
                                 data_packet_length=data_packet_length,
                                 simulator=self.simulator,
                                 channel_id=channel_id)
                pkd.transmission_mode = 0  # the default transmission mode of data packet is "unicast" (0) 追加属性，0表示只有特定无人机可接受到信号而不是大喇叭
                #构造函数DataPacket()调用DataPacket类来创建一个数据包实例pkd

                self.simulator.metrics.datapacket_generated_num += 1   #加的就是刚刚创建的pkd

                logger.info('At time: %s (us) ++++ UAV: %s generates a data packet (id: %s, dst: %s)',
                            self.env.now, self.identifier, pkd.packet_id, destination.identifier)
                #日志文件打印信息，包含了当前仿真时间（self.env.now）、发送者 ID、数据包 ID 和目的地 ID。

                pkd.waiting_start_time = self.env.now #增加一个属性，记录它开始排队的时间
                #排队时延（Queuing Delay） = 实际发出时间 - waiting_start_time$$

                if self.transmitting_queue.qsize() < self.max_queue_size:   #检查当前“发送队列”（排队缓冲区）里的包裹数量是否还没达到上限
                    self.transmitting_queue.put(pkd)   #如果还有空位，就把 pkd 实例放进队列里。FIFO
                else:
                    # the drone has no more room for new packets
                    pass   #什么也不做，丢包
         else:  # 此处与 if not self.sleep 对齐
              break  
         
    def blocking(self):    #定义名为blocking的方法，返回一个“布尔值”（True 或 False），告诉无人机现在是否处于被阻塞的状态。
        """
        队头阻塞问题
        The process of waiting for an ACK will block subsequent incoming data packets to simulate the
        "head-of-line blocking problem"
        """

        if self.enable_blocking:   #检查配置中是否开启了“阻塞机制”。
            if not self.mac_protocol.wait_ack_process_finish:   #检查 mac_protocol（MAC协议层）里记录“等待确认进程”的字典/列表是否为空。
                                                                #如果根本没有正在等待的 ACK 进程，说明路是通的，flag = False。

                flag = False  # there is currently no waiting process for ACK
            else:
                # get the latest process status
                final_indicator = list(self.mac_protocol.wait_ack_process_finish.items())[-1]  #如果有等待记录，取最新（最后一次）的等待进程状态。

                if final_indicator[1] == 0:     #最新的状态指示符等于 0，表示“正在等待”
                    flag = True  # indicates that the drone is still waiting
                else:
                    flag = False  # there is currently no waiting process for ACK
        else:
            flag = False

        return flag

    def feed_packet(self):
        """
        It should be noted that this function is designed for those packets which need to compete for wireless channel

        Firstly, all packets received or generated will be put into the "transmitting_queue", every very short
        time, the drone will read the packet in the head of the "transmitting_queue". Then the drone will check
        if the packet is expired (exceed its maximum lifetime in the network), check the type of packet:
        1) data packet: check if the data packet exceeds its maximum re-transmission attempts. If the above inspection
           passes, routing protocol is executed to determine the next hop drone. If next hop is found, then this data
           packet is ready to transmit, otherwise, it will be put into the "waiting_queue".
        2) control packet: no need to determine next hop, so it will directly start waiting for buffer
        """

        while True:
            if not self.sleep:  # if drone still has enough energy to relay packets
                yield self.env.timeout(10)  # for speed up the simulation

                if not self.blocking():
                    if not self.transmitting_queue.empty():
                        packet = self.transmitting_queue.get()  # get the packet at the head of the queue

                        if self.env.now < packet.creation_time + packet.deadline:  # this packet has not expired
                            if isinstance(packet, DataPacket):
                                if packet.number_retransmission_attempt[self.identifier] < config.MAX_RETRANSMISSION_ATTEMPT:
                                    # it should be noted that "final_packet" may be the data packet itself or a control
                                    # packet, depending on whether the routing protocol can find an appropriate next hop
                                    has_route, final_packet, enquire = self.routing_protocol.next_hop_selection(packet)

                                    if has_route:
                                        logger.info('At time: %s (us) ---- UAV: %s obtain the next hop: %s of data'
                                                    ' packet (id: %s)',
                                                    self.env.now, self.identifier, packet.next_hop_id, packet.packet_id)

                                        # in this case, the "final_packet" is actually the data packet
                                        yield self.env.process(self.packet_coming(final_packet))
                                    else:
                                        self.waiting_list.append(packet)
                                        self.remove_from_queue(packet)

                                        if enquire:
                                            # in this case, the "final_packet" is actually the control packet
                                            yield self.env.process(self.packet_coming(final_packet))

                            else:  # control packet but not ack
                                yield self.env.process(self.packet_coming(packet))
                        else:
                            pass  # means dropping this data packet for expiration
            else:  # this drone runs out of energy
                break  # it is important to break the while loop

    def packet_coming(self, pkd):
        """
        When drone has a packet ready to transmit, yield it.

        The requirement of "ready" is:
            1) this packet is a control packet, or
            2) the valid next hop of this data packet is obtained

        Parameter:
            pkd: packet that waits to enter the buffer of drone
        """

        if not self.sleep:
            arrival_time = self.env.now
            logger.info('At time: %s (us) ---- Packet: %s starts waiting for UAV: %s buffer resource',
                        arrival_time, pkd.packet_id, self.identifier)

            with self.buffer.request() as request:
                yield request  # wait to enter to buffer

                logger.info('At time: %s (us) ---- Packet: %s has been added to the buffer of UAV: %s, '
                            'waiting time is: %s',
                            self.env.now, pkd.packet_id, self.identifier, self.env.now - arrival_time)

                pkd.number_retransmission_attempt[self.identifier] += 1

                if pkd.number_retransmission_attempt[self.identifier] == 1:
                    pkd.time_transmitted_at_last_hop = self.env.now

                logger.info('At time: %s (us) ---- Re-transmission attempts of pkd: %s at UAV: %s is: %s',
                            self.env.now, pkd.packet_id, self.identifier,
                            pkd.number_retransmission_attempt[self.identifier])

                # every time the drone initiates a data packet transmission, "mac_process_count" will be increased by 1
                self.mac_process_count += 1

                key=''.join(['mac_send', str(self.identifier), '_', str(pkd.packet_id)])

                mac_process = self.env.process(self.mac_protocol.mac_send(pkd))
                self.mac_process_dict[key] = mac_process
                self.mac_process_finish[key] = 0

                yield mac_process
        else:
            pass

    def remove_from_queue(self, data_pkd):
        """
        After receiving the ack packet, drone should remove the data packet that has been acked from its queue

        Parameter:
            data_pkd: the acked data packet
        """
        temp_queue = queue.Queue()

        while not self.transmitting_queue.empty():
            pkd_entry = self.transmitting_queue.get()
            if pkd_entry != data_pkd:
                temp_queue.put(pkd_entry)

        while not temp_queue.empty():
            self.transmitting_queue.put(temp_queue.get())

    def receive(self):
        """
        Core receiving function of drone
        1. the drone checks its "inbox" to see if there is incoming packet every 5 units (in us) from the time it is
           instantiated to the end of the simulation
        2. update the "inbox" by deleting the inconsequential data packet
        3. then the drone will detect if it receives a (or multiple) complete data packet(s)
        4. SINR calculation
        """

        while True:
            if not self.sleep:
                # delete packets that have been processed and do not interfere with
                # the transmission and reception of all current packets
                self.update_inbox()

                flag, all_drones_send_to_me, time_span, potential_packet = self.trigger()

                if flag:
                    # find the transmitters of all packets currently transmitted on the channel
                    transmitting_node_list = []
                    for drone in self.simulator.drones:
                        for item in drone.inbox:
                            packet = item[0]
                            insertion_time = item[1]
                            transmitter = item[2]
                            channel_used = item[4]
                            transmitting_time = packet.packet_length / config.BIT_RATE * 1e6
                            interval = [insertion_time, insertion_time + transmitting_time]

                            for interval2 in time_span:
                                if has_intersection(interval, interval2):
                                    transmitting_node_list.append([transmitter, channel_used])

                    # remove duplicates
                    transmitting_node_list = [list(x) for x in {tuple(i) for i in transmitting_node_list}]

                    sinr_list = sinr_calculator(self, all_drones_send_to_me, transmitting_node_list)

                    # receive the packet of the transmitting node corresponding to the maximum SINR
                    max_sinr = max(sinr_list)
                    if max_sinr >= config.SNR_THRESHOLD:
                        which_one = sinr_list.index(max_sinr)

                        pkd = potential_packet[which_one]

                        if pkd.get_current_ttl() < config.MAX_TTL:
                            sender = all_drones_send_to_me[which_one][0]

                            logger.info('At time: %s (us) ---- Packet %s from UAV: %s is received by UAV: %s, sinr is: %s',
                                        self.env.now, pkd.packet_id, sender, self.identifier, max_sinr)

                            yield self.env.process(self.routing_protocol.packet_reception(pkd, sender))
                        else:
                            logger.info('At time: %s (us) ---- Packet %s is dropped due to exceeding max TTL',
                                        self.env.now, pkd.packet_id)
                    else:  # sinr is lower than threshold
                        pass

                yield self.env.timeout(5)
            else:
                break

    def update_inbox(self):
        """
        Clear the packets that have been processed.
                                           ↓ (current time step)
                              |==========|←- (current incoming packet p1)
                       |==========|←- (packet p2 that has been processed, but also can affect p1, so reserve it)
        |==========|←- (packet p3 that has been processed, no impact on p1, can be deleted)
        --------------------------------------------------------> time
        """

        if config.VARIABLE_PAYLOAD_LENGTH:
            max_transmission_time = ((config.AVERAGE_PAYLOAD_LENGTH + config.MAXIMUM_PAYLOAD_VARIATION)
                                     / config.BIT_RATE) * 1e6  # for a single data packet
        else:
            max_transmission_time = (config.AVERAGE_PAYLOAD_LENGTH / config.BIT_RATE) * 1e6  # for a single data packet

        for item in self.inbox:
            insertion_time = item[1]  # the moment that this packet begins to be sent to the channel
            received = item[3]  # used to indicate if this packet has been processed (1: processed, 0: unprocessed)
            if insertion_time + 2 * max_transmission_time < self.env.now:  # no impact on the current packet
                if received:
                    self.inbox.remove(item)

    def trigger(self):
        """
        Detects whether the drone has received a complete data packet

        Returns:
            flag: bool variable, "1" means a complete data packet has been received by this drone and vice versa
            all_drones_send_to_me: a nested list, whose element is a list including sender id and the channel id
            time_span: a nested list, whose element is a list including the time when the packet is transmitted and the
                time when the packet reached
            potential_packet: a list, including all the instances of the received complete data packet
        """

        flag = 0  # used to indicate if I receive a complete packet
        all_drones_send_to_me = []
        time_span = []
        potential_packet = []

        for item in self.inbox:
            packet = item[0]  # not sure yet whether it has been completely transmitted
            insertion_time = item[1]  # transmission start time
            transmitter = item[2]
            processed = item[3]  # indicate if this packet has been processed
            channel_used = item[4]  # indicate the sub-channel that used to transmit this packet

            transmitting_time = packet.packet_length / config.BIT_RATE * 1e6  # expected transmission time

            if not processed:  # this packet has not been processed yet
                if self.env.now >= insertion_time + transmitting_time:  # it has been transmitted completely
                    flag = 1
                    all_drones_send_to_me.append([transmitter, channel_used])
                    time_span.append([insertion_time, insertion_time + transmitting_time])
                    potential_packet.append(packet)
                    item[3] = 1
                else:
                    pass
            else:
                pass

        return flag, all_drones_send_to_me, time_span, potential_packet
