Header header              # 每个行为的唯一标识，包括序号、时间戳和frame_id
                           #    (可以使用机器人名称定义，如小胖'fabo')
string name                # 行为名称，与定义好的行为数据库关联
                           #   （如子行为数、执行器必要占用等信息）
string type                # 行为类型
int8 total_phase           # 行为总共包含的子行为数量
int8 current_phase         # 开始执行的子行为位置，如0表示从头开始执行
int8[5] occupancy          # 执行器分配
string target              # 交互对象
float64 target_angle       # 交互对象角度
float64 target_distance    # 交互对象距离
string speech              # 语音内容文本
string rob_emotion         # 机器人心情（与表情关联）
int8 rob_emotion_intensity # 机器人情绪强度