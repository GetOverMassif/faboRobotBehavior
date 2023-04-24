bool idleState    # 空闲为1，忙碌中为0，当状态改变时，发送此topic。
string hehavior_name  #执行结束的行为，只有当idleState为1时有效。
string person_name  # 用户姓名
string IDtype               # 用户身份
float64 target_angle        # 交互对象角度
float64 target_distance     # 交互对象距离
string person_emotion       # 用户的心情