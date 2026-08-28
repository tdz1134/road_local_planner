# 道路单侧边界（感知输出，车体系：x 向前，y 向左）
# frame_id 填车辆坐标系（与 base_link 一致）
std_msgs/Header header
geometry_msgs/Point32[] points   # 一侧边界点序列（当前约 20 个点，长度可变）
float32 confidence               # 整侧置信度 [0,1]，感知不提供时可填 1.0
