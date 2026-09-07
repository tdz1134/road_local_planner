# 定位质量（外部定位模块输入）
std_msgs/Header header
float32 quality   # [0,1]，越大表示定位越可信。仲裁器按滞回阈值切换模式
