# 起飞状态消息
# 通过话题实时发布起飞过程中的状态

uint8 PHASE_IDLE=0          # 空闲状态，未启动起飞
uint8 PHASE_ASCENDING=1     # 上升阶段
uint8 PHASE_DECELERATING=2  # 减速阶段
uint8 PHASE_DESCENDING=3    # 下降调整阶段
uint8 PHASE_STABLE=4        # 稳定悬停
uint8 PHASE_COMPLETE=5      # 起飞完成
uint8 PHASE_FAILED=6        # 起飞失败
uint8 PHASE_TIMEOUT=7       # 起飞超时

uint8 phase                  # 当前阶段
float64 home_altitude        # 起飞点高度 (m)
float64 target_altitude      # 目标高度 (m)
float64 current_altitude     # 当前高度 (m)
float64 altitude_error       # 高度误差 (m)
float64 phase_elapsed_time   # 当前阶段已耗时 (s)
float64 total_elapsed_time   # 总耗时 (s)
string status_message        # 状态消息文本
