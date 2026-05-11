std_msgs/Header header
string agv_id            # "agv_1" | "agv_2" | "agv_3"
string state             # IDLE | GOING_TO_DEPOT | LOADING | GOING_TO_SHOP | UNLOADING | RETURNING
string current_order_id  # "" if idle
string carrying_material # "A" | "B" | "C" | ""
float32 battery_pct      # 0-100
float32 pos_x
float32 pos_y
