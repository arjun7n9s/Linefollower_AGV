std_msgs/Header header
string order_id          # unique UUID string
string material_type     # "A" | "B" | "C"
string shop_id           # "shop_1" | "shop_2" | "shop_3"
int32  quantity          # units to deliver
string status            # "pending" | "assigned" | "in_progress" | "completed"
string assigned_agv      # "agv_1" | "agv_2" | "agv_3" | ""
