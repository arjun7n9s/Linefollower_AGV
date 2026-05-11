std_msgs/Header header
string shop_id           # "shop_1" | "shop_2" | "shop_3"
string material_type     # which material this shop tracks
int32  stock             # current units in stock
int32  capacity          # max units
int32  threshold         # reorder threshold — order triggered when stock <= threshold
string status            # "ok" | "low" | "critical" | "replenishing"
