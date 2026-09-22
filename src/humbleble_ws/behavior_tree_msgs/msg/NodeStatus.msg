string node_name # BTノードの名前
int16 node_id # BT内のどのノードか識別するためのid
byte IDLE=0
byte RUNNING=1
byte SUCCESS=2
byte FAILURE=3
byte status

byte UNDEFINED=0
byte ACTION=1
byte CONDITION=2
byte CONTROL=3
byte DECORATOR=4
byte SUBTREE=5
byte node_type