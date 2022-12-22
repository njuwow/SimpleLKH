LKH算法的简单版本, 参考 https://zhuanlan.zhihu.com/p/317685848

# compile
sh run_compile.sh

# test
./simple_LKH  # simple_LKH 为编译出的bin文件; 需要将数据 att48.tsp 放在和 simple_LKH 同一级目录下

# exec
./simple_LKH [tsp_file=<tsp_file>] [k_opt=<k_opt>] [max_trials=<max_trials>]

参数含义:
tsp_file: string, TSP问题的定义文件, 第一行为节点数, 从第二行开始, 每行3个值, 分别表示
          节点编号(从1开始), 节点x坐标, 节点y坐标, 空格分割; 参考 att48.tsp
          默认值: att48.tsp
k_opt: int, 搜索深度, 建议 2 <= k_opt <= 5
          默认值: 2
max_trials: int, 尝试轮数, 每轮都会遍历TSP问题的所有节点, 尝试从每个节点出发寻找更优路径
          默认值: 5

