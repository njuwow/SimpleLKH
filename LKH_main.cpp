#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifndef _LOGUTILS_H_
#define _LOGUTILS_H_

//#define __LOG_DEBUG__
#ifdef __LOG_DEBUG__
#define LOG_DEBUG(format, ...) \
    fprintf(stderr, "DEBUG [%s +%d %s] " format "\n", __FILE__,  __LINE__, __func__, ##__VA_ARGS__);
#else
#define LOG_DEBUG(format, ...)
#endif

#define __LOG_INFO__
#ifdef __LOG_INFO__
#define LOG_INFO(format, ...) \
    fprintf(stderr, "INFO [%s +%d %s] " format "\n", __FILE__,  __LINE__, __func__, ##__VA_ARGS__);
#else
#define LOG_INFO(format, ...)
#endif

#define __LOG_WARNING__
#ifdef __LOG_WARNING__
#define LOG_WARNING(format, ...) \
    fprintf(stderr, "WARNING [%s +%d %s] " format "\n", __FILE__,  __LINE__, __func__, ##__VA_ARGS__);
#else
#define LOG_WARNING(format, ...)
#endif

#define __LOG_ERROR__
#ifdef __LOG_ERROR__
#define LOG_ERROR(format, ...) { \
    fprintf(stderr, "ERROR [%s +%d %s] " format "\n", __FILE__,  __LINE__, __func__, ##__VA_ARGS__); \
    exit(EXIT_FAILURE); \
}
#else
#define LOG_ERROR(format, ...)
#endif

#endif // _LOGUTILS_H_

namespace utils {
template<typename T=std::chrono::microseconds>
class Timer {
public:
    Timer(const std::string &name = "program", const std::string &unit = "micro seconds") :
        _start(std::chrono::high_resolution_clock::now()), _name(name), _unit(unit), _times(1) {
    }

    void set_times(int times) {
        _times = times;
    }

    ~Timer() {
        auto duration = std::chrono::duration_cast<T>(std::chrono::high_resolution_clock::now() - _start);
        LOG_INFO("execution time of \"%s\": %d %s", _name.c_str(), duration.count() / _times, _unit.c_str());
    }
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> _start;
    std::string _name;
    std::string _unit;
    int _times;
};

} // end of namespace utils

struct Point2D {
    Point2D(double px = 0.0, double py = 0.0) : x(px), y(py) {}

    double x;
    double y;
};

struct Node: public Point2D {
    Node(int id_, double x_ = 0.0, double y_ = 0.0) : id(id_), Point2D(x_, y_) {}
    Node(int id_, const Point2D &p) : id(id_), Point2D(p) {}

    friend bool operator==(const Node &n1, const Node &n2) {
        return n1.id == n2.id;
    }

    int id;
};

struct Edge {
    Edge(int n1, int n2) {
        assert(n1 != n2);
        if (n1 <= n2) {
            first = n1;
            second = n2;
        } else {
            first = n2;
            second = n1;
        }
    }

    friend bool operator==(const Edge &e1, const Edge &e2) {
        return e1.first == e2.first && e1.second == e2.second;
    }

    int first;
    int second;
};

template<>
struct std::hash<Edge> {
    std::size_t operator()(Edge const& e) const noexcept {
        std::size_t h1 = std::hash<size_t>{}(e.first);
        std::size_t h2 = std::hash<size_t>{}(e.second);
        return (h1 ^ (h2 << 1));
    }
};

std::string to_string(std::unordered_set<Edge> &edges) {
    std::string ret = "";
    for (auto &e: edges) {
        ret += "(" + std::to_string(e.first) + "," + std::to_string(e.second) + ")";
    }
    return ret;
}

std::string to_string(std::vector<int> &path) {
    if (path.empty()) {
        return "";
    }
    std::string ret = std::string("n1=") + std::to_string(path[0]);
    for (int i = 1; i < path.size(); ++i) {
        ret += "->n" + std::to_string(i + 1) + "=" + std::to_string(path[i]);
    }
    return ret;
}

class Tour {
public:
    static int ms_num_nodes;  // 问题的节点数
    static std::vector<Node> ms_nodes;  // 问题的节点信息
    static std::vector<std::vector<double>> ms_distances;  // 节点距离矩阵
private:
    // 一条具体的路线
    std::unordered_map<int, std::pair<int, int>> _tour_nodes;  // node -> (prev_node, next_node)
    std::unordered_set<Edge> _tour_edges;
    int _head_node;
public:
    static bool init_static(const std::vector<Point2D> &points) {
        if (points.empty()) {
            return false;
        }
        ms_num_nodes = points.size();
        ms_nodes.clear();
        ms_distances = std::vector<std::vector<double>>(ms_num_nodes, std::vector<double>(ms_num_nodes, 0.));
        for (int i = 0; i < ms_num_nodes; ++i) {
            ms_nodes.emplace_back(i, points[i]);
            for (int j = 0; j < i; ++j) {
                double dx = points[i].x - points[j].x;
                double dy = points[i].y - points[j].y;
                ms_distances[i][j] = ms_distances[j][i] = sqrt(dx * dx + dy * dy);
            }
        }
        return true;
    }

    void init() {
        _tour_nodes.clear();
        _tour_edges.clear();
        for (int i = 0; i < ms_num_nodes; ++i) {
            _tour_nodes[i] = std::make_pair((i - 1 + ms_num_nodes) % ms_num_nodes, (i + 1) % ms_num_nodes);
            _tour_edges.insert(Edge(i, (i + 1) % ms_num_nodes));
        }
        _head_node = 0;
    }

    double calc_cost() {
        double cost = 0.;
        for (auto &e: _tour_edges) {
            cost += ms_distances[e.first][e.second];
        }
        return cost;
    }

    static double calc_cost(const std::unordered_set<Edge> &edges) {
        double cost = 0.;
        for (auto &e: edges) {
            cost += ms_distances[e.first][e.second];
        }
        return cost;
    }

    // 判断现有边的集合是否可构成一条有效的路线, 新路线存放在入参new_tour中
    bool is_valid_tour(std::unordered_map<int, std::pair<int, int>> &new_tour) const {
        assert(_tour_edges.size() == ms_nodes.size());
        new_tour.clear();
        int cur_node = _head_node;
        std::unordered_set<Edge> edges = _tour_edges;
        while (!edges.empty()) {
            bool found_next_node = false;
            for (auto &e: edges) {
                int next = cur_node == e.first ? e.second : (cur_node == e.second ? e.first : -1);
                if (next != -1) {
                    new_tour[cur_node].second = next;
                    new_tour[next].first = cur_node;
                    cur_node = next;
                    edges.erase(e);
                    found_next_node = true;
                    break;
                }
            }
            if (!found_next_node) {
                return false;
            }
            if (cur_node == _head_node) {
                break;
            }
        }
        return new_tour.size() == ms_nodes.size();
    }

    // 从原路线中删除集合remove_edges中的边, 添加集合add_edges中的边, 并尝试重新连成一条路线; 返回是否成功
    bool try_relink_tour(const std::unordered_set<Edge> &remove_edges, const std::unordered_set<Edge> &add_edges) {
        if (remove_edges.size() != add_edges.size()) {
            LOG_ERROR("remove_edges.size() = %zu, add_edges.size() = %zu", remove_edges.size(), add_edges.size());
        }
        std::unordered_set<Edge> snapshot = _tour_edges; // 保存原有路线
        for (const auto &e: remove_edges) {
            auto it = _tour_edges.find(e);
            if (it == _tour_edges.end()) {
                return false;
            }
            _tour_edges.erase(it);
        }
        for (const auto &e: add_edges) {
            if (_tour_edges.find(e) != _tour_edges.end()) {
                return false;
            }
            _tour_edges.insert(e);
        }
        std::unordered_map<int, std::pair<int, int>> tour_nodes;
        if (is_valid_tour(tour_nodes)) {
            _tour_nodes = tour_nodes;
            return true;
        } else {
            _tour_edges = snapshot;
            return false;
        }
    }

    std::string to_string(bool reverse = false) {
        std::string tour_string = std::to_string(_head_node);
        int cur_node = _head_node;
        int next_node;
        int counter = 0;
        do {
            counter += 1;
            if (counter > ms_num_nodes) {
                fprintf(stderr, "head_node=%d, tour=\n", _head_node);
                for (auto &mit: _tour_nodes) {
                    fprintf(stderr, "  cur_node=%d, prev_node=%d, next_node=%d\n",
                            mit.first,
                            mit.second.first,
                            mit.second.second);
                }
                LOG_ERROR("invalid tour, please check!");
            }
            next_node = reverse ? _tour_nodes[cur_node].first : _tour_nodes[cur_node].second;
            tour_string += "->";
            tour_string += std::to_string(next_node);
        } while ((cur_node = next_node) != _head_node);
        return tour_string;
    }

    inline bool edge_in_tour(const Edge &e) const {
        return _tour_edges.find(e) != _tour_edges.end();
    }

    std::vector<int> get_adjacent_nodes(int node_id) {
        auto nodes = _tour_nodes.at(node_id);
        return {nodes.first, nodes.second};
    }
};

// 算法思路: 维护两个边的集合：remove_edges (待删除的边集合) 和 add_edges (待新增的边集合)
//
// 1. 选择搜索起点n1
// 2. 选原有路线中n1的前驱或者后继节点n2，组成第一条待删除的边(n1,n2), 放入remove_edges
// 3. 选n2的候选节点n3, 将边(n2,n3)放入add_edges, 要求(n2,n3)满足
//     (1) 不属于原路径上的边
//     (2) 不在 remove_edges 中
// 4. 选择原有路线中n3的前驱或者后继节点n4，边(n3,n4)放入remove_edges
// 5. 假设将n4和n1连接, 即(n1,n4)放入add_edges, 如果将原有路线中的边(n1,n2)和(n3,n4)替换
//    为(n1,n4)和(n2,n3), 替换后能形成一条有效路径, 且
//        distance(n1,n2) + distance(n3,n4) > distance(n1,n4) + distance(n2,n3),
//    则得到一条新的更优路径
// 6. 否则从add_edges中剔除(n1,n4), 从n4出发, 按照3,4的步骤重新找待添加的边(n4,n5)和
//    待删除的边(n5,n6), 并按照步骤5判断连接(n1,n6)是否可形成一条新的有效路径
//
//    当一直找不到新的更优路径时, remove_edges和add_edges会逐渐增大, 计算复杂度也越大,
//    需要设置终止条件, 一般当 remove_edges/add_edges 超过5时退出搜索.
class TSP {
private:
    std::vector<int> _search_path;  // LKH算法的搜索路径
    std::unordered_set<Edge> _remove_edges; // set of edges to be removed
    std::unordered_set<Edge> _add_edges; // set of edges to be added
    Tour _tour;
    std::vector<int> _node_indices;
    int _k_opt;
    int _max_trials;
    int _trial;  // 当前尝试轮数
    int _step;  // 当前搜索节点数
    int _cumulative_gain; // The positive gain criterion. It is required that yi is always chosen
                          // so that the gain, Gi, from the proposed set of exchanges is positive.
                          // Suppose gi = c(xi) - c(yi) is the gain from exchanging xi with yi.
                          // Then Gi is the sum g1 + g2 + ... + gi.
public:
    TSP(const std::vector<Point2D> &points, int k_opt = 2) {
        Tour::init_static(points);
        _tour.init();
        _node_indices = std::vector<int>(Tour::ms_num_nodes);
        for (int i = 0; i < Tour::ms_num_nodes; ++i) {
            _node_indices[i] = i;
        }
        _k_opt = k_opt;
    }

    void solve(int max_trials);
    bool try_optimize(int n1, Tour &new_tour);
    bool search_from_x(int n2, int n1, Tour &new_tour);
    bool search_from_y(int n3, int n1, Tour &new_tour);

    double cost() {
        return _tour.calc_cost();
    }

    std::string tour_to_string(bool reverse = false) {
        return _tour.to_string(reverse);
    }
};

void TSP::solve(int max_trials) {
    _max_trials = max_trials;
    int num_nodes = Tour::ms_num_nodes;
    int indices[num_nodes];
    for (int i = 0; i < num_nodes; ++i) {
        indices[i] = i;
    }
    for (_trial = 1; _trial <= max_trials; ++_trial) {
        std::random_shuffle(indices, indices + num_nodes);
        for (_step = 1; _step <= num_nodes; ++_step) {
            Tour better_tour = _tour;
            if (try_optimize(Tour::ms_nodes[indices[_step - 1]].id, better_tour)) {
                LOG_INFO("[trial %d/%d, step %d/%d: better tour found, cost = %lf, tour = %s",
                        _trial,
                        max_trials,
                        _step,
                        num_nodes,
                        better_tour.calc_cost(),
                        better_tour.to_string().c_str());
                _tour = better_tour;
            } else {
                LOG_INFO("[trial %d/%d, step %d/%d: fail to find a better tour", _trial, max_trials, _step, num_nodes);
            }
        }
    }
}

// n1为搜索的起点, 选择在原有路线中n1的前驱或后继节点n2, 组成一条待删除的边加入集合_remove_edges
bool TSP::try_optimize(int n1, Tour &new_tour) {
    _cumulative_gain = 0.;
    _search_path.clear();
    _search_path.emplace_back(n1);
    LOG_DEBUG("search_path add node %d", _search_path.back());
    for (auto &n2: _tour.get_adjacent_nodes(n1)) {
        _remove_edges.clear();
        _add_edges.clear();
        _remove_edges.insert(Edge(n1, n2));
        _search_path.emplace_back(n2);
        LOG_DEBUG("search_path add node %d", _search_path.back());
        if (search_from_x(n2, n1, new_tour)) {  // 找到一条从n1出发的较优路线
            return true;
        }
        LOG_DEBUG("search_path delete node %d", _search_path.back());
        _search_path.pop_back();
    }
    return false;
}

/** 搜索类型1: 从n2开始寻找待新增的边(n2,n3), 转搜索类型2
 *             从n4开始寻找待新增的边(n4,n5), 转搜索类型2
 *             ...
 *
 * n1为起点, 已固定; n2为n1的前驱节点或后继节点; 该函数用来寻找n2的候选节点n3, 满足:
 *
 * 1. (n2, n3)不在原有路线tour中
 * 2. (n2, n3)不在需要删除的边集合 remove_edges 和需要新增的边集合 add_edges 中
 *
 * 运行 search_from_x 时, _remove_edges边的个数比_add_edges边的个数大1
 **/
bool TSP::search_from_x(int n2, int n1, Tour &new_tour) {
    if (_remove_edges.size() != _add_edges.size() + 1) {
        LOG_ERROR("remove_edges.size() = %zu, add_edges.size() = %zu", _remove_edges.size(), _add_edges.size());
    }
    LOG_DEBUG("trial %d/%d, step %d/%d, call search_from_x(%d, %d, ...) starts, search_path:%s",
            _trial, _max_trials, _step, Tour::ms_num_nodes, n2, n1, to_string(_search_path).c_str());
    // 寻找n3, 满足(n2,n3)不在tour中, 且(n2,n3)不在_remove_edges和_add_edges中
    std::random_shuffle(_node_indices.begin(), _node_indices.end());
    double gain = 0.;
    for (auto index: _node_indices) {
        int n3 = _tour.ms_nodes[index].id;
        if (n3 == n1 || n3 == n2) {
            continue;
        }
        Edge e23(n2, n3);  // 不失一般性, 这里只考虑添加新的边(n2,n3), 如果(n2,n3)失败, 则(n2,n4)也会被程序取到
        if (_tour.edge_in_tour(e23) || _remove_edges.find(e23) != _remove_edges.end() || _add_edges.find(e23) != _add_edges.end()) {
            continue;
        }
        gain = Tour::ms_distances[n1][n2] - Tour::ms_distances[n2][n3];
        if (_cumulative_gain + gain <= 0.) {
            continue;
        } else {
            _cumulative_gain += gain;
        }
        _add_edges.insert(e23);
        _search_path.emplace_back(n3);
        LOG_DEBUG("search_path add node %d, non-adjacent to node %d", _search_path.back(), n2);
        if (search_from_y(n3, n1, new_tour)) {
            return true;
        }
        LOG_DEBUG("search_path delete node %d, non-adjacent to node %d", _search_path.back(), n2);
        _search_path.pop_back();
        _add_edges.erase(e23);
        _cumulative_gain -= gain;
    }
    return false;
}

/** 搜索类型2: 从n3开始寻找待删除的边(n3,n4)并判断是否可以连接(n1,n4)
 *             从n5开始寻找待删除的边(n5,n6)并判断是否可以连接(n1,n6)
 *             ...
 *
 * n1为搜索起点; n3为n2的候选节点, 满足 (n1,n2)在原有tour中, 但(n2, n3)不在原有tour中
 *
 * 运行 search_from_y 时, _remove_edges和_add_edges边的个数是一样的
 **/
bool TSP::search_from_y(int n3, int n1, Tour &new_tour) {
    if (_remove_edges.size() != _add_edges.size()) {
        LOG_ERROR("remove_edges.size() = %zu, add_edges.size() = %zu", _remove_edges.size(), _add_edges.size());
    }
    LOG_DEBUG("trial %d/%d, step %d/%d, call search_next_x(%d, %d, ...) starts, search_path:%s",
            _trial, _max_trials, _step, Tour::ms_num_nodes, n3, n1,
            to_string(_search_path).c_str());
    for (auto &n4: _tour.get_adjacent_nodes(n3)) {  // n4为n3的前驱或后继节点 (n3,n4)需要在原路线中
        if (n4 == n1) {  // n4也不可能等于n2, 因为(n2,n3)不在原路线中
            continue;
        }
        Edge e14(n1, n4);  // (n2,n3)已是候选待添加的边, 所以这里只考虑(n1,n4)是否是可选待添加的边
        if (_tour.edge_in_tour(e14)
                || _remove_edges.find(e14) != _remove_edges.end()
                || _add_edges.find(e14) != _add_edges.end()) {
            continue;
        }
        Edge e34(n3, n4);
        if (_remove_edges.find(e34) != _remove_edges.end()) {  // e34已在删除边的集合中
            continue;
        }
        _remove_edges.insert(e34);
        _search_path.emplace_back(n4);
        LOG_DEBUG("search_path add node %d, adjacent to node %d", _search_path.back(), n3);
        LOG_DEBUG("try relink (%d, %d) starts, search_path:%s", n1, n4, to_string(_search_path).c_str());
        _add_edges.insert(e14);
        if (Tour::calc_cost(_remove_edges) > Tour::calc_cost(_add_edges)) {  // 候选边的cost更小
            if (new_tour.try_relink_tour(_remove_edges, _add_edges)) {  // 找到了一条更优的路线
                LOG_DEBUG("try relink (%d, %d) success", n1, n4);
                return true;
            } else {
                LOG_DEBUG("try relink (%d, %d) fails", n1, n4);
            }
        } else {
                LOG_DEBUG("try relink (%d, %d) fails", n1, n4);
        }
        _add_edges.erase(e14);  // 尝试连接(n1,n4)失败, 继续从n4开始探索

        if (_remove_edges.size() <= _k_opt && search_from_x(n4, n1, new_tour)) {
            return true;
        }
        _remove_edges.erase(e34);
        LOG_DEBUG("search_path delete node %d, adjacent to node %d", _search_path.back(), n3);
        _search_path.pop_back();
    }
    return false;
}

/**
 * 第一行为city数 num_cities
 * 从第二行开始, 每行3个值, 分别为 city_id, city_x, city_y, 其中 1 <= city_id <= num_cities
 **/
int tsp_reader(const std::string &filename, std::vector<Point2D> &points) {
    try {
        std::ifstream fin(filename);
        if (!fin.is_open()) {
            LOG_ERROR("open file \"%s\" fails", filename.c_str());
        }
        int num_cities;
        int city_id;
        int city_x;
        int city_y;
        fin >> num_cities;
        for (int i = 0; i < num_cities; ++i) {
            fin >> city_id >> city_x >> city_y; // city_id从1开始
            points.emplace_back(city_x, city_y);
        }
        fin.close();
    } catch (...) {
        LOG_ERROR("read file fails");
    }
    if (points.size() <= 3) {
        LOG_ERROR("read %zu points from tsp_file \"%s\", number of points should be >= 4", points.size(), filename.c_str());
    }
    return EXIT_SUCCESS;
}

int Tour::ms_num_nodes;
std::vector<Node> Tour::ms_nodes;
std::vector<std::vector<double>> Tour::ms_distances;

void opt_att48_tsp() {
    int opt_tour[] = {1, 8, 38, 31, 44, 18, 7, 28, 6, 37,
        19, 27, 17, 43, 30, 36, 46, 33, 20, 47,
        21, 32, 39, 48, 5, 42, 24, 10, 45, 35,
        4, 26, 2, 29, 34, 41, 16, 22, 3, 23,
        14, 25, 13, 11, 12, 15, 40, 9
    };  // att48.tsp的最优解, 节点编号从1开始

    int num_nodes = sizeof(opt_tour) / sizeof(int);
    std::unordered_set<Edge> opt_edges;
    std::string opt_tour_string = "";
    for (int i = 0; i < num_nodes; ++i) {
        opt_edges.insert(Edge(opt_tour[i] - 1, opt_tour[(i + 1) % num_nodes] - 1));
        opt_tour_string += std::to_string(opt_tour[i] - 1);
        opt_tour_string += "->";
    }
    opt_tour_string += "0";  // 闭环路线, 回到起点
    double opt_cost = Tour::calc_cost(opt_edges);
    LOG_INFO("att48.tsp optimal cost = %lf, optimal tour = %s", opt_cost, opt_tour_string.c_str());
}

// usage: ./LKH [tsp_file=<string>] [k_opt=<int>] [max_trials=<int>]
int main(int argc, char *argv[]) {
    utils::Timer<std::chrono::milliseconds> timer("LKH", "milliseconds");
    srand(time(0));
    std::string filename("att48.tsp");
    int k_opt = 2;
    int max_trials = 5;

    if (argc >= 2) {
        char *token;
        for (int i = 1; i < argc; ++i) {
            if (!(token = strtok(argv[i], "="))) {
                continue;
            }
            if (!strcmp(token, "tsp_file")) {
                if (!(token = strtok(0, "="))) {
                    LOG_ERROR("argument error, a string filename is expected! \"tsp_file=\"");
                } else {
                    filename = std::string(token);
                    LOG_DEBUG("read argument tsp_file = %s", filename.c_str());
                }
            } else if (!strcmp(token, "k_opt")) {
                if (!(token = strtok(NULL, "=")) || !sscanf(token, "%d", &k_opt)) {
                    LOG_ERROR("argument error, an integer is expected! \"k_opt=%s\"", token ? token : "");
                } else if (k_opt < 2) {
                    LOG_ERROR("argument error, k_opt >= 2 expected, \"k_opt=%d\"", k_opt);
                } else {
                    LOG_DEBUG("read argument k_opt = %d", k_opt);
                }
            } else if (!strcmp(token, "max_trials")) {
                if (!(token = strtok(NULL, "=")) || !sscanf(token, "%d", &max_trials)) {
                    LOG_ERROR("argument error, an integer is expected! \"max_trials=%s\"", token ? token : "");
                } else if (max_trials < 1) {
                    LOG_ERROR("argument error, max_trials >= 1 expected, \"max_trials=%d\"", max_trials);
                    LOG_DEBUG("read argument max_trials = %d", max_trials);
                }
            } else {
                LOG_ERROR("undefined keyword %s\nusage: ./bin tsp_file=<filename> k_opt=<k_opt> max_trials=<max_trials>", token);
            }
        }
    }

    LOG_INFO("solve TSP problem \"%s\", k_opt = %d, max_trials = %d starts", filename.c_str(), k_opt, max_trials);

    std::vector<Point2D> points;
    tsp_reader(filename, points);

    TSP tsp(points, k_opt);
    LOG_INFO("TSP init cost = %lf", tsp.cost());

    tsp.solve(max_trials);
    LOG_INFO("solve TSP problem \"%s\", k_opt = %d, max_trials = %d ends", filename.c_str(), k_opt, max_trials);
    LOG_INFO("TSP best cost = %lf, best tour found = %s", tsp.cost(), tsp.tour_to_string().c_str());
    LOG_INFO("TSP best cost = %lf, best tour found = %s", tsp.cost(), tsp.tour_to_string(true).c_str());

    if (filename == "att48.tsp") {
        opt_att48_tsp();
    }

    return 0;
}

