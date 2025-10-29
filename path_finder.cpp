#include "path_finder.h"
#include <cmath>
#include <unordered_set>
#include<iostream>




double PathFinder::heuristic(int from, int to) const {
    const auto& v1 = vertices[from];
    const auto& v2 = vertices[to];
    const double base_distance = std::hypot(v1.x - v2.x, v1.y - v2.y);

    // 添加动态权重平衡因子（0.5-1.5区间）
    const double dynamic_factor = (traffic_simulator ? 0.8 : 1.0);
    return base_distance * dynamic_factor;
}

std::vector<int> PathFinder::find_path(int start_id, int end_id) const {
    constexpr int MAX_NODES = 100; // 新增路径节点数限制
    int node_count = 0;
    // 验证节点有效性
    if (start_id < 0 || start_id >= vertices.size() ||
        end_id < 0 || end_id >= vertices.size()) {
        last_explored_count = 0;
        return {};
    }

    // 初始化数据结构
    std::priority_queue<Node> open_list;
    std::unordered_map<int, double> g_values;
    std::unordered_map<int, int> came_from;
    std::unordered_set<int> closed_set;
    last_explored_count = 0;  // 重置计数器

    // 初始化起点
    open_list.emplace(start_id, 0.0, heuristic(start_id, end_id));
    g_values[start_id] = 0.0;

    while (!open_list.empty()) {
        // 新增复杂度检查
        if (++last_explored_count > 7000) {  // 根据路网规模调整
            std::cout << "安全机制：终止路径查找，已探索节点数"
                << last_explored_count << std::endl;
            return {};
        }
        // 取出当前最优节点
        Node current = open_list.top();
        open_list.pop();
        // 到达终点
        if (current.id == end_id) {
            std::vector<int> path;
            int node_id = end_id;
            while (node_id != start_id) {
                path.push_back(node_id);
                node_id = came_from[node_id];
            }
            path.push_back(start_id);
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (closed_set.count(current.id)) continue;
        closed_set.insert(current.id);

        // 遍历邻居
        for (const auto& neighbor : get_neighbors(current.id)) {
            int neighbor_id = neighbor.first;
            double cost = neighbor.second;

            if (closed_set.count(neighbor_id)) continue;

            double tentative_g = g_values[current.id] + cost;

            // 发现新节点或找到更优路径
            if (!g_values.count(neighbor_id) || tentative_g < g_values[neighbor_id]) {
                came_from[neighbor_id] = current.id;
                g_values[neighbor_id] = tentative_g;
                double h = heuristic(neighbor_id, end_id);
                open_list.emplace(neighbor_id, tentative_g, h);
            }
        }
    }

    return {}; // 无路径
}