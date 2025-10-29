#include "path_finder.h"
#include <cmath>
#include <unordered_set>
#include<iostream>




double PathFinder::heuristic(int from, int to) const {
    const auto& v1 = vertices[from];
    const auto& v2 = vertices[to];
    const double base_distance = std::hypot(v1.x - v2.x, v1.y - v2.y);

    // ��Ӷ�̬Ȩ��ƽ�����ӣ�0.5-1.5���䣩
    const double dynamic_factor = (traffic_simulator ? 0.8 : 1.0);
    return base_distance * dynamic_factor;
}

std::vector<int> PathFinder::find_path(int start_id, int end_id) const {
    constexpr int MAX_NODES = 100; // ����·���ڵ�������
    int node_count = 0;
    // ��֤�ڵ���Ч��
    if (start_id < 0 || start_id >= vertices.size() ||
        end_id < 0 || end_id >= vertices.size()) {
        last_explored_count = 0;
        return {};
    }

    // ��ʼ�����ݽṹ
    std::priority_queue<Node> open_list;
    std::unordered_map<int, double> g_values;
    std::unordered_map<int, int> came_from;
    std::unordered_set<int> closed_set;
    last_explored_count = 0;  // ���ü�����

    // ��ʼ�����
    open_list.emplace(start_id, 0.0, heuristic(start_id, end_id));
    g_values[start_id] = 0.0;

    while (!open_list.empty()) {
        // �������Ӷȼ��
        if (++last_explored_count > 7000) {  // ����·����ģ����
            std::cout << "��ȫ���ƣ���ֹ·�����ң���̽���ڵ���"
                << last_explored_count << std::endl;
            return {};
        }
        // ȡ����ǰ���Žڵ�
        Node current = open_list.top();
        open_list.pop();
        // �����յ�
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

        // �����ھ�
        for (const auto& neighbor : get_neighbors(current.id)) {
            int neighbor_id = neighbor.first;
            double cost = neighbor.second;

            if (closed_set.count(neighbor_id)) continue;

            double tentative_g = g_values[current.id] + cost;

            // �����½ڵ���ҵ�����·��
            if (!g_values.count(neighbor_id) || tentative_g < g_values[neighbor_id]) {
                came_from[neighbor_id] = current.id;
                g_values[neighbor_id] = tentative_g;
                double h = heuristic(neighbor_id, end_id);
                open_list.emplace(neighbor_id, tentative_g, h);
            }
        }
    }

    return {}; // ��·��
}