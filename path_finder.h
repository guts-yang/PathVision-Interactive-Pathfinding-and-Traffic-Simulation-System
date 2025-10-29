#pragma once
#include"map_generation.h"
#include"traffic_stimulator.h"
#include<queue>
#include<unordered_map>
#include<vector>
#include<functional>
using namespace std;
class PathFinder {
public:
	//节点结构用于A*算法
	struct Node {
		int id;
		double g; //从起点到当前节点的实际代价
		double h; //启发式估计代价
		double f()const { return g + h; }


		Node(int _id, double _g, double _h) : id(_id), g(_g), h(_h) {}

		//优先队列比较函数
		bool operator<(const Node& other)const {
			return f() > other.f();  //小顶堆
		}
	};

	explicit PathFinder(const MapGenerator& map, const TrafficSimulator* simulator = nullptr)
		: vertices(map.get_vertices()), edges(map.get_edges()),
		traffic_simulator(simulator) {
	};

	//查找路径主函数
	vector<int> find_path(int staart_id, int end_id)const;

	//获取顶点和边
	const std::vector<Vertex>& get_vertices() const { return vertices; }
	const std::vector<Edge>& get_edges() const { return edges; }
	vector<pair<int, double>> get_neighbors(int id) const {
		vector<pair<int, double>> neighbors;
		for (const auto& edge : edges) {
			if (edge.src == id || edge.dest == id) {
				// 使用动态权重代替距离
				double cost = edge.dynamic_weight;
				neighbors.emplace_back((edge.src == id) ? edge.dest : edge.src, cost);
			}
		}
		return neighbors;
	}
	//获取相邻节点及距离
	mutable int last_explored_count = 0;  // [!code ++]
private:
	const vector<Vertex>& vertices;
	const vector<Edge>& edges;
	const TrafficSimulator* traffic_simulator;


	//启发式函数（欧几里得距离）
	double heuristic(int from, int to)const;
};