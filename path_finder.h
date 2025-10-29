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
	//�ڵ�ṹ����A*�㷨
	struct Node {
		int id;
		double g; //����㵽��ǰ�ڵ��ʵ�ʴ���
		double h; //����ʽ���ƴ���
		double f()const { return g + h; }


		Node(int _id, double _g, double _h) : id(_id), g(_g), h(_h) {}

		//���ȶ��бȽϺ���
		bool operator<(const Node& other)const {
			return f() > other.f();  //С����
		}
	};

	explicit PathFinder(const MapGenerator& map, const TrafficSimulator* simulator = nullptr)
		: vertices(map.get_vertices()), edges(map.get_edges()),
		traffic_simulator(simulator) {
	};

	//����·��������
	vector<int> find_path(int staart_id, int end_id)const;

	//��ȡ����ͱ�
	const std::vector<Vertex>& get_vertices() const { return vertices; }
	const std::vector<Edge>& get_edges() const { return edges; }
	vector<pair<int, double>> get_neighbors(int id) const {
		vector<pair<int, double>> neighbors;
		for (const auto& edge : edges) {
			if (edge.src == id || edge.dest == id) {
				// ʹ�ö�̬Ȩ�ش������
				double cost = edge.dynamic_weight;
				neighbors.emplace_back((edge.src == id) ? edge.dest : edge.src, cost);
			}
		}
		return neighbors;
	}
	//��ȡ���ڽڵ㼰����
	mutable int last_explored_count = 0;  // [!code ++]
private:
	const vector<Vertex>& vertices;
	const vector<Edge>& edges;
	const TrafficSimulator* traffic_simulator;


	//����ʽ������ŷ����þ��룩
	double heuristic(int from, int to)const;
};