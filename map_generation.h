#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <fstream>
#include <stdexcept>
#include <nanoflann.hpp> // ���� nanoflann ������ KD ��

using namespace std;

#define BIN_FILE_MAGIC "NAV1"

// ����ṹ�壬���ڱ�ʾͼ�е�һ������
struct Vertex {
    int id;     // �����Ψһ��ʶ
    double x;   // ����� x ����
    double y;   // ����� y ����
    int degree;  // ����Ķȣ���ʾ���Ӹö���ıߵ�����

    // ���캯�����ṩĬ�ϲ��������ڳ�ʼ�����������
    Vertex(int i = -1, double px = 0.0, double py = 0.0, int d = 0)
        : id(i), x(px), y(py), degree(d) {
    }
};

// �߽ṹ�壬���ڱ�ʾͼ����������֮�������
struct Edge {
    int src;    // �ߵ���ʼ����ı�ʶ
    int dest;   // �ߵ���ֹ����ı�ʶ
    double distance;   // �ߵĳ��ȣ���ʾ��������֮��ľ���
    double dynamic_weight; // �ߵĶ�̬Ȩ��
    int v;      // �ߵ�����
    int n = 0;  // �ߵ�ǰ��������Ĭ��ֵΪ 0
    Edge(int s = -1, int d = -1, double dist = 0.0, int capacity = 0)
        : src(s), dest(d), distance(dist), dynamic_weight(dist), v(static_cast<int>(dist / 10) + 2) {
    }
};

// �Զ����ϣ����
// �� pair<int, int> ���͵Ķ����ת��Ϊ��ϣֵ���Ա��� unordered_map ��ʹ��
struct PairHash {
    size_t operator()(const pair<int, int>& p) const {
        return static_cast<size_t>(p.first) << 32 |
            static_cast<size_t>(static_cast<unsigned int>(p.second));
    }
};

// �Զ�����ȱȽϺ��������ڱȽ����� pair<int, int> ���͵Ķ�����Ƿ����
struct PairEqual {
    bool operator()(const pair<int, int>& lhs, const pair<int, int>& rhs) const {
        return lhs.first == rhs.first && lhs.second == rhs.second;
    }
};

// ���ڴ洢����ͱߵĶ������ļ��ṹ�����ڶ������ļ��Ķ�д
#pragma pack(push, 1)
struct BinVertex {
    int32_t id;
    double x;
    double y;
};

struct BinEdge {
    int32_t src;
    int32_t dest;
    double distance;
    double dynamic_weight; // �ߵĶ�̬Ȩ��
    int32_t v;
    int32_t n;
};
#pragma pack(pop)

// �ļ�ͷ�ṹ�����ڶ������ļ��Ĵ洢�Ͷ�ȡ��ȷ���ļ���ʽ��һ����
#pragma pack(push, 1)
struct FileHeader {
    char magic[4] = { 'N', 'A', 'V', '1' }; // 4 �ֽ�ħ�������ڱ�ʶ�ļ���ʽ
    uint32_t version = 1;                 // �ļ���ʽ�汾
    uint32_t vertex_count;                // ���������
    uint32_t edge_count;                  // �ߵ�����
};
#pragma pack(pop)

// ����һ�����ƽṹ�壬���� nanoflann KD ��
struct PointCloud {
    std::vector<Vertex> pts;

    // ��ȡ�������
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // ��ȡָ��ά���ϵĵ�����
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return pts[idx].x;
        else return pts[idx].y;
    }

    // ��ѡ�ı߽�����
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

// ���� KD ������
typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>,
    PointCloud,
    2 /* dim */
> KDTree;

class MapGenerator {
public:
    // ���ɵ�ͼ����Ķ���ͱߣ�����ָ���Ķ������������а뾶�����߾������ɵ�ͼ
    void generate(int N, double radius, double max_dist);

    // �����ɵĵ�ͼ����Ϊ�������ļ�
    void save_binary(const string& filename);
    // �Ӷ������ļ��м��ص�ͼ
    void load_binary(const string& filename);
    // ��������Ϣ����Ϊ�ı��ļ���ʹ�� TSV ��ʽ
    void save_vertices(const string& filename);
    // ������Ϣ����Ϊ�ı��ļ���ʹ�� TSV ��ʽ
    void save_edges(const string& filename);

    // ��ȡ���������
    size_t vertex_count() const { return vertices.size(); }
    // ��ȡ�ߵ�����
    size_t edge_count() const { return edges.size(); }

    // ��ȡ����ͱߵĳ�������
    const std::vector<Vertex>& get_vertices() const { return vertices; }
    const std::vector<Edge>& get_edges() const { return edges; }
    // ��ȡ�ߵĿɱ�����
    std::vector<Edge>& get_edges_mutable() { return edges; }
    std::vector<int> find_nearest_vertices(double x, double y, int count) const{
        PointCloud cloud;
        cloud.pts = vertices;

        typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<double, PointCloud>,
            PointCloud, 2> KDTree;

        KDTree index(2, cloud);
        index.buildIndex();

        const double query_pt[2] = { x, y };
        std::vector<size_t> ret_indexes(count);
        std::vector<double> out_dists_sqr(count);

        nanoflann::KNNResultSet<double> resultSet(count);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

        // �޸ĵ㣺ʹ����ȷ�Ĳ�������SearchParameters[2,3](@ref)
        index.findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));

        return std::vector<int>(ret_indexes.begin(), ret_indexes.end());
    }

private:
    std::vector<Vertex> vertices;   // �洢���ж������Ϣ
    std::vector<Edge> edges;    // �洢���бߵ���Ϣ
    std::vector<int> parent;  // ���ڲ��鼯�㷨���洢ÿ������ĸ��ڵ�

    // �ڲ�����
    // ����ָ�������Ķ��㣬������̬�ֲ��;��ȷֲ�������ɶ��������
    void generate_vertices(int N, double radius);
    // ȷ��ͼ����ͨ�ԣ���ӱ�Ҫ�ı��Ա�֤ͼ����ͨ��
    void ensure_connectivity(double max_dist);
    // �Ƴ��ظ��ıߣ�����ͼ�д����ظ�������
    void remove_duplicates();

    // ���鼯�㷨
    // ���Ҷ���ĸ��ڵ㣬ʹ��·��ѹ���Ż����ҹ���
    int find(int u);
    // �ϲ������������ڵļ��ϣ����������ӵ�ͬһ�����ڵ���
    void unite(int u, int v);

    // ���㺯��
    // ������������֮��ľ��룬ʹ��ŷ����þ��빫ʽ
    static double distance(const Vertex& a, const Vertex& b);
    // �ж���������֮���Ƿ���Խ������ӣ����Ǿ���Ͷ���Ķȵ�����
    bool valid_connection(int u, int v, double max_dist) const;

    // ʹ�� KD �����ɱ�
    void generate_edges_with_kdtree(double max_dist);

    void generate_vertices_grid(int N, double grid_size); // ��������ֿ����ɶ���

};