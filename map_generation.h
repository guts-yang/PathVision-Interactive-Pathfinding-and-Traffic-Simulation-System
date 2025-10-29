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
#include <nanoflann.hpp> // 引入 nanoflann 库用于 KD 树

using namespace std;

#define BIN_FILE_MAGIC "NAV1"

// 顶点结构体，用于表示图中的一个顶点
struct Vertex {
    int id;     // 顶点的唯一标识
    double x;   // 顶点的 x 坐标
    double y;   // 顶点的 y 坐标
    int degree;  // 顶点的度，表示连接该顶点的边的数量

    // 构造函数，提供默认参数，用于初始化顶点的属性
    Vertex(int i = -1, double px = 0.0, double py = 0.0, int d = 0)
        : id(i), x(px), y(py), degree(d) {
    }
};

// 边结构体，用于表示图中两个顶点之间的连接
struct Edge {
    int src;    // 边的起始顶点的标识
    int dest;   // 边的终止顶点的标识
    double distance;   // 边的长度，表示两个顶点之间的距离
    double dynamic_weight; // 边的动态权重
    int v;      // 边的容量
    int n = 0;  // 边当前的流量，默认值为 0
    Edge(int s = -1, int d = -1, double dist = 0.0, int capacity = 0)
        : src(s), dest(d), distance(dist), dynamic_weight(dist), v(static_cast<int>(dist / 10) + 2) {
    }
};

// 自定义哈希函数
// 将 pair<int, int> 类型的顶点对转换为哈希值，以便在 unordered_map 中使用
struct PairHash {
    size_t operator()(const pair<int, int>& p) const {
        return static_cast<size_t>(p.first) << 32 |
            static_cast<size_t>(static_cast<unsigned int>(p.second));
    }
};

// 自定义相等比较函数，用于比较两个 pair<int, int> 类型的顶点对是否相等
struct PairEqual {
    bool operator()(const pair<int, int>& lhs, const pair<int, int>& rhs) const {
        return lhs.first == rhs.first && lhs.second == rhs.second;
    }
};

// 用于存储顶点和边的二进制文件结构，用于二进制文件的读写
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
    double dynamic_weight; // 边的动态权重
    int32_t v;
    int32_t n;
};
#pragma pack(pop)

// 文件头结构，用于二进制文件的存储和读取，确保文件格式的一致性
#pragma pack(push, 1)
struct FileHeader {
    char magic[4] = { 'N', 'A', 'V', '1' }; // 4 字节魔数，用于标识文件格式
    uint32_t version = 1;                 // 文件格式版本
    uint32_t vertex_count;                // 顶点的数量
    uint32_t edge_count;                  // 边的数量
};
#pragma pack(pop)

// 定义一个点云结构体，用于 nanoflann KD 树
struct PointCloud {
    std::vector<Vertex> pts;

    // 获取点的数量
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // 获取指定维度上的点坐标
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return pts[idx].x;
        else return pts[idx].y;
    }

    // 可选的边界框计算
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

// 定义 KD 树类型
typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>,
    PointCloud,
    2 /* dim */
> KDTree;

class MapGenerator {
public:
    // 生成地图所需的顶点和边，根据指定的顶点数量、城市半径和最大边距离生成地图
    void generate(int N, double radius, double max_dist);

    // 将生成的地图保存为二进制文件
    void save_binary(const string& filename);
    // 从二进制文件中加载地图
    void load_binary(const string& filename);
    // 将顶点信息保存为文本文件，使用 TSV 格式
    void save_vertices(const string& filename);
    // 将边信息保存为文本文件，使用 TSV 格式
    void save_edges(const string& filename);

    // 获取顶点的数量
    size_t vertex_count() const { return vertices.size(); }
    // 获取边的数量
    size_t edge_count() const { return edges.size(); }

    // 获取顶点和边的常量引用
    const std::vector<Vertex>& get_vertices() const { return vertices; }
    const std::vector<Edge>& get_edges() const { return edges; }
    // 获取边的可变引用
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

        // 修改点：使用正确的参数类型SearchParameters[2,3](@ref)
        index.findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));

        return std::vector<int>(ret_indexes.begin(), ret_indexes.end());
    }

private:
    std::vector<Vertex> vertices;   // 存储所有顶点的信息
    std::vector<Edge> edges;    // 存储所有边的信息
    std::vector<int> parent;  // 用于并查集算法，存储每个顶点的父节点

    // 内部函数
    // 生成指定数量的顶点，根据正态分布和均匀分布随机生成顶点的坐标
    void generate_vertices(int N, double radius);
    // 确保图的连通性，添加必要的边以保证图是连通的
    void ensure_connectivity(double max_dist);
    // 移除重复的边，避免图中存在重复的连接
    void remove_duplicates();

    // 并查集算法
    // 查找顶点的根节点，使用路径压缩优化查找过程
    int find(int u);
    // 合并两个顶点所在的集合，将它们连接到同一个根节点下
    void unite(int u, int v);

    // 计算函数
    // 计算两个顶点之间的距离，使用欧几里得距离公式
    static double distance(const Vertex& a, const Vertex& b);
    // 判断两个顶点之间是否可以建立连接，考虑距离和顶点的度等因素
    bool valid_connection(int u, int v, double max_dist) const;

    // 使用 KD 树生成边
    void generate_edges_with_kdtree(double max_dist);

    void generate_vertices_grid(int N, double grid_size); // 新增网格分块生成顶点

};