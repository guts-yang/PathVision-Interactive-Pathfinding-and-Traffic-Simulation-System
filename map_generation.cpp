#include "map_generation.h"
#include <random>
#include <algorithm>
#include <future>
#include <numeric>
#include <iomanip>
#include <nanoflann.hpp> // 引入 nanoflann 库用于 KD 树
#include <vector>
#include <cmath>

using namespace std;

// 并查集算法中的查找操作，用于查找顶点的根节点，并进行路径压缩
int MapGenerator::find(int u) {
    while (parent[u] != u) {
        parent[u] = parent[parent[u]]; // 路径压缩优化
        u = parent[u];
    }
    return u;
}

// 并查集算法中的合并操作，将两个顶点所在的集合合并为一个
void MapGenerator::unite(int u, int v) {
    parent[find(u)] = find(v);
}

// 计算两个顶点之间的欧几里得距离
double MapGenerator::distance(const Vertex& a, const Vertex& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

// 生成指定数量的顶点，根据正态分布和均匀分布随机生成顶点的坐标
void MapGenerator::generate_vertices(int N, double radius) {
    mt19937 gen(random_device{}());  // 随机数生成器
    normal_distribution<double> center_dist(0.0, radius / 3.0); // 中心区域的正态分布
    uniform_real_distribution<double> edge_dist(-radius, radius); // 边缘区域的均匀分布

    vertices.clear();    // 清空顶点列表
    vertices.reserve(N);    // 预先分配内存
    for (int i = 0; i < N; ++i) {
        double x = (i < N * 0.8) ? center_dist(gen) : edge_dist(gen);    // 根据概率选择分布生成 x 坐标
        double y = (i < N * 0.8) ? center_dist(gen) : edge_dist(gen);    // 根据概率选择分布生成 y 坐标
        vertices.emplace_back(i, x, y, 0); // 创建新的顶点并添加到列表中
    }
}

// 判断两个顶点之间是否可以建立连接，考虑距离和顶点的度等因素
bool MapGenerator::valid_connection(int u, int v, double max_dist) const {
    if (u == v) return false;  // 不能连接自身
    if (distance(vertices[u], vertices[v]) > max_dist) return false;  // 距离超过最大限制
    if (vertices[u].degree >= 4 || vertices[v].degree >= 4) return false;   // 顶点的度超过 4
    
    // 计算顶点 u 周围的边密度
    int uNeighborCount = 0;
    for (const auto& e : edges) {
        if (e.src == u || e.dest == u) {
            uNeighborCount++;
        }
    }
    // 如果边密度过高，减少生成新边的概率
    if (uNeighborCount > 2) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, 1);
        if (dis(gen) > 0.5) {
            return false;
        }
    }

     //增加边的角度约束
    const double min_angle = 30.0 * 3.1415926 / 180.0; // 最小夹角 30 度
    for (const auto& e : edges) {
        if (e.src == u) {
            double dx1 = vertices[e.dest].x - vertices[u].x;
            double dy1 = vertices[e.dest].y - vertices[u].y;
            double dx2 = vertices[v].x - vertices[u].x;
            double dy2 = vertices[v].y - vertices[u].y;
            double dot = dx1 * dx2 + dy1 * dy2;
            double len1 = std::hypot(dx1, dy1);
            double len2 = std::hypot(dx2, dy2);
            double angle = std::acos(dot / (len1 * len2));
            if (angle < min_angle) {
                return false;
            }
        }
    }

    return true;
}

// 使用 KD 树生成边
void MapGenerator::generate_edges_with_kdtree(double max_dist) {
    PointCloud cloud;
    cloud.pts = vertices;

    // 修正模板参数顺序和类型
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloud, double>, // 移除多余的uint32_t参数
        PointCloud,
        2,
        uint32_t
    > my_kd_tree_t;

    // 初始化适配器（参数名称修正）
    my_kd_tree_t index(
        2,          // 维度
        cloud,
        nanoflann::KDTreeSingleIndexAdaptorParams(10) // 保持参数名称一致
    );
    index.buildIndex();

    for (size_t i = 0; i < vertices.size(); ++i) {
        const Vertex& v = vertices[i];
        const double query_pt[2] = { v.x, v.y };

        // 使用正确的参数类型（参考代码中的SearchParameters）
        nanoflann::SearchParameters params(0.0, false); // 参数改为SearchParameters

        std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;

        // 修正radiusSearch调用方式
        const double search_radius_sq = max_dist * max_dist / 5;
        index.radiusSearch(
            query_pt,         // 直接传递数组指针
            search_radius_sq,
            ret_matches,
            params            // 使用修正后的参数类型
        );

        for (const auto& match : ret_matches) {
            const uint32_t j = match.first;
            if (i < j && valid_connection(static_cast<int>(i), static_cast<int>(j), max_dist)) {
                edges.emplace_back(
                    static_cast<int>(i),
                    static_cast<int>(j),
                    distance(vertices[i], vertices[j])
                );
                vertices[i].degree++;
                vertices[j].degree++;
                unite(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }
}

// 移除重复的边，避免图中存在重复的连接
void MapGenerator::remove_duplicates() {
    unordered_set<std::string> seen; // 用于记录已经出现过的边
    vector<Edge> unique_edges; // 存储唯一的边

    for (const auto& e : edges) {
        string key = (e.src < e.dest)
            ? to_string(e.src) + "|" + to_string(e.dest)
            : to_string(e.dest) + "|" + to_string(e.src);

        if (seen.insert(key).second) {
            unique_edges.push_back(e);  // 如果边未出现过，则添加到唯一边列表中
        }
    }
    edges.swap(unique_edges); // 交换边列表，移除重复的边
}

// 确保图的连通性，添加必要的边以保证图是连通的
void MapGenerator::ensure_connectivity(double max_dist) {
    unordered_map<int, vector<int>> components; // 存储每个连通分量的顶点
    for (size_t i = 0; i < vertices.size(); ++i)
        components[find(static_cast<int>(i))].push_back(static_cast<int>(i));

    if (components.size() <= 1) return; // 如果只有一个连通分量，则不需要处理

    vector<pair<double, pair<int, int>>> bridges; // 存储需要添加的桥边
    vector<vector<int>> comp_list; // 存储所有连通分量的顶点列表
    for (auto& comp : components)
        comp_list.push_back(comp.second);

    vector<thread> threads;  // 存储线程
    mutex mtx; // 互斥锁，用于线程安全
    for (size_t i = 0; i < comp_list.size(); ++i) {
        for (size_t j = i + 1; j < comp_list.size(); ++j) {
            threads.emplace_back([&, i, j]() {
                double min_dist = max_dist * 2; // 初始化最小距离
                pair<int, int> best_pair{ -1, -1 }; // 初始化最佳边

                for (int u : comp_list[i]) {
                    for (int v : comp_list[j]) {
                        double d = distance(vertices[u], vertices[v]);
                        if (d < min_dist) {
                            min_dist = d;
                            best_pair = { u, v };
                        }
                    }
                }

                if (best_pair.first != -1) {
                    lock_guard<std::mutex> lock(mtx); // 加锁，确保线程安全
                    bridges.emplace_back(min_dist, best_pair); // 将最佳边添加到桥边列表中
                }
                });
        }
    }
    for (auto& t : threads) t.join(); // 等待所有线程完成

    sort(bridges.begin(), bridges.end()); // 按距离从小到大排序
    for (auto& bridge : bridges) {
        double dist = bridge.first;
        int u = bridge.second.first;
        int v = bridge.second.second;
        if (find(u) != find(v)) {
            edges.emplace_back(u, v, dist); // 添加边
            edges.emplace_back(v, u, dist); // 添加反向边
            unite(u, v); // 合并连通分量
        }
    }
}

// 生成地图所需的顶点和边，根据指定的顶点数量、城市半径和最大边距离生成地图
void MapGenerator::generate(int N, double radius, double max_dist) {
    generate_vertices(N, radius);
    //generate_vertices_grid(N, 100.0); // 网格大小设为100，可调整
    parent.resize(vertices.size()); // 调整并查集数组的大小
    iota(parent.begin(), parent.end(), 0); // 初始化并查集数组

    generate_edges_with_kdtree(max_dist);
    ensure_connectivity(max_dist * 1.5);
    remove_duplicates();
}

// 将生成的地图保存为二进制文件
void MapGenerator::save_binary(const string& filename) {
    ofstream file(filename, ios::binary);
    if (!file) {
        throw std::runtime_error("无法打开文件进行写入: " + filename); // 以二进制模式打开文件
    }

    // 准备文件头
    FileHeader header;
    header.vertex_count = static_cast<uint32_t>(vertices.size());
    header.edge_count = static_cast<uint32_t>(edges.size());

    // 写入文件头
    file.write(reinterpret_cast<const char*>(&header), sizeof(FileHeader));

    // 写入顶点信息
    for (const auto& v : vertices) {
        BinVertex bv{ static_cast<int32_t>(v.id), v.x, v.y };
        file.write(reinterpret_cast<const char*>(&bv), sizeof(BinVertex));
    }

    // 写入边信息
    for (const auto& e : edges) {
        BinEdge be{ static_cast<int32_t>(e.src),
                  static_cast<int32_t>(e.dest),
                  e.distance, e.dynamic_weight, e.v, e.n };
        file.write(reinterpret_cast<const char*>(&be), sizeof(BinEdge));
    }

    if (!file) {
        throw std::runtime_error("文件写入失败: " + filename);
    }
}

// 从二进制文件中加载地图
void MapGenerator::load_binary(const string& filename) {
    ifstream file(filename, ios::binary);
    if (!file) {
        throw std::runtime_error("无法打开文件进行读取: " + filename);
    }

    // 读取文件头
    FileHeader header;
    file.read(reinterpret_cast<char*>(&header), sizeof(FileHeader));

    // 验证文件格式
    if (string(header.magic, 4) != "NAV1") {
        throw runtime_error("无效的文件格式: " + filename);
    }

    // 准备存储空间
    vertices.clear();
    vertices.reserve(header.vertex_count);
    edges.clear();
    edges.reserve(header.edge_count);

    // 读取顶点信息
    for (uint32_t i = 0; i < header.vertex_count; ++i) {
        BinVertex bv;
        file.read(reinterpret_cast<char*>(&bv), sizeof(BinVertex));
        vertices.emplace_back(bv.id, bv.x, bv.y, 0); // 初始化为 0
    }

    // 读取边信息
    for (uint32_t i = 0; i < header.edge_count; ++i) {
        BinEdge be;
        file.read(reinterpret_cast<char*>(&be), sizeof(BinEdge));
        edges.emplace_back(be.src, be.dest, be.distance, be.v);
        edges.back().n = be.n;
        edges.back().dynamic_weight = be.dynamic_weight; // 恢复动态权重

        // 更新顶点的度
        if (be.src < vertices.size()) vertices[be.src].degree++;
        if (be.dest < vertices.size()) vertices[be.dest].degree++;
    }

    // 重建并查集
    parent.resize(vertices.size());
    iota(parent.begin(), parent.end(), 0);
    for (const auto& e : edges) {
        unite(e.src, e.dest);
    }

    if (!file) {
        throw runtime_error("文件读取失败或数据不完整: " + filename);
    }
}

// 将顶点信息保存为文本文件，使用 TSV 格式
void MapGenerator::save_vertices(const string& filename) {
    ofstream out(filename);  // 打开文件
    out << "ID\tX\tY\n";
    for (const auto& v : vertices) {
        out << v.id << "\t" << v.x << "\t" << v.y << "\n";
    }
}

// 将边信息保存为文本文件，使用 TSV 格式
void MapGenerator::save_edges(const string& filename) {
    ofstream out(filename);  // 打开文件
    out << "Source\tDest\tDistance\n";  // 写入表头
    for (const auto& e : edges) {
        out << e.src << "\t" << e.dest << "\t" << e.distance << "\n";  // 写入边信息
    }
}


// 网格分块生成顶点（替代原随机分布）
void MapGenerator::generate_vertices_grid(int N, double grid_size) {
    vertices.clear();
    int rows = static_cast<int>(sqrt(N));
    int cols = (N + rows - 1) / rows; // 确保 cols * rows >= N

    // 计算网格的中心偏移量
    double center_x = (rows - 1) * grid_size / 2.0;
    double center_y = (cols - 1) * grid_size / 2.0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            double x = i * grid_size + grid_size * 0.5 - center_x; // 调整为以 (0, 0) 为中心
            double y = j * grid_size + grid_size * 0.5 - center_y;
            // 添加微小扰动避免完全对齐
            x += (rand() % 1001 - 500) * 0.01; // ±5单位扰动
            y += (rand() % 1001 - 500) * 0.01;
            vertices.emplace_back(vertices.size(), x, y, 0);
        }
    }
    // 如果生成的顶点数量超过 N，删除多余的顶点
    while (vertices.size() > N) {
        vertices.pop_back();
    }
}