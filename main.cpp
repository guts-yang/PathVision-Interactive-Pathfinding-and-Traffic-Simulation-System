#include "map_generation.h"
#include "path_finder.h"
#include "traffic_stimulator.h"
#include "visualization.h"
#include <iostream>
#include <chrono>
#include <limits>
#include <Windows.h>
using namespace std;
int main() {

    SetConsoleOutputCP(CP_UTF8);

    try {
        MapGenerator generator;

        // 地图参数 - 例如10000个顶点
        const int N = 10000;
        const double city_radius = 10000.0;
        const double max_edge_dist = 1000.0;  // 最大边距离

        std::cout << "正在生成地图..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        generator.generate(10000, city_radius, max_edge_dist);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        std::cout << "生成成功，耗时: " << duration.count() << " ms" << std::endl;
        std::cout << "顶点数量: " << generator.vertex_count() << std::endl;
        std::cout << "边的数量: " << generator.edge_count() << std::endl;

        NavigationVisualizer visualizer(generator);
        visualizer.run();

    }
    catch (const std::exception& e) {
        //std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}