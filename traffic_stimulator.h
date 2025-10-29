#pragma once
#include "map_generation.h"
#include "path_finder.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <random>
#include <algorithm>  

template<typename T>
T clamp(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

class TrafficSimulator {
public:
    // 初始化模拟参数
    TrafficSimulator(MapGenerator& gen)
        : map_gen(gen), running(false), c(0.1), threshold(1.0), e(0.5) {
    }

    // 启动模拟线程
    void start() {
        running.store(true);
        simulator_thread = std::thread([this]() {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dist(-20, 20);

            while (running.load()) {
                update_traffic(dist, gen);
                std::this_thread::sleep_for(std::chrono::seconds(10));
            }
            });
    }

    // 获取边的通行时间（线程安全）
    double get_travel_time(const Edge& edge) const {
        std::lock_guard<std::mutex> lock(edges_mutex);
        double x = static_cast<double>(edge.n) / edge.v;
        double f_x = (x <= threshold) ? 1.0 : 1.0 + e * (x - threshold);
        return c * edge.distance * f_x;
    }

    // 更新动态权重
    void update_dynamic_weights(MapGenerator& gen) {
        std::lock_guard<std::mutex> lock(edges_mutex);
        for (auto& edge : gen.get_edges_mutable()) {
            double density = static_cast<double>(edge.n) / edge.v;
            edge.dynamic_weight = edge.distance * (1 + 0.5 * pow(density, 2));
        }
    }

    // 停止模拟
    void stop() {
        running.store(false);
        if (simulator_thread.joinable()) {
            simulator_thread.join();
        }
    }

    // 添加获取mutex的公共方法
    std::mutex& get_edges_mutex() { return edges_mutex; }
    const std::mutex& get_edges_mutex() const { return edges_mutex; }

    mutable std::mutex edges_mutex;
private:
    MapGenerator& map_gen;
    std::atomic<bool> running;
    std::thread simulator_thread;

    // 模拟参数
    const double c;        // 时间系数
    const double threshold; // 拥堵阈值
    const double e;        // 拥堵系数

    // 更新车流量（随机波动）
    void update_traffic(std::uniform_int_distribution<>& dist, std::mt19937& gen) {
        std::lock_guard<std::mutex> lock(edges_mutex);
        for (auto& edge : map_gen.get_edges_mutable()) {
            int delta = dist(gen);
            edge.n = clamp(edge.n + delta, 0, edge.v); // 限制在0到容量之间
            // 更新动态权重
            double density = static_cast<double>(edge.n) / edge.v;
            edge.dynamic_weight = edge.distance * (1 + 0.5 * pow(density, 2));
        }
    }
};