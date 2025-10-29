#pragma once
#include <SFML/Graphics.hpp>
#include <unordered_map>
#include <queue>
#include "map_generation.h"
#include "path_finder.h"
#include "traffic_stimulator.h"
#include<iostream>

class NavigationVisualizer {
public:
    NavigationVisualizer(const MapGenerator& map);
    ~NavigationVisualizer();
    void run();

private:
    void handleEvents();
    void update();
    void render();

    void drawMap();
    void drawPath();
    void drawTraffic();
    void drawUI();
    void drawEvents();

    void drawNearestArea();
    void findRelatedEdges();

    void zoomViewAt(sf::Vector2i pixel, float zoom);
    void panView(sf::Vector2f delta);
    int findClosestVertex(float x, float y) const;
    // 查找指定半径内的顶点
    std::vector<int> findVerticesInRadius(float x, float y, float radius) const {
        std::vector<int> nearbyVertices;
        for (const auto& v : mapData.get_vertices()) {
            float dist = std::hypot(v.x - x, v.y - y);
            if (dist <= radius) {
                nearbyVertices.push_back(v.id);
            }
        }
        return nearbyVertices;
    }

    // 查找指定半径内的路径
    std::vector<int> findEdgesInRadius(float x, float y, float radius) const {
        std::vector<int> nearbyEdges;
        for (size_t i = 0; i < mapData.get_edges().size(); ++i) {
            const auto& edge = mapData.get_edges()[i];
            const auto& v1 = mapData.get_vertices()[edge.src];
            const auto& v2 = mapData.get_vertices()[edge.dest];

            // 检查边的两个端点是否都在搜索半径内
            float dist1 = std::hypot(v1.x - x, v1.y - y);
            float dist2 = std::hypot(v2.x - x, v2.y - y);
            if (dist1 <= radius && dist2 <= radius) {
                nearbyEdges.push_back(i);
            }
        }
        return nearbyEdges;
    }

    void drawFocusArea() {
        // 绘制搜索区域
        sf::CircleShape searchArea(searchRadius);
        searchArea.setOutlineColor(sf::Color::Blue);
        searchArea.setOutlineThickness(2);
        searchArea.setPosition(focusPosition.x - searchRadius, focusPosition.y - searchRadius);
        window.draw(searchArea);

        // 绘制搜索区域内的顶点
        auto nearbyVertices = findVerticesInRadius(focusPosition.x, focusPosition.y, searchRadius);
        for (int id : nearbyVertices) {
            const auto& v = mapData.get_vertices()[id];
            sf::CircleShape point(5);
            point.setFillColor(sf::Color::Cyan);
            point.setPosition(v.x - 5, v.y - 5);
            window.draw(point);
        }

        // 绘制搜索区域内的路径和车流量
        auto nearbyEdges = findEdgesInRadius(focusPosition.x, focusPosition.y, searchRadius);
        std::lock_guard<std::mutex> lock(trafficSimulator->get_edges_mutex());
        for (int edgeIndex : nearbyEdges) {
            const auto& edge = mapData.get_edges()[edgeIndex];
            const auto& v1 = mapData.get_vertices()[edge.src];
            const auto& v2 = mapData.get_vertices()[edge.dest];

            // 根据动态权重计算拥堵程度
            double ratio = static_cast<double>(edge.n) / edge.v;
            sf::Color color;

            if (ratio < 0.3) {
                color = sf::Color(128, 0, 128); // 紫色 - 畅通
            }
            else if (ratio < 0.6) {
                color = sf::Color(255, 165, 0); // 橙色 - 缓行
            }
            else {
                color = sf::Color(0, 255, 255); // 青色 - 拥堵
            }

            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(v1.x, v1.y), color),
                sf::Vertex(sf::Vector2f(v2.x, v2.y), color)
            };
            window.draw(line, 2, sf::Lines);
        }
    }

    const MapGenerator& mapData;
    PathFinder pathFinder;
    TrafficSimulator* trafficSimulator;

    sf::RenderWindow window;
    sf::View view;
    sf::Font font;

    // 可视化状态
    std::vector<int> currentPath;
    bool showTraffic = true;
    float currentZoom = 1.0f;
    sf::Vector2f lastPanPosition;

    // UI状态
    int selectedStart = -1;
    int selectedEnd = -1;
    bool isSimulating = false;

    // 性能优化
    sf::Clock frameClock;
    float lastFrameTime = 0;
    int fps = 0;

    // 在 NavigationVisualizer 的私有成员中添加
    sf::Vector2f focusPosition = sf::Vector2f(-10000, -10000); // 默认无效位置
    bool showFocusArea = false;
    float searchRadius = 500.0f; // 搜索区域的半径

    // 新增状态变量
    bool showNearestArea = false;
    sf::Vector2f searchCenter;
    std::vector<int> nearestVertices;
    std::vector<int> relatedEdges;
};