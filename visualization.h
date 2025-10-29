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
    // ����ָ���뾶�ڵĶ���
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

    // ����ָ���뾶�ڵ�·��
    std::vector<int> findEdgesInRadius(float x, float y, float radius) const {
        std::vector<int> nearbyEdges;
        for (size_t i = 0; i < mapData.get_edges().size(); ++i) {
            const auto& edge = mapData.get_edges()[i];
            const auto& v1 = mapData.get_vertices()[edge.src];
            const auto& v2 = mapData.get_vertices()[edge.dest];

            // ���ߵ������˵��Ƿ��������뾶��
            float dist1 = std::hypot(v1.x - x, v1.y - y);
            float dist2 = std::hypot(v2.x - x, v2.y - y);
            if (dist1 <= radius && dist2 <= radius) {
                nearbyEdges.push_back(i);
            }
        }
        return nearbyEdges;
    }

    void drawFocusArea() {
        // ������������
        sf::CircleShape searchArea(searchRadius);
        searchArea.setOutlineColor(sf::Color::Blue);
        searchArea.setOutlineThickness(2);
        searchArea.setPosition(focusPosition.x - searchRadius, focusPosition.y - searchRadius);
        window.draw(searchArea);

        // �������������ڵĶ���
        auto nearbyVertices = findVerticesInRadius(focusPosition.x, focusPosition.y, searchRadius);
        for (int id : nearbyVertices) {
            const auto& v = mapData.get_vertices()[id];
            sf::CircleShape point(5);
            point.setFillColor(sf::Color::Cyan);
            point.setPosition(v.x - 5, v.y - 5);
            window.draw(point);
        }

        // �������������ڵ�·���ͳ�����
        auto nearbyEdges = findEdgesInRadius(focusPosition.x, focusPosition.y, searchRadius);
        std::lock_guard<std::mutex> lock(trafficSimulator->get_edges_mutex());
        for (int edgeIndex : nearbyEdges) {
            const auto& edge = mapData.get_edges()[edgeIndex];
            const auto& v1 = mapData.get_vertices()[edge.src];
            const auto& v2 = mapData.get_vertices()[edge.dest];

            // ���ݶ�̬Ȩ�ؼ���ӵ�³̶�
            double ratio = static_cast<double>(edge.n) / edge.v;
            sf::Color color;

            if (ratio < 0.3) {
                color = sf::Color(128, 0, 128); // ��ɫ - ��ͨ
            }
            else if (ratio < 0.6) {
                color = sf::Color(255, 165, 0); // ��ɫ - ����
            }
            else {
                color = sf::Color(0, 255, 255); // ��ɫ - ӵ��
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

    // ���ӻ�״̬
    std::vector<int> currentPath;
    bool showTraffic = true;
    float currentZoom = 1.0f;
    sf::Vector2f lastPanPosition;

    // UI״̬
    int selectedStart = -1;
    int selectedEnd = -1;
    bool isSimulating = false;

    // �����Ż�
    sf::Clock frameClock;
    float lastFrameTime = 0;
    int fps = 0;

    // �� NavigationVisualizer ��˽�г�Ա�����
    sf::Vector2f focusPosition = sf::Vector2f(-10000, -10000); // Ĭ����Чλ��
    bool showFocusArea = false;
    float searchRadius = 500.0f; // ��������İ뾶

    // ����״̬����
    bool showNearestArea = false;
    sf::Vector2f searchCenter;
    std::vector<int> nearestVertices;
    std::vector<int> relatedEdges;
};