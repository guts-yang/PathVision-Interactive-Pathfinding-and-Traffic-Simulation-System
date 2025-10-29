// quadtree.h
#pragma once
#include <vector>
#include <memory>

struct Bounds {
    float x, y, width, height;

    bool contains(float px, float py) const {
        return px >= x && px <= x + width &&
            py >= y && py <= y + height;
    }

    bool intersects(const Bounds& other) const {
        return !(x > other.x + other.width ||
            x + width < other.x ||
            y > other.y + other.height ||
            y + height < other.y);
    }
};

template<typename T>
class QuadTree {
public:
    QuadTree(const Bounds& bounds, int capacity = 4)
        : bounds(bounds), capacity(capacity) {}

    bool insert(T item, float x, float y);
    std::vector<T> query(const Bounds& range);

private:
    void subdivide();

    Bounds bounds;
    int capacity;
    std::vector<std::pair<T, sf::Vector2f>> points;

    bool divided = false;
    std::unique_ptr<QuadTree> nw, ne, sw, se;
};
