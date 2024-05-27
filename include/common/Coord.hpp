#pragma once

template<typename T>
struct Coord {
    T x, y;

    Coord(T x, T y) : x(x), y(y) {};
    Coord() { x = 0; y = 0; };
    Coord(const Coord& a) : x(a.x), y(a.y) {};

    void reInit(T x, T y) {
        this->x = x;
        this->y = y;
    }

    Coord& operator=(const Coord& a) {
        x = a.x;
        y = a.y;

        return *this;
    }

    Coord operator+(const Coord& a) const {
        return Coord{ x + a.x, y + a.y };
    }

    Coord operator-(const Coord& a) const {
        return Coord{ x - a.x, y - a.y };
    }

    bool operator==(const Coord& a) const {
        return (x == a.x && y == a.y);
    }

    bool operator!=(const Coord& a) const {
        return !(*this == a);
    }
};
