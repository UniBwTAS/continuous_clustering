#ifndef CONTINUOUS_CLUSTERING_GENERAL_HPP
#define CONTINUOUS_CLUSTERING_GENERAL_HPP

namespace continuous_clustering
{

struct Point2D
{
    Point2D() : x(0), y(0){};
    Point2D(float x, float y) : x(x), y(y){};

    float x{};
    float y{};

    inline float length() const
    {
        return std::sqrt(x * x + y * y);
    }

    inline float lengthSquared() const
    {
        return x * x + y * y;
    }

    inline Point2D min(Point2D& other) const
    {
        Point2D min = other;
        if (x < min.x)
            min.x = x;
        if (y < min.y)
            min.y = y;
        return min;
    }

    inline Point2D max(Point2D& other) const
    {
        Point2D max = other;
        if (x > max.x)
            max.x = x;
        if (y > max.y)
            max.y = y;
        return max;
    }

    inline Point2D normed() const
    {
        float l = 1.f / length();
        return {x * l, y * l};
    }
};

inline Point2D operator-(const Point2D& p1, const Point2D& p2)
{
    return {p1.x - p2.x, p1.y - p2.y};
}

inline Point2D operator+(const Point2D& p1, const Point2D& p2)
{
    return {p1.x + p2.x, p1.y + p2.y};
}

inline float operator*(const Point2D& p1, const Point2D& p2)
{
    return p1.x * p2.x + p1.y * p2.y;
}

inline Point2D operator*(const Point2D& p, float s)
{
    return {p.x * s, p.y * s};
}

inline Point2D operator*(float s, const Point2D& p)
{
    return {p.x * s, p.y * s};
}

inline Point2D operator/(const Point2D& p, float s)
{
    return {p.x / s, p.y / s};
}

struct Point3D
{
    Point3D() : x(0), y(0), z(0){};
    Point3D(float x, float y, float z) : x(x), y(y), z(z){};

    float x{};
    float y{};
    float z{};

    inline float length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    inline float lengthSquared() const
    {
        return x * x + y * y + z * z;
    }

    inline float lengthXY() const
    {
        return std::sqrt(x * x + y * y);
    }

    inline float lengthXZ() const
    {
        return std::sqrt(x * x + z * z);
    }

    inline float lengthYZ() const
    {
        return std::sqrt(y * y + z * z);
    }

    inline Point2D xy() const
    {
        return {x, y};
    }

    inline Point2D xz() const
    {
        return {x, z};
    }

    inline Point2D yz() const
    {
        return {y, z};
    }

    inline Point3D min(Point3D& other) const
    {
        Point3D min = other;
        if (x < min.x)
            min.x = x;
        if (y < min.y)
            min.y = y;
        if (z < min.z)
            min.z = z;
        return min;
    }

    inline Point3D max(Point3D& other) const
    {
        Point3D max = other;
        if (x > max.x)
            max.x = x;
        if (y > max.y)
            max.y = y;
        if (z > max.z)
            max.z = z;
        return max;
    }

    inline Point3D normed() const
    {
        float l = 1.f / length();
        return {x * l, y * l, z * l};
    }
};

inline Point3D operator-(const Point3D& p1, const Point3D& p2)
{
    return {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z};
}

inline Point3D operator+(const Point3D& p1, const Point3D& p2)
{
    return {p1.x + p2.x, p1.y + p2.y, p1.z + p2.z};
}

inline float operator*(const Point3D& p1, const Point3D& p2)
{
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

inline Point3D operator*(const Point3D& p, float s)
{
    return {p.x * s, p.y * s, p.z * s};
}

inline Point3D operator*(float s, const Point3D& p)
{
    return {p.x * s, p.y * s, p.z * s};
}

inline Point3D operator/(const Point3D& p, float s)
{
    return {p.x / s, p.y / s, p.z / s};
}

template<typename PointXD>
inline PointXD get_line_direction(PointXD line_start, PointXD line_end)
{
    return (line_end - line_start).normed();
}

template<typename PointXD>
inline float distance_point_from_line(PointXD point, PointXD line_start, PointXD line_direction)
{
    // see e.g.: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Vector_formulation (from 28. July 2023)
    PointXD start_to_point = point - line_start;
    return (start_to_point - (start_to_point * line_direction) * line_direction).length();
}

// this enum corresponds to QColor::colorNames(), where "transparent" (140) was skipped for obvious reasons
// our visualization uses the same color list, so it is easy to assign a color to a point for visualization/debugging
enum PointCloudColors
{
    ALICEBLUE = 0,
    ANTIQUEWHITE = 1,
    AQUA = 2,
    AQUAMARINE = 3,
    AZURE = 4,
    BEIGE = 5,
    BISQUE = 6,
    BLACK = 7,
    BLANCHEDALMOND = 8,
    BLUE = 9,
    BLUEVIOLET = 10,
    BROWN = 11,
    BURLYWOOD = 12,
    CADETBLUE = 13,
    CHARTREUSE = 14,
    CHOCOLATE = 15,
    CORAL = 16,
    CORNFLOWERBLUE = 17,
    CORNSILK = 18,
    CRIMSON = 19,
    CYAN = 20,
    DARKBLUE = 21,
    DARKCYAN = 22,
    DARKGOLDENROD = 23,
    DARKGRAY = 24,
    DARKGREEN = 25,
    DARKGREY = 26,
    DARKKHAKI = 27,
    DARKMAGENTA = 28,
    DARKOLIVEGREEN = 29,
    DARKORANGE = 30,
    DARKORCHID = 31,
    DARKRED = 32,
    DARKSALMON = 33,
    DARKSEAGREEN = 34,
    DARKSLATEBLUE = 35,
    DARKSLATEGRAY = 36,
    DARKSLATEGREY = 37,
    DARKTURQUOISE = 38,
    DARKVIOLET = 39,
    DEEPPINK = 40,
    DEEPSKYBLUE = 41,
    DIMGRAY = 42,
    DIMGREY = 43,
    DODGERBLUE = 44,
    FIREBRICK = 45,
    FLORALWHITE = 46,
    FORESTGREEN = 47,
    FUCHSIA = 48,
    GAINSBORO = 49,
    GHOSTWHITE = 50,
    GOLD = 51,
    GOLDENROD = 52,
    GRAY = 53,
    GREEN = 54,
    GREENYELLOW = 55,
    GREY = 56,
    HONEYDEW = 57,
    HOTPINK = 58,
    INDIANRED = 59,
    INDIGO = 60,
    IVORY = 61,
    KHAKI = 62,
    LAVENDER = 63,
    LAVENDERBLUSH = 64,
    LAWNGREEN = 65,
    LEMONCHIFFON = 66,
    LIGHTBLUE = 67,
    LIGHTCORAL = 68,
    LIGHTCYAN = 69,
    LIGHTGOLDENRODYELLOW = 70,
    LIGHTGRAY = 71,
    LIGHTGREEN = 72,
    LIGHTGREY = 73,
    LIGHTPINK = 74,
    LIGHTSALMON = 75,
    LIGHTSEAGREEN = 76,
    LIGHTSKYBLUE = 77,
    LIGHTSLATEGRAY = 78,
    LIGHTSLATEGREY = 79,
    LIGHTSTEELBLUE = 80,
    LIGHTYELLOW = 81,
    LIME = 82,
    LIMEGREEN = 83,
    LINEN = 84,
    MAGENTA = 85,
    MAROON = 86,
    MEDIUMAQUAMARINE = 87,
    MEDIUMBLUE = 88,
    MEDIUMORCHID = 89,
    MEDIUMPURPLE = 90,
    MEDIUMSEAGREEN = 91,
    MEDIUMSLATEBLUE = 92,
    MEDIUMSPRINGGREEN = 93,
    MEDIUMTURQUOISE = 94,
    MEDIUMVIOLETRED = 95,
    MIDNIGHTBLUE = 96,
    MINTCREAM = 97,
    MISTYROSE = 98,
    MOCCASIN = 99,
    NAVAJOWHITE = 100,
    NAVY = 101,
    OLDLACE = 102,
    OLIVE = 103,
    OLIVEDRAB = 104,
    ORANGE = 105,
    ORANGERED = 106,
    ORCHID = 107,
    PALEGOLDENROD = 108,
    PALEGREEN = 109,
    PALETURQUOISE = 110,
    PALEVIOLETRED = 111,
    PAPAYAWHIP = 112,
    PEACHPUFF = 113,
    PERU = 114,
    PINK = 115,
    PLUM = 116,
    POWDERBLUE = 117,
    PURPLE = 118,
    RED = 119,
    ROSYBROWN = 120,
    ROYALBLUE = 121,
    SADDLEBROWN = 122,
    SALMON = 123,
    SANDYBROWN = 124,
    SEAGREEN = 125,
    SEASHELL = 126,
    SIENNA = 127,
    SILVER = 128,
    SKYBLUE = 129,
    SLATEBLUE = 130,
    SLATEGRAY = 131,
    SLATEGREY = 132,
    SNOW = 133,
    SPRINGGREEN = 134,
    STEELBLUE = 135,
    TAN = 136,
    TEAL = 137,
    THISTLE = 138,
    TOMATO = 139,
    TURQUOISE = 140,
    VIOLET = 141,
    WHEAT = 142,
    WHITE = 143,
    WHITESMOKE = 144,
    YELLOW = 145,
    YELLOWGREEN = 146
};
} // namespace continuous_clustering

#endif
