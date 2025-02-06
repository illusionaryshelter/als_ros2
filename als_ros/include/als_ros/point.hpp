#pragma once
#include <als_ros/als_ros_utils.hpp>
namespace als_ros {

class Point {
private:
  double x_, y_;

public:
  Point() : x_(0.0), y_(0.0){};

  Point(double x, double y) : x_(x), y_(y){};

  ~Point(){};

  inline void setX(double x) { x_ = x; }
  inline void setY(double y) { y_ = y; }
  inline void setPoint(double x, double y) { x_ = x, y_ = y; }
  inline void setPoint(Point p) { x_ = p.x_, y_ = p.y_; }

  inline double getX(void) { return x_; }
  inline double getY(void) { return y_; }
  inline Point getPoint(void) { return Point(x_, y_); }

}; // class Point

} // namespace als_ros
