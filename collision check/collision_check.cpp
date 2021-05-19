// Author Wenbo Yu
// Date 5/18/21
//
// This is a class for collision checking between planned ego vehicle path and
// predicted path of surrounding vehicles. In detail, the inputs are:
// 1. ego vehicle radius
// 2. vector of 2D ego vehicle path
// 3. list of surrounding vehicles
//      a. vehicle radius
//      b. predicted path
//
// The class check the distance between the shortest segment (waypoints) of ego vehicle
// and surrounding vehicle, and compared it with the safe radius to tell whether 
// collision would happen. The code could be used as part of the behavior planner,


#include <iostream>
#include <vector>
#include <math.h>

template <class T>
struct Point
{
  T x;
  T y;
  Point(T _x, T _y) : x(_x), y(_y) {}
};

template <class T>
class CollisionCheck
{
private:
  struct DynamicObj
  {
    std::vector<Point<T>> future_traj;
    T radius;
  };

  T eucliDist(const Point<T>& p1, const Point<T>& p2) const
  {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y + p2.y, 2));
  }

  bool collisionCheckBetweenTwoPaths(const DynamicObj& obj, const DynamicObj& ego) const
  {
    T safe_dist = ego.radius + obj.radius;
    for (int i = 0; i < ego.future_traj.size() - 1; i++)
    {
      for (int j = 0; j < obj.future_traj.size() - 1; j++)
      {
        T dist = Intersection(ego.future_traj[i], ego.future_traj[i + 1], obj.future_traj[j], obj.future_traj[j + 1]);
        if (dist < safe_dist)
          return true;
      }
    }
    return false;
  }



  T Intersection(const Point<T>& p1, const Point<T>& p2, const Point<T>& p3, const Point<T>& p4) const
  {
    T dx_12 = p2.x - p1.x;
    T dy_12 = p2.y - p1.y;
    T dx_34 = p4.x - p3.x;
    T dy_34 = p4.y - p3.y;

    T denominator = dy_12 * dx_34 - dx_12 * dy_34;
    if (abs(denominator - 0) < 1e-4)
    {
      return eucliDist(p1, p2);
    }
    T t1 = ((p1.x - p3.x) * dy_34 + (p3.y - p1.y) * dx_34) / denominator;
    T t2 = ((p1.x - p3.x) * dy_12 + (p3.y - p1.y) * dx_12) / denominator;

    bool segments_intersect =
      ((t1 >= 0) && (t1 <= 1) &&
        (t2 >= 0) && (t2 <= 1));
    if (segments_intersect)
      return 0;

    t1 = t1 < 0 ? 0 : t1;
    t1 = t1 > 1 ? 1 : t1;
    t2 = t2 < 0 ? 0 : t2;
    t2 = t2 > 1 ? 1 : t2;

    Point<T> close_p1 = {p1.x + dx_12 * t1, p1.y + dy_12 *t1};
    Point<T> close_p2 = {p3.x + dx_34 * t2, p1.y + dy_34 * t2};
    return eucliDist(close_p1, close_p2);
  }

public:

  DynamicObj ego;
  std::vector<DynamicObj> surroundings;

  CollisionCheck()
  {
    ego.radius = 0;
    ego.future_traj = {{1, 2}, {2, 3}};

    DynamicObj obj;
    obj.radius = 0;
    obj.future_traj = {{4, 5}, {5, 6}};
    surroundings.push_back(obj);

    obj.radius = 0;
    obj.future_traj = {{6, 7}, {7, 8}};
    surroundings.push_back(obj);
  }

  bool collisionCheck() const
  {
    for (const auto& obj : surroundings)
    {
      if (collisionCheckBetweenTwoPaths(obj, ego))
      {
        return true;
      }
    }
    return false;
  }
};

int main()
{
  CollisionCheck<double> checker;
  bool collision_found = checker.collisionCheck();
  if (collision_found)
    std::cout << "collision" << std::endl;
  else
    std::cout << "No collision" << std::endl;
  return 0;
}

