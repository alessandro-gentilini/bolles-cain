#define _USE_MATH_DEFINES
#include <cmath>
#include <set>
#include <iostream>
#include <map>



#include <opencv2/core.hpp>

// https://isocpp.org/wiki/faq/mixing-c-and-cpp#include-c-hdrs-nonsystem
extern "C" {
#include "cliquer-1.21/cliquer.h"
}


double deg2rad(double d) {
   return d * M_PI / 180;
}

double distance(const std::vector<cv::Point2d>& P1,
                const std::vector<cv::Point2d>& P2,
                const std::pair<size_t, size_t>& src,
                const std::pair<size_t, size_t>& dst) {
   auto d1 = cv::norm(P1[src.first] - P1[dst.first]);
   auto d2 = cv::norm(P2[src.second] - P2[dst.second]);
   return std::abs(d1 - d2);
}

double angle(const std::vector<cv::Point2d>& P1,
             const std::vector<cv::Point2d>& P2,
             const std::pair<size_t, size_t>& src,
             const std::pair<size_t, size_t>& dst) {
   auto p1 = P1[src.first] - P1[dst.first];
   auto p2 = P2[src.second] - P2[dst.second];
   auto a1 = std::atan2(p1.y, p1.x);
   auto a2 = std::atan2(p2.y, p2.x);
   // https://stackoverflow.com/a/2007279
   return std::atan2(sin(a1 - a2), cos(a1 - a2));
}

template <class T1, class T2>
class Bijection {
 public:
   void insert(const T1& t1, const T2& t2) {
      t12_[t1] = t2;
      t21_[t2] = t1;
   }

   T2 look_up_12(const T1& t1) {
      if (t12_.count(t1) == 0) throw std::logic_error("12 not found");
      return t12_[t1];
   }

   T1 look_up_21(const T2& t2) {
      if (t21_.count(t2) == 0) throw std::logic_error("21 not found");
      return t21_[t2];
   }

   bool present_12(const T1& t1) const {
      return t12_.count(t1) != 0;
   }

   bool present_21(const T2& t2) const {
      return t21_.count(t2) != 0;
   }
 private:
   std::map<T1, T2> t12_;
   std::map<T2, T1> t21_;
};

#include <utility>
int main(int, char*[]) {
   std::vector<cv::Point2d> P1;
   P1.push_back(cv::Point2d(1, 0));
   P1.push_back(cv::Point2d(cos(deg2rad(175)), sin(deg2rad(175))));
   P1.push_back(cv::Point2d(cos(deg2rad(185)), sin(deg2rad(185))));

   auto theta = deg2rad(45);
   cv::Mat R = (cv::Mat_<double>(2, 2) << cos(theta), -sin(theta), sin(theta), cos(theta));

   std::vector<cv::Point2d> P2;
   for (size_t i = 0; i < P1.size(); i++) {
      P2.push_back(cv::Point2d(cos(theta)*P1[i].x - sin(theta)*P1[i].y, sin(theta)*P1[i].x + cos(theta)*P1[i].y));
   }

   std::vector<std::pair<size_t, size_t>> couples;
   for (size_t i = 0; i < P1.size(); i++) {
      for (size_t j = 0; j < P2.size(); j++) {
         couples.push_back(std::make_pair(i, j));
      }
   }

   Bijection<int, std::pair<size_t, size_t>> two_way;

   std::set< std::pair< std::pair<size_t, size_t>, std::pair<size_t, size_t>>> assignments;
   double distance_threshold = 0.1;
   double angle_threshold = M_PI / 100;
   int n_vertices = 0;
   for (size_t i = 0; i < couples.size(); i++) {
      for (size_t j = 0; j < couples.size(); j++) {
         if (i == j) continue;
         if (distance(P1, P2, couples[i], couples[j]) < distance_threshold &&
               angle(P1, P2, couples[i], couples[j]) < angle_threshold) {
            // https://stackoverflow.com/a/38942461
            assignments.insert(std::minmax(couples[i], couples[j]));
            if (!two_way.present_21(couples[i])) {
               two_way.insert(n_vertices++, couples[i]);
            }
            if (!two_way.present_21(couples[j])) {
               two_way.insert(n_vertices++, couples[j]);
            }
         }
      }
   }

   int n_edges = 0;
   std::cout << "graph G{\n";
   for (auto c : assignments) {
      std::cout << "\"(" << c.first.first << "," << c.first.second << ")\" -- \"(" << c.second.first << "," << c.second.second << ")\";\n";
      n_edges++;
   }
   std::cout << "}\n";

   std::cout << "\n";

   graph_t *g = graph_new(n_vertices);

   for (auto c : assignments) {
      GRAPH_ADD_EDGE(g, two_way.look_up_21(c.first), two_way.look_up_21(c.second));
   }

   static clique_options opt_struct = {
      reorder_by_default, NULL, NULL, NULL, NULL, NULL, NULL, 0
   };
   clique_options *opt = &opt_struct;

   set_t s;

   s = clique_find_single(g, 0, 0, FALSE, opt);
   for (size_t v = 0; v < n_vertices; v++) {
      if (SET_CONTAINS(s, v)) {
         auto vv = two_way.look_up_12(v);
         std::cout << "\"(" << vv.first << "," << vv.second << ")\"\n";
      }
   }

   return 0;
}
