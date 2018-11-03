#define _USE_MATH_DEFINES
#include <cmath>
#include <set>
#include <iostream>
#include <map>
#include <random>



#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>


// https://isocpp.org/wiki/faq/mixing-c-and-cpp#include-c-hdrs-nonsystem
extern "C" {
#include "cliquer-1.21/cliquer.h"
}


double deg2rad(double d) {
   return d * M_PI / 180;
}

double rad2deg(double r) {
   return r * 180 / M_PI;
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
   auto result = std::abs(std::atan2(sin(a1 - a2), cos(a1 - a2)));
   //std::cerr << p1 << " " << p2 << " " << rad2deg(a1) << " " << rad2deg(a2) << " " << rad2deg(result) << "\n";
   return result;
}

std::vector<double> quad_code(
   const cv::Point2d& A,
   const cv::Point2d& B,
   const cv::Point2d& C,
   const cv::Point2d& D) {
   std::vector<double> result;
   auto x_A = A.x;
   auto x_B = B.x;
   auto y_A = A.y;
   auto y_B = B.y;   
   auto c=(1.0/2.0)*M_SQRT2*(-x_A + x_B - y_A + y_B)*sqrt(1.0/(pow(x_A, 2) - 2*x_A*x_B + pow(x_B, 2) + pow(y_A, 2) - 2*y_A*y_B + pow(y_B, 2)));
   auto s=(1.0/2.0)*M_SQRT2*(-x_A + x_B + y_A - y_B)*sqrt(1.0/(pow(x_A, 2) - 2*x_A*x_B + pow(x_B, 2) + pow(y_A, 2) - 2*y_A*y_B + pow(y_B, 2)));
   auto t_x=(pow(x_A, 2) - x_A*x_B - x_A*y_B + x_B*y_A + pow(y_A, 2) - y_A*y_B)/(pow(x_A, 2) - 2*x_A*x_B + pow(x_B, 2) + pow(y_A, 2) - 2*y_A*y_B + pow(y_B, 2));
   auto t_y=(pow(x_A, 2) - x_A*x_B + x_A*y_B - x_B*y_A + pow(y_A, 2) - y_A*y_B)/(pow(x_A, 2) - 2*x_A*x_B + pow(x_B, 2) + pow(y_A, 2) - 2*y_A*y_B + pow(y_B, 2));
   auto lambda=M_SQRT2*sqrt(1.0/(pow(x_A, 2) - 2*x_A*x_B + pow(x_B, 2) + pow(y_A, 2) - 2*y_A*y_B + pow(y_B, 2)));
   result.push_back(c);
   result.push_back(s);
   result.push_back(t_x);
   result.push_back(t_y);
   result.push_back(lambda);
   return result;
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
    // P1.push_back(cv::Point2d(cos(deg2rad(15)), sin(deg2rad(15))));
    // P1.push_back(cv::Point2d(cos(deg2rad(35)), sin(deg2rad(35))));
    // P1.push_back(cv::Point2d(cos(deg2rad(45)), sin(deg2rad(45))));

   cv::Mat image = cv::Mat::zeros(400, 400, CV_8UC3);
   cv::Point2d center(200,200);
   cv::circle(image,center+100*P1[0],3,cv::Scalar(0,0,255));
   for(size_t i = 1; i < P1.size(); i++){
      cv::line(image, center+100*P1[i], center+100*P1[i-1], cv::Scalar(0, 0, 255));//,  2, 8 );
      cv::circle(image,center+100*P1[i],3,cv::Scalar(0,0,255));
   }
   
 

   auto theta = deg2rad(45);


   std::vector<cv::Point2d> P2;
   for (size_t i = 0; i < P1.size(); i++) {
      if(i==5 || i==3) continue;
      P2.push_back(cv::Point2d(cos(theta)*P1[i].x - sin(theta)*P1[i].y, sin(theta)*P1[i].x + cos(theta)*P1[i].y));
   }

   std::random_device rd;
   std::mt19937 rng(rd());
   //std::shuffle(P2.begin(), P2.end(), rng);

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
         if (distance(P1, P2, couples[i], couples[j]) < distance_threshold 
            && angle(P1, P2, couples[i], couples[j]) < angle_threshold
            ) {
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

   std::vector<double> quad = quad_code(cv::Point2d(0,0),cv::Point2d(1,1),cv::Point2d(0,0),cv::Point2d(1,1));
   for(size_t i = 0; i < quad.size(); i++){
      std::cout << quad[i] << " ";
   }
   std::cout << "\n";

   if(n_vertices<=0) {
      std::cerr << "Empty graph\n";
      return 1;
   }

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

   std::cout << "\ngraph G{\n";

   std::vector<cv::Point2d> src, dst;

   // give a color to the nodes in the clique
   for (size_t v = 0; v < n_vertices; v++) {
      if (SET_CONTAINS(s, v)) {
         auto vv = two_way.look_up_12(v);
         std::cout << "\"(" << vv.first << "," << vv.second << ")\" [color=red];\n";
         src.push_back(P1[vv.first]);
         dst.push_back(P2[vv.second]);
         cv::circle(image,center+100*P1[vv.first],5,cv::Scalar(0,255,0));
         //cv::circle(image,center+100*P2[vv.second],6,cv::Scalar(0,0,220));
      }
   }
   

   cv::Mat RT = cv::estimateRigidTransform(src, dst, false);
   cv::Mat original = (cv::Mat_<double>(2, 3) << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0);

   std::cerr << "Applied transformation:\n" << original << "\n\n";
   std::cerr << "Estimated transformation:\n" << RT << "\n\n";

   if(!RT.empty()) {
      double maxVal=-1;
      cv::minMaxIdx(cv::abs(RT - original),0,&maxVal);
      std::cerr << "Max error: " << maxVal << "\n\n";
      auto estimated_theta = std::acos(RT.at<double>(0, 0));
      std::cerr << "Rotation angle: " << rad2deg(estimated_theta) << "deg\n";


      for (size_t v = 0; v < n_vertices; v++) {
         if (SET_CONTAINS(s, v)) {
            auto vv = two_way.look_up_12(v);
            auto p = P2[vv.second];
            auto transformed = cv::Point2d(cos(-estimated_theta)*p.x - sin(-estimated_theta)*p.y, sin(-estimated_theta)*p.x + cos(-estimated_theta)*p.y);
            cv::circle(image,center+100*transformed,7,cv::Scalar(255,0,0));
         }
      }
      cv::imwrite("result.png",image);
   }


   // print the clique edges
   for (auto c : assignments) {
      std::cout << "\"(" << c.first.first << "," << c.first.second << ")\" -- \"(" << c.second.first << "," << c.second.second << ")\";\n";
   }
   std::cout << "}\n";

   return 0;
}
