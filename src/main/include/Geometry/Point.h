#pragma once

#include <cmath>
class Point{
    public:
        Point(double x, double y): x_(x), y_(y) {}
        Point() = default;
        Point(const Point& other) : x_(other.x_), y_(other.y_) {}

        double originDist() {
            return sqrt(x_*x_ + y_*y_);
        }

        double dist(Point p) {
            double dx = p.x_ - x_;
            double dy = p.y_ - y_;
            return sqrt(dx*dx + dy*dy);
        }

        double getAng() {
            return atan2(y_, x_);
        }

        void move(double dx, double dy){
            x_ += dx;
            y_ += dy;
        }

        void move(Point p){
            x_ += p.x_;
            y_ += p.y_;
        }

        void rotateThis(double ang){//Radians
            //[cos(a)  -sin(a)]
            //[sin(a)  cos(-a)]
            double nx = cos(ang) * x_ - sin(ang) * y_;
            double ny = sin(ang) * x_ + cos(ang) * y_;
            x_ = nx;
            y_ = ny;
        }

        double getX(){return x_;}
        double getY(){return y_;}

        Point& operator= (const Point& p){
            x_ = p.x_;
            y_ = p.y_;
            return *this;
        }

        Point& operator+= (const Point& p){
            x_ += p.x_;
            y_ += p.y_;
            return *this;
        }

        Point& operator-= (const Point& p){
            x_ -= p.x_;
            y_ -= p.y_;
            return *this;
        }

        Point& operator/= (double k){
            x_ /= k;
            y_ /= k;
            return *this;
        }

        Point& operator- (const Point& p){
            return *(new Point(x_ - p.x_, y_ - p.y_));
        }

        Point& operator* (double k){
            return *(new Point(k * x_, k * y_));
        }

        Point& operator/ (double k){
            return *(new Point(x_ / k, y_ / k));
        }

    private:
        double x_;
        double y_;
};
typedef Point Vector;