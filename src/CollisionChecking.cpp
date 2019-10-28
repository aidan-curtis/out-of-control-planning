///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Aidan Curtis
//////////////////////////////////////

#include "CollisionChecking.h"
#include <cmath>


// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    for (int i = 0; i < (int)obstacles.size(); i++){
        // Simply check to see if the point is inside the rectangle
        if(x >= obstacles[i].x && x <= obstacles[i].x + obstacles[i].width && y >= obstacles[i].y && y <= obstacles[i].y + obstacles[i].height){
            return false;
        }
    }
    // The point is not inside the rectangle
    return true;
}


float norm(double x1, double y1, double x2, double y2){
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    for (int i = 0; i < (int)obstacles.size(); i++) {
        if (obstacles[i].x - radius <= x && x <= obstacles[i].x + obstacles[i].width + radius && obstacles[i].y <= y && obstacles[i].y + obstacles[i].height >= y){
            return false; // Edge checks for circle intersection
        }
        if (obstacles[i].y - radius <= y && y <= obstacles[i].y + obstacles[i].height + radius && obstacles[i].x <= x && obstacles[i].x + obstacles[i].width >= x){
            return false; // More edge checks for circle intersection
        }
        if (norm(x, y, obstacles[i].x, obstacles[i].y) <= radius ){
            return false; // Vertex intersection check
        }
        if (norm(x, y, obstacles[i].x+obstacles[i].width, obstacles[i].y) <= radius ){
            return false;  // Vertex intersection check 
        }
        if (norm(x, y, obstacles[i].x, obstacles[i].y+obstacles[i].height) <= radius ){
            return false; // Vertex intersection check
        }
        if (norm(x, y, obstacles[i].x+obstacles[i].width, obstacles[i].y+obstacles[i].height) <= radius ){
            return false; // Vertex intersection check
        }
    }
    return true;
}



bool segments_intersect(double a1x, double a1y, double b1x, double b1y, double a2x, double a2y, double b2x, double b2y){
    /* 
     * This function checks to see if two line segments intersect.
     * The first point of the first line segment is represented by a1x, a1y
     * The second point of the first line segment is represented by b1x, b1y
     * The first point of the second line segment is represented by a2x, a2y
     * The second point of the second line segment is represented by b2x, b2y
     */

    if(a1y > b1y){
        double tempx = a1x;
        double tempy = a1y;
        a1y = b1y;
        a1x = b1x;
        b1y = tempy;
        b1x = tempx;
    }

    if(a2y > b2y){
        double tempx = a2x;
        double tempy = a2y;
        a2y = b2y;
        a2x = b2x;
        b2y = tempy;
        b2x = tempx;
    }
    // If the lines are both vertical
    if(a1x == b1x && a2x == b2x){
        if(a1x == a2x){
            // Both lines are vertical
            if(a2y <= b1y && b1y < a2y && a2y <= b2y){
                return false;
            }
            if(a2y <= b2y && b2y < a1y && a1y <= b1y){
                return false;
            }
            return true;
        } else {
            return false;
        }
    } else if(a1x == b1x){
        // One line is vertical
        double slope2 = (a2y-b2y)/(a2x-b2x);
        double yint2 = -slope2*a2x+a2y;
        double yintersect = slope2*a1x+yint2;
        // Determine if they are intersecting or not
        if(a1y <= yintersect && yintersect <= b1y && a2y <= yintersect && yintersect <= b2y){
            if((a2x <= a1x && a1x<= b2x) || (b2x <= a1x && a1x<= a2x)){
                return true;
            } else {
                return false;
            }
        }
        return false;

    } else if (a2x == b2x){
        // Other line is vertical
        double slope1 = (a1y-b1y)/(a1x-b1x);
        double yint1 = -slope1*a1x+a1y;
        double yintersect = slope1*a2x+yint1;

        //Determine if they are intersecting or not
        if(a2y <= yintersect && yintersect <= b2y && a1y <= yintersect && yintersect <= b1y){
            if((a1x <= a2x && a2x<= b1x) || (b1x <= a2x && a2x<= a1x)){
                return true;
            } else {
                return false;
            }
        }
        return false;
    }
    
    double slope1 = (a1y-b1y)/(a1x-b1x);
    double slope2 = (a2y-b2y)/(a2x-b2x);
    
    double yint1 = -slope1*a1x+a1y;
    double yint2 = -slope2*a2x+a2y;

    // Order them by x value
    if(a1x > b1x){
        double tempx = a1x;
        double tempy = a1y;
        a1y = b1y;
        a1x = b1x;
        b1y = tempy;
        b1x = tempx;
    }
    if(a2x > b2x){
        double tempx = a2x;
        double tempy = a2y;
        a2y = b2y;
        a2x = b2x;
        b2y = tempy;
        b2x = tempx;
    }
    // If they are parallel check if they intersect
    if(slope1 == slope2){
        if(yint1 == yint2){
            if(a1x <= b1x && b1x < a2x && a2x <= b2x){
                return false;
            } else if(a2x <= b2x && b2x < a1x && a1x <= b1x){
                return false;
            }
            return true;
        }
        return false;
    }
    // Finally, just check if they intersect using the formula
    double intersect = (yint2-yint1)/(slope1-slope2);
    if(a1x <= intersect && intersect <= b1x && a2x <= intersect && intersect <= b2x){
        return true;
    }
    return false;
}

// Checks if the point is inside a rectangle
bool point_inside(double x, double y, Rectangle r){
    return x>=r.x && x<=r.x+r.width && y>=r.y && y<=r.y+r.height;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{


    // Rotation Matrix
    // c  -s   x       x
    // s   c   y   *   y
    // 0   0   1       1

    std::vector<double> rx1, ry1, rx2, ry2; 

    double s = sin(theta);
    double c = cos(theta);

    // Multiply points by transformation matrix
    rx1.push_back(c * (-sideLength/2.0) - s * (-sideLength/2.0) + x);
    ry1.push_back(s * (-sideLength/2.0) + c * (-sideLength/2.0) + y); 
    rx2.push_back(c * (sideLength/2.0) - s * (-sideLength/2.0) + x);
    ry2.push_back(s * (sideLength/2.0) + c * (-sideLength/2.0) + y);

    rx1.push_back(c * (sideLength/2.0) - s * (-sideLength/2.0) + x);
    ry1.push_back(s * (sideLength/2.0) + c * (-sideLength/2.0) + y);
    rx2.push_back(c * (sideLength/2.0) - s * (sideLength/2.0) + x);
    ry2.push_back(s * (sideLength/2.0) + c * (sideLength/2.0) + y);

    rx1.push_back(c * (-sideLength/2.0) - s * (-sideLength/2.0) + x);
    ry1.push_back(s * (-sideLength/2.0) + c * (-sideLength/2.0) + y);
    rx2.push_back(c * (-sideLength/2.0) - s * (sideLength/2.0) + x);
    ry2.push_back(s * (-sideLength/2.0) + c * (sideLength/2.0) + y);

    rx1.push_back(c * (-sideLength/2.0) - s * (sideLength/2.0) + x);
    ry1.push_back(s * (-sideLength/2.0) + c * (sideLength/2.0) + y);  
    rx2.push_back(c * (sideLength/2.0) - s * (sideLength/2.0) + x);
    ry2.push_back(s * (sideLength/2.0) + c * (sideLength/2.0) + y);


    for (int i = 0; i < (int)obstacles.size(); i++) {
        for (int j = 0; j < (int)rx1.size(); j++){
            // Get the 4 line segments of the obstacle and check intersections with the transformed object
            if(segments_intersect(rx1[j], ry1[j], rx2[j], ry2[j], obstacles[i].x, obstacles[i].y, obstacles[i].x+obstacles[i].width, obstacles[i].y)){
                return false;
            } else if(segments_intersect(rx1[j], ry1[j], rx2[j], ry2[j], obstacles[i].x, obstacles[i].y, obstacles[i].x, obstacles[i].y+obstacles[i].height)){
                return false;
            } else if(segments_intersect(rx1[j], ry1[j], rx2[j], ry2[j], obstacles[i].x+obstacles[i].width, obstacles[i].y+obstacles[i].height, obstacles[i].x+obstacles[i].width, obstacles[i].y)){
                return false;
            } else if(segments_intersect(rx1[j], ry1[j], rx2[j], ry2[j], obstacles[i].x+obstacles[i].width, obstacles[i].y+obstacles[i].height, obstacles[i].x, obstacles[i].y+obstacles[i].height)){
                return false;
            } else if(point_inside(rx1[j], ry1[j], obstacles[i]) || point_inside(rx2[j], ry2[j], obstacles[i])){
                return false;
            }

        }
    }
    return true;
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot> & /*robots*/, const std::vector<Rectangle> & /*obstacles*/,
               const std::vector<bool> & /*valid*/)
{
}
