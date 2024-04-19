#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    double x;
    double y;
} Point;

double calculate_distance(Point p1, Point p2) {
    double x_diff = p2.x - p1.x;
    double y_diff = p2.y - p1.y;
    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

int main() {
    Point p1 = {1, 2};
    Point p2 = {3, 4};
    double distance = calculate_distance(p1, p2);
    printf("Distance between p1 and p2: %f\n", distance);
    return 0;
}