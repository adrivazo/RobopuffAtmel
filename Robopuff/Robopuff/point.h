//
//  point.h
//  Matrix
//
//  Created by Adriana Catalina Vazquez Ortiz on 11/18/14.
//  Copyright (c) 2014 adrivazo. All rights reserved.
//

#ifndef __Matrix__point__
#define __Matrix__point__

#include <stdio.h>

struct _Point{
    double x;
    double y;
};

typedef struct _Point Point;

extern Point *Point_create(double x, double y);
extern Point *Point_add(Point *m1, Point *m2);
extern Point *Point_sub(Point *m1, Point *m2);
extern Point *Point_product_scaler(Point *m, double s);
extern Point *Point_get_midpoint(Point *m1, Point *m2);
extern double Point_get_distance_between(Point* m1, Point* m2);
extern double Point_get_x(Point *m);
extern double Point_get_y(Point *m);
extern void Point_set_x(Point *m, double value);
extern void Point_set_y(Point *m, double value);

extern void Point_dump(Point *m);

#endif /* defined(__Matrix__point__) */