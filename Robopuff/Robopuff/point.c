/*
 * Simple Point library
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "point.h"

Point *Point_create(double x, double y){
    Point *p = (Point*)malloc(sizeof(Point));
    p->x=x;
    p->y=y;
    return p;
}


void Point_dump(Point *m){
    printf("%0.5f", Point_get_x(m));
    printf(",");
    printf("%0.5f", Point_get_y(m));
    printf("\n");
}

Point *Point_add(Point *m1, Point *m2){
    Point* sum=Point_create(Point_get_x(m1)+Point_get_x(m2),Point_get_y(m1)+Point_get_y(m2));
    return(sum);
}

Point *Point_sub(Point *m1, Point *m2){
    Point* sum=Point_create(Point_get_x(m1)-Point_get_x(m2),Point_get_y(m1)-Point_get_y(m2));
    return(sum);
}


Point *Point_product_scaler(Point *m, double s){
    double x_s = s*Point_get_x(m);
    double y_s = s*Point_get_y(m);
    Point* newm=Point_create(x_s,y_s);
    return(newm);
}

Point *Point_get_midpoint(Point *m1, Point *m2){
    return Point_product_scaler(Point_add(m1, m2),0.5);
}

double Point_get_distance_between(Point *m1, Point *m2){
//	Point *diff = Point_sub(m1, m2);
	return sqrt(pow(Point_get_x(m1)-Point_get_x(m2),2)+pow(Point_get_y(m1)-Point_get_y(m2), 2));
	//double x_sq = (Point_get_x(m1)-Point_get_x(m2))*(Point_get_x(m1)-Point_get_x(m2));
	//double y_sq = (Point_get_y(m1)-Point_get_y(m2))*(Point_get_y(m1)-Point_get_y(m2));
	//return sqrt(x_sq+y_sq);
}

double Point_get_x(Point *m){
    return m->x;
}

double Point_get_y(Point *m){
    return m->y;
}

void Point_set_x(Point *m, double value){
    m->x=value;
}
void Point_set_y(Point *m, double value){
    m->y=value;
}