//
//  math_util.c
//  Matrix
//
//  Created by Adriana Catalina Vazquez Ortiz on 11/18/14.
//  Copyright (c) 2014 adrivazo. All rights reserved.
//

#include "math_util.h"
#include "math.h"

typedef int bool;
enum { false, true };


/**Calculates the distance matrix from an array of 4 points
 */

Matrix *Util_get_distance_matrix(Point * PointArr){
    Matrix *DM = Matrix_create(4,4);
    double d0 = NAN;
    double d01 = Point_get_distance_between(&PointArr[0], &PointArr[1]);
    double d02 = Point_get_distance_between(&PointArr[0], &PointArr[2]);
    double d03 = Point_get_distance_between(&PointArr[0], &PointArr[3]);
    double d12 = Point_get_distance_between(&PointArr[1], &PointArr[2]);
    double d13 = Point_get_distance_between(&PointArr[1], &PointArr[3]);
    double d23 = Point_get_distance_between(&PointArr[2], &PointArr[3]);

    //diagonals
    Matrix_set(DM, 0, 0, d0);
    Matrix_set(DM, 1, 1, d0);
    Matrix_set(DM, 2, 2, d0);
    Matrix_set(DM, 3, 3, d0);
    
    Matrix_set(DM,0,1, d01);
    Matrix_set(DM,1,0, d01);
    Matrix_set(DM,0,2, d02);
    Matrix_set(DM,2,0, d02);
    
    Matrix_set(DM,0,3, d03);
    Matrix_set(DM,3,0, d03);
    Matrix_set(DM,1,2, d12);
    Matrix_set(DM,2,1, d12);
    
    Matrix_set(DM,1,3, d13);
    Matrix_set(DM,3,1, d13);
    Matrix_set(DM,2,3, d23);
    Matrix_set(DM,3,2, d23);
    
    return DM;
}


bool Util_is_bad_star(Point *star_wannabe){
    if (Point_get_x(star_wannabe)==1023 && Point_get_y(star_wannabe)== 1023)
        {return true;}
    return false;
}

int  Util_find_intersection(int *r_c_min,int * r_c_max){
    int p_i1 = r_c_min[0];
    int p_i2 = r_c_min[1];
    
    if(p_i1== r_c_max[0]||p_i1== r_c_max[1]){
        return p_i1;}
    
    else if(p_i2== r_c_max[0]||p_i2== r_c_max[1]){
        return p_i2;}
    
    //no intersection
    else return NAN;
}



int Util_find_other_star(int y_pos_star, int * r_c_max){
    if(r_c_max[0]==y_pos_star){
        return r_c_max[1];
    }
    
    else return r_c_max[0];
}