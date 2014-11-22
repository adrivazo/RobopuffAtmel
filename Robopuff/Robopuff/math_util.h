//
//  math_util.h
//  Matrix
//
//  Created by Adriana Catalina Vazquez Ortiz on 11/18/14.
//  Copyright (c) 2014 adrivazo. All rights reserved.
//

#ifndef __Matrix__math_util__
#define __Matrix__math_util__

#include <stdio.h>
#include "matrix.h"
#include "point.h"

extern void Util_get_distance_matrix(Point * PointArr, Matrix* distance_matrix);
extern int Util_is_bad_star(Point *star_wannabe);
extern int Util_find_intersection(int * r_c_min,int *r_c_max);
extern int Util_find_other_star(int y_pos_star, int * r_c_max);




#endif /* defined(__Matrix__math_util__) */
