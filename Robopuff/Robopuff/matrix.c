/*
 *
Cmatrix, Simple Matrix Library written in C
Copyright (C) 2010,2011,2012  Mehmet Hakan Satman

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Mehmet Hakan Satman - mhsatman@yahoo.com
 * http://www.mhsatman.com
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matrix.h"
#include "m_general.h"

Matrix *Matrix_create(int rows, int cols){
    int i;
    int total=rows*cols;
    Matrix *m=(Matrix*)malloc(sizeof(Matrix));
    m->cols=cols;
    m->rows=rows;
    m->data=(double*)malloc(sizeof(double)*total);
    for (i=0;i<total;i++)m->data[i]=0.0;
    return(m);
}


void Matrix_dump(Matrix *m){
    int i,j,d1=m->rows, d2=m->cols;
    double *p=m->data;
    for (i=0;i<d1;i++){
        for (j=0;j<d2;j++){
            printf("%0.5f ",*p);
            p++;
        }
        printf("\n");
    }
}

double Matrix_get(Matrix *m, int i, int j){
    return(m->data[i*m->cols+j]);
}

void Matrix_find_min(int row_col[], Matrix *m){
    
    int row=0;
    int col=0;
    
    double min_so_far = Matrix_get(m, 0, 1);
    double *p=m->data;
    int i,j,d1=m->rows, d2=m->cols;
    
    for (i=0;i<d1;i++){
        for (j=0;j<d2;j++){
            if((!isnan(*p)) && (*p<min_so_far)){
                min_so_far=*p;
                row=i;
                col=j;
            }
            p++;
        }
    }
    
    row_col[0] = row;
    row_col[1]= col;
}


void Matrix_find_max(int row_col[], Matrix *m){
    int row=0;
    int col=0;
    
    double max_so_far = Matrix_get(m, 0, 1);
    double *p=m->data;
    int i,j,d1=m->rows, d2=m->cols;
    
    for (i=0;i<d1;i++){
        for (j=0;j<d2;j++){
            if((!isnan(*p)) && (*p>max_so_far)){
                max_so_far=*p;
                row=i;
                col=j;
            }
            p++;
        }
    }
    
    row_col[0] = row;
    row_col[1]= col;
}

void Matrix_set(Matrix *m, int i, int j, double value){
    m->data[i*m->cols+j]=value;
}


//calculates the inverse of a 3 by 3 matrix m, where the last line is 0 0 1 and stores it in inverse
void Matrix_inverse_3_by_3_H(Matrix *m, Matrix *inverse){
	float detMat = (Matrix_get(m,0,0)*Matrix_get(m,1,1)*Matrix_get(m,2,2))
				-  (Matrix_get(m,0,1)*Matrix_get(m,1,0)*Matrix_get(m,2,2));
				
	float a00 = m->data[0*m->cols+0];
	float a01 = m->data[0*m->cols+1];
	float a02 = m->data[0*m->cols+2];
	
	float a10 = m->data[1*m->cols+0];
	float a11 = m->data[1*m->cols+1];
	float a12 = m->data[1*m->cols+2];
	
	float a20 = m->data[2*m->cols+0];
	float a21 = m->data[2*m->cols+1];
	float a22 = m->data[2*m->cols+2];
	
	//inverse->data[i*inverse->cols+j]=value;
	
	inverse->data[0*inverse->cols+0] =Matrix_determinant_2_by_2(a11,a12,a21,a22)/detMat;
	inverse->data[0*inverse->cols+1]=Matrix_determinant_2_by_2(a02,a01,a22,a21)/detMat;
	inverse->data[0*inverse->cols+2]=Matrix_determinant_2_by_2(a01,a02,a11,a12)/detMat;
	
	inverse->data[1*inverse->cols+0]=Matrix_determinant_2_by_2(a12,a10,a22,a20)/detMat;
	inverse->data[1*inverse->cols+1]=Matrix_determinant_2_by_2(a00,a02,a20,a22)/detMat;
	inverse->data[1*inverse->cols+2]=Matrix_determinant_2_by_2(a02,a00,a12,a10)/detMat;
	
	inverse->data[2*inverse->cols+0]=Matrix_determinant_2_by_2(a10,a11,a20,a21)/detMat;
	inverse->data[2*inverse->cols+1]=Matrix_determinant_2_by_2(a01,a00,a21,a20)/detMat;
	inverse->data[2*inverse->cols+2]=Matrix_determinant_2_by_2(a00,a01,a10,a11)/detMat;
	
}


float Matrix_determinant_2_by_2(float a, float b, float c, float d){
	return a*d - c*b;
}


