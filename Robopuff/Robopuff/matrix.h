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

struct _Matrix{
    int cols;
    int rows;
    double* data;
};

typedef struct _Matrix Matrix;

extern Matrix *Matrix_create(int rows, int cols);
extern double Matrix_get(Matrix *m, int i, int j);
extern void Matrix_find_min(int row_col[], Matrix *m);
extern void Matrix_find_max(int row_col[], Matrix *m);
extern void Matrix_set(Matrix *m, int i, int j, double value);
extern void Matrix_dump(Matrix *m);
extern float Matrix_determinant_2_by_2(float a, float b, float c, float d);
extern void Matrix_inverse_3_by_3_H(Matrix *m, Matrix *inverse);
