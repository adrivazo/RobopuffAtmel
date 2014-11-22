#include <stdio.h>
#include <stdlib.h>
#include "matrix.h"

int main()
{
    int n=50;
    int m=50;
    Matrix *m1=Matrix_create(n,m);
    Matrix *m2=Matrix_create_identity(n);

    int i,j;
    double val=1;
    for (i=0;i<n;i++){
        for(j=0;j<m;j++){
            val=((double)random()/RAND_MAX)*1;
            if(((double)random()/RAND_MAX)<0.5) val*=-1;
            Matrix_set(m1,i,j,val);
        }
    }


    Matrix *top=Matrix_product(m1,m2);
    Matrix_dump(m1);
    printf("_________Inversing____\n");
	m2=Matrix_inverse(m1);
	printf("OK\n");
 //   Matrix_dump(Matrix_inverse(m1));
    return 0;
}
