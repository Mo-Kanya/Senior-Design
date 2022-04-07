#include "light_matrix.h"
#include <stdio.h>
#include <stdlib.h>

#define MAT_LEGAL_CHECKING
#define MAT_INIT_FLAG   0x5C

/************************************************************************/
/*                          Private Function                            */
/************************************************************************/

void swap(int *a, int *b)
{
  int m;
  m = *a;
  *a = *b;
  *b = m;
}

void perm(int list[], int k, int m, int* p, Mat* mat, float* det) 
{
  int i;

  if(k > m)
  {
    float res = mat->element[0][list[0]];

    for(i = 1; i < mat->row ; i++){
        res *= mat->element[i][list[i]];
    }
    if(*p%2){
        //odd is negative
        *det -= res;
    }else{
        //even is positive
        *det += res;
    }
  }
  else
  {
    perm(list, k + 1, m, p, mat, det);
    for(i = k+1; i <= m; i++)
    {
      swap(&list[k], &list[i]);
      *p += 1;
      perm(list, k + 1, m, p, mat, det);
      swap(&list[k], &list[i]);
      *p -= 1; 
    }
  }
}

/************************************************************************/
/*                           Public Function                            */
/************************************************************************/

Mat* MatCreate(Mat* mat, int row, int col)
{
    int i;

#ifdef MAT_LEGAL_CHECKING
    if(mat->init == MAT_INIT_FLAG){
        if(mat->row == row && mat->col == col)
            return mat;
        else
            MatDelete(mat);
    }
#endif

    mat->element = (float**)malloc(row * sizeof(float*));
    for(i = 0 ; i < row ; i++){
        mat->element[i] = (float*)malloc(col * sizeof(float));  
    }

    if(mat->element == NULL){
        return NULL;
    }
    mat->row = row;
    mat->col = col;
    mat->init = MAT_INIT_FLAG;

    return mat;
}

void MatDelete(Mat* mat)
{
    int i;

#ifdef MAT_LEGAL_CHECKING
    if(mat->init != MAT_INIT_FLAG){
        return ;
    }
#endif

    for(i = 0 ; i<mat->row ; i++)
        free(mat->element[i]);
    free(mat->element);

    mat->init = 0;
}

Mat* MatSetVal(Mat* mat, float* val)
{
    int row,col;

    for(row = 0 ; row < mat->row ; row++){
        for(col = 0 ; col < mat->col ; col++){
            mat->element[row][col] = val[col + row * mat->col];
        }
    }

    return mat;
}

void MatDump(const Mat* mat)
{
    int row,col;

#ifdef MAT_LEGAL_CHECKING
    if(mat == NULL){
        return ;
    }
#endif

    printf("Mat %dx%d:\n", mat->row, mat->col);
    for(row = 0 ; row < mat->row ; row++){
        for(col = 0 ; col < mat->col ; col++){
            printf("%.4f\t", mat->element[row][col]);
        }
        printf("\n");
    }
}

Mat* MatZeros(Mat* mat)
{
    int row,col;

#ifdef MAT_LEGAL_CHECKING
    if(mat->init != MAT_INIT_FLAG){
        printf("err check, none init matrix for MatZeros\n");
        return NULL;
    }
#endif

    for(row = 0 ; row < mat->row ; row++){
        for(col = 0 ; col < mat->col ; col++){
            mat->element[row][col] = 0.0f;
        }
    }

    return mat;
}

Mat* MatEye(Mat* mat)
{
    int i;

#ifdef MAT_LEGAL_CHECKING
    if(mat->init != MAT_INIT_FLAG){
        printf("err check, none init matrix for MatEye\n");
        return NULL;
    }
#endif

    for(i = 0 ; i < min(mat->row, mat->col) ; i++){
        mat->element[i][i] = 1.0f;
    }

    return mat;
}

/* dst = src1 + src2 */
Mat* MatAdd(Mat* src1, Mat* src2, Mat* dst)
{
    int row, col;

#ifdef MAT_LEGAL_CHECKING
    if( !(src1->row == src2->row && src2->row == dst->row && src1->col == src2->col && src2->col == dst->col) ){
        printf("err check, unmatch matrix for MatAdd\n");
        MatDump(src1);
        MatDump(src2);
        MatDump(dst);
        return NULL;
    }
#endif

    for(row = 0 ; row < src1->row ; row++){
        for(col = 0 ; col < src1->col ; col++){
            dst->element[row][col] = src1->element[row][col] + src2->element[row][col];
        }
    }

    return dst;
}

/* dst = src1 - src2 */
Mat* MatSub(Mat* src1, Mat* src2, Mat* dst)
{
    int row, col;

#ifdef MAT_LEGAL_CHECKING
    if( !(src1->row == src2->row && src2->row == dst->row && src1->col == src2->col && src2->col == dst->col) ){
        printf("err check, unmatch matrix for MatSub\n");
        MatDump(src1);
        MatDump(src2);
        MatDump(dst);
        return NULL;
    }
#endif

    for(row = 0 ; row < src1->row ; row++){
        for(col = 0 ; col < src1->col ; col++){
            dst->element[row][col] = src1->element[row][col] - src2->element[row][col];
        }
    }

    return dst;
}

/* dst = src1 * src2 */
Mat* MatMul(Mat* src1, Mat* src2, Mat* dst)
{
    int row, col;
    int i;
    float temp;

#ifdef MAT_LEGAL_CHECKING
    if( src1->col != src2->row || src1->row != dst->row || src2->col != dst->col ){
        printf("err check, unmatch matrix for MatMul\n");
        MatDump(src1);
        MatDump(src2);
        MatDump(dst);
        return NULL;
    }
#endif

    for(row = 0 ; row < dst->row ; row++){
        for(col = 0 ; col < dst->col ; col++){
            temp = 0.0f;
            for(i = 0 ; i < src1->col ; i++){
                temp += src1->element[row][i] * src2->element[i][col];
            }
            dst->element[row][col] = temp;
        }
    }

    return dst;
}

/* dst = src' */
Mat* MatTrans(Mat* src, Mat* dst)
{
    int row, col;

#ifdef MAT_LEGAL_CHECKING
    if( src->row != dst->col || src->col != dst->row ){
        printf("err check, unmatch matrix for MatTranspose\n");
        MatDump(src);
        MatDump(dst);
        return NULL;
    }
#endif

    for(row = 0 ; row < dst->row ; row++){
        for(col = 0 ; col < dst->col ; col++){
            dst->element[row][col] = src->element[col][row];
        }
    }

    return dst;
}

// return det(mat)
float MatDet(Mat* mat)
{
    float det = 0.0f;
    int plarity = 0;
    int *list;
    int i;

#ifdef MAT_LEGAL_CHECKING
    if( mat->row != mat->col){
        printf("err check, not a square matrix for MatDetermine\n");
        MatDump(mat);
        return 0.0f;
    }
#endif

    list = (int*)malloc(sizeof(int)*mat->col);
    for(i = 0 ; i < mat->col ; i++)
        list[i] = i;

    perm(list, 0, mat->row-1, &plarity, mat, &det);
    free(list);

    return det;
}

// dst = adj(src)
Mat* MatAdj(Mat* src, Mat* dst)
{
    Mat smat;
    int row, col;
    int i,j,r,c;
    float det;

#ifdef MAT_LEGAL_CHECKING
    if( src->row != src->col || src->row != dst->row || src->col != dst->col){
        printf("err check, not a square matrix for MatAdj\n");
        MatDump(src);
        MatDump(dst);
        return NULL;
    }
#endif

    MatCreate(&smat, src->row-1, src->col-1);

    for(row = 0 ; row < src->row ; row++){
        for(col = 0 ; col < src->col ; col++){
            r = 0;
            for(i = 0 ; i < src->row ; i++){
                if(i == row)
                    continue;
                c = 0;
                for(j = 0; j < src->col ; j++){
                    if(j == col)
                        continue;
                    smat.element[r][c] = src->element[i][j];
                    c++;
                }
                r++;
            }
            det = MatDet(&smat);
            if((row+col)%2)
                det = -det;
            dst->element[col][row] = det;
        }
    }

    MatDelete(&smat);

    return dst;
}

// dst = src^(-1)
Mat* MatInv(Mat* src, Mat* dst)
{
    Mat adj_mat;
    float det;
    int row, col;

#ifdef MAT_LEGAL_CHECKING
    if( src->row != src->col || src->row != dst->row || src->col != dst->col){
        printf("err check, not a square matrix for MatInv\n");
        MatDump(src);
        MatDump(dst);
        return NULL;
    }
#endif

    MatCreate(&adj_mat, src->row, src->col);
    MatAdj(src, &adj_mat);
    det = MatDet(src);

    for(row = 0 ; row < src->row ; row++){
        for(col = 0 ; col < src->col ; col++)
            dst->element[row][col] = adj_mat.element[row][col]/det;
    }

    MatDelete(&adj_mat);

    return dst;
}