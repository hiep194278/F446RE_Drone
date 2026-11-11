#ifndef MATRIX_H_
#define MATRIX_H_

#include <stdint.h>

void mat_mult(const float *A, int rA, int cA, const float *B, int rB, int cB, float *C);
void mat_add(const float *A, const float *B, int r, int c, float *C);
void mat_sub(const float *A, const float *B, int r, int c, float *C);
void mat_trans(const float *A, int r, int c, float *At);
uint8_t mat_inv_2x2(const float *A, float *A_inv);

#endif /* MATRIX_H_ */
