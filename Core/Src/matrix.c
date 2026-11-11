#include "matrix.h"

// C = A * B
void mat_mult(const float *A, int rA, int cA, const float *B, int rB, int cB, float *C) {
    for (int i = 0; i < rA; i++) {
        for (int j = 0; j < cB; j++) {
            float sum = 0.0f;
            for (int k = 0; k < cA; k++) {
                sum += A[i * cA + k] * B[k * cB + j];
            }
            C[i * cB + j] = sum;
        }
    }
}

// C = A + B
void mat_add(const float *A, const float *B, int r, int c, float *C) {
    for (int i = 0; i < r * c; i++) {
        C[i] = A[i] + B[i];
    }
}

// C = A - B
void mat_sub(const float *A, const float *B, int r, int c, float *C) {
    for (int i = 0; i < r * c; i++) {
        C[i] = A[i] - B[i];
    }
}

// At = A^T
void mat_trans(const float *A, int r, int c, float *At) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            At[j * r + i] = A[i * c + j];
        }
    }
}

// 2x2 Matrix Inversion
uint8_t mat_inv_2x2(const float *A, float *A_inv) {
    float det = A[0] * A[3] - A[1] * A[2];

    // Check if matrix is singular (near zero determinant)
    if (det > -1e-6f && det < 1e-6f) {
        return 0; // Inversion failed
    }

    float inv_det = 1.0f / det;

    A_inv[0] =  A[3] * inv_det;
    A_inv[1] = -A[1] * inv_det;
    A_inv[2] = -A[2] * inv_det;
    A_inv[3] =  A[0] * inv_det;

    return 1; // Success
}
