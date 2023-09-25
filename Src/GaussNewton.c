#include "GaussNewton.h"
#include "ucos_ii.h"
#include "mpu6050.h"
#include <math.h>
#include <stdio.h>

#define N 6
const int n = N; // 未知数个数和测量数据组数都为 6，即 beta = [Ox, Sx, Oy, Sy, Oz, Sz]T，
                 // 其中 Ox, Oy, Oz 为偏移量，Sx, Sy, Sz 为比例系数

float accData[N][3];
// {
//     {-0.019556, 0.012036, 1.685950},
//     {1.018469, 0.069458, 0.720422},
//     {-0.981909, 0.001819, 0.450183},
//     {0.063464, -0.957397, 0.515881},
//     {-0.021716, 1.004285, 0.713989},
//     {0.032495, -0.022266, -0.387183}
// };
cali accCali;
float magData[N][3];
cali magCali;
static const float eps = 1e-6;
float LM_lamda = 0.1f;

float constrainFloat(float x, float low, float high) {
    if (isnan(x)) {
        return (low + high) * 0.5f;
    }
    return ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)));
}


// 校准前收集数据
void prepareData() {
    // 测试数据1
    // accData[0][0] = 0.0f; accData[0][1] = 0.0f; accData[0][2] = 2.0f;
    // accData[1][0] = 0.0f; accData[1][1] = 0.0f; accData[1][2] = -2.0f;
    // accData[2][0] = 0.0f; accData[2][1] = 1.0f; accData[2][2] = 0.0f;
    // accData[3][0] = 0.0f; accData[3][1] = -1.0f; accData[3][2] = 0.0f;
    // accData[4][0] = 1.0f; accData[4][1] = 0.0f; accData[4][2] = 0.0f;
    // accData[5][0] = -1.0f; accData[5][1] = 0.0f; accData[5][2] = 0.0f;
    printf("Prepare data will begin in 5s...\r\n");
    OSTimeDlyHMSM(0, 0, 5, 0);
    printf("Prepare data begin...\r\n");
    for (int i = 0; i < n; ++i) {
        int m = 10; // 采样次数
        float acc[3] = {0, 0, 0};
        float mag[3] = {0, 0, 0};
        for (int j = 0; j < m; ++j) {
            short accTemp[3], magTemp[3];
            MPU6050ReadAcc(accTemp);
            HMC_ReadMa(magTemp);
            for (int k = 0; k < 3; ++k) {
                acc[k] += accTemp[k] / 16384.0f; // 加速度计量程变了这里记着改！
                mag[k] += magTemp[k] / 1090.0f; // 磁力计量程变了这里记着改！
            }
            OSTimeDlyHMSM(0, 0, 0, 100);
        }
        for (int j = 0; j < 3; ++j) {
            accData[i][j] = acc[j] / m;
            magData[i][j] = mag[j] / m;
        }
        printf("accData[%d] = {%f, %f, %f}\r\n", i, accData[i][0], accData[i][1], accData[i][2]);
        printf("magData[%d] = {%f, %f, %f}\r\n", i, magData[i][0], magData[i][1], magData[i][2]);
        printf("Finished %d/%d\r\n", i + 1, n);
        if (i != n - 1) {
            printf("Next data will be collected in 3s...\r\n");
            OSTimeDlyHMSM(0, 0, 3, 0);
        }
    }
}

void gaussElimination(float (*a)[N + 1]) {
    for (int i = 0; i < n; ++i) {
		int r = i;
		for (int j = i + 1; j < n; ++j)
		    if(fabsf(a[j][i]) > fabsf(a[r][i])) r = j;
        if (fabsf(a[r][i]) < eps) {
			printf("Error: gaussElimination no solution!\r\n");
			return;
		}
		if(r != i) {
            for (int j = 0; j < n + 1; ++j) {
                float temp = a[i][j];
                a[i][j] = a[r][j];
                a[r][j] = temp;
            }
        }
		for (int j = i + 1; j < n; ++j){
			const float div = a[j][i] / a[i][i];
            for (int k = i; k < n + 1; ++k)
			    a[j][k] -= div * a[i][k];
		}
	}
	for(int i = n - 1; i >= 0; --i){
		for (int j = i + 1; j < n; ++j)
		    a[i][n] -= a[j][n] * a[i][j];
		a[i][n] /= a[i][i];
	}
} 

// 高斯牛顿迭代
void gaussNewton(cali *caliVal, float (*data)[3]) {
    float Delta = 100.0f, DeltaNew = 0.0f; 
    static float Jr[N][N], JrT[N][N]; // Jacobi 矩阵以及 Jacobi 矩阵的转置
    static float r[N]; // 残差函数 r(beta)
    static float delta[N][N + 1]; // 待求的迭代增量 delta，也是高斯消元的系数矩阵
    float sum = 0.0f;

    // 初始化    
    static cali beta;
    beta.Ox = beta.Oy = beta.Oz = 0.0f;
    beta.Sx = beta.Sy = beta.Sz = 1.0f;
    LM_lamda = 0.1f;

    int cnt = 0;
    // 迭代
    while (Delta > eps && cnt < 100) {
        // 计算 Jacobi 矩阵
        for (int i = 0; i < n; ++i) {
            Jr[i][0] = 2.0f * beta.Sx * beta.Sx * (beta.Ox - data[i][0]);
            Jr[i][1] = 2.0f * (data[i][0] - beta.Ox) * (data[i][0] - beta.Ox) * beta.Sx;
            Jr[i][2] = 2.0f * beta.Sy * beta.Sy * (beta.Oy - data[i][1]);
            Jr[i][3] = 2.0f * (data[i][1] - beta.Oy) * (data[i][1] - beta.Oy) * beta.Sy;
            Jr[i][4] = 2.0f * beta.Sz * beta.Sz * (beta.Oz - data[i][2]);
            Jr[i][5] = 2.0f * (data[i][2] - beta.Oz) * (data[i][2] - beta.Oz) * beta.Sz;
        }
        // 计算 Jacobi 矩阵的转置
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                JrT[j][i] = Jr[i][j];
            }
        }
        sum = 0.0;
        // 计算残差函数 r(beta)
        for (int i = 0; i < n; ++i) {
            r[i] = beta.Sx * beta.Sx * (beta.Ox - data[i][0]) * (beta.Ox - data[i][0]) + 
                   beta.Sy * beta.Sy * (beta.Oy - data[i][1]) * (beta.Oy - data[i][1]) + 
                   beta.Sz * beta.Sz * (beta.Oz - data[i][2]) * (beta.Oz - data[i][2]) - 1.0f;
            sum += r[i] * r[i];
        }
        // 计算 JrT * Jr 并作为系数矩阵放入 delta
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                delta[i][j] = 0;
                for (int k = 0; k < n; ++k) {
                    delta[i][j] += JrT[i][k] * Jr[k][j];
                }
            }
        }
        // 计算 JrT * r 并作为增广列放入 delta
        for (int i = 0; i < n; ++i) {
            delta[i][n] = 0;
            for (int j = 0; j < n; ++j) {
                delta[i][n] += JrT[i][j] * r[j];
            }
        }
        // 加入 LM 因子
        for (int i = 0; i < n; ++i) {
            delta[i][i] += LM_lamda;
        }
        // 高斯消元
        gaussElimination(delta);
        // 计算 DeltaNew
        DeltaNew = 0;
        for (int i = 0; i < n; ++i) {
            DeltaNew += delta[i][n] * delta[i][n];
        }
        if (DeltaNew < Delta) {
            // LM 因子减小
            LM_lamda /= 3.0;

            // 更新 beta
            beta.Ox -= delta[0][n];
            beta.Sx -= delta[1][n];
            beta.Oy -= delta[2][n];
            beta.Sy -= delta[3][n];
            beta.Oz -= delta[4][n];
            beta.Sz -= delta[5][n];

            Delta = DeltaNew;
        } else {
            // LM 因子增大
            LM_lamda *= 3.0;
            LM_lamda = constrainFloat(LM_lamda, 0, 1e10f);
        }
        
        printf("After %d iterations: Ox = %f, Sx = %f, Oy = %f, Sy = %f, Oz = %f, Sz = %f sum = %f\r\n", 
            ++cnt, beta.Ox, beta.Sx, beta.Oy, beta.Sy, beta.Oz, beta.Sz, sum);
    }
    // 将校准结果写入 caliVal
    caliVal->Ox = beta.Ox;
    caliVal->Sx = beta.Sx;
    caliVal->Oy = beta.Oy;
    caliVal->Sy = beta.Sy;
    caliVal->Oz = beta.Oz;
    caliVal->Sz = beta.Sz;
    printf("Calibration finished!\r\n");
    printf("Ox = %f, Sx = %f, Oy = %f, Sy = %f, Oz = %f, Sz = %f\r\n", 
            caliVal->Ox, caliVal->Sx, caliVal->Oy, caliVal->Sy, caliVal->Oz, caliVal->Sz);
}