#include "mpu6050.h"

#include <stdio.h>

#include "i2c.h"

#define MPU_ERROR I2C_ERROR
#define MPU_INFO I2C_INFO
/**
 * @brief   写数据到MPU6050寄存器
 * @param   reg_add:寄存器地址
 * @param	 reg_data:要写入的数据
 * @retval
 */
void MPU6050_WriteReg(u8 reg_add, u8 reg_dat) {
    Sensors_I2C_WriteRegister(MPU6050_ADDRESS, reg_add, 1, &reg_dat);
}

/**
 * @brief   从MPU6050寄存器读取数据
 * @param   reg_add:寄存器地址
 * @param	 Read：存储数据的缓冲区
 * @param	 num：要读取的数据量
 * @retval
 */
void MPU6050_ReadData(u8 reg_add, unsigned char *Read, u8 num) {
    Sensors_I2C_ReadRegister(MPU6050_ADDRESS, reg_add, num, Read);
}

//写数据到HMC5883L寄存器
void HMC_WriteReg(u8 reg_add, u8 reg_dat) {
    Sensors_I2C_WriteRegister(HMC_ADDR, reg_add, 1, &reg_dat);
}

//从HMC5883L寄存器读取数据
void HMC_ReadData(u8 reg_add, unsigned char *Read, u8 num) {
    Sensors_I2C_ReadRegister(HMC_ADDR, reg_add, num, Read);
}

/**
 * @brief   初始化MPU6050芯片
 * @param
 * @retval
 */
void MPU6050_Init(void) {
    int i = 0, j = 0;
    //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
    for (i = 0; i < 1000; i++) {
        for (j = 0; j < 1000; j++) {
            ;
        }
    }
    // MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x80);
    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);      //解除休眠状态
    MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV, 0x00);      //陀螺仪采样率为1KHz
    MPU6050_WriteReg(MPU6050_RA_CONFIG, 0x06);          //低通滤波频率
    // MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);
    // //配置加速度传感器工作在16G模式
    MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG, 0x00);   //配置加速度传感器工作在 2G 模式
    // MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18); //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
    MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x00);    //陀螺仪自检及测量范围，典型值：0x00(不自检，250deg/s)
    MPU6050_WriteReg(MPU6050_RA_INT_PIN_CFG, 0x02);    //打开旁路模式
    MPU6050_WriteReg(MPU6050_RA_USER_CTRL, 0x00);      //关闭I2C主模式
}

/**
 * @brief   读取MPU6050的ID
 * @param
 * @retval  正常返回1，异常返回0
 */
uint8_t MPU6050ReadID(void) {
    unsigned char Re = 0;
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &Re, 1);  //读器件地址
    if (Re != 0x68) {
        MPU_ERROR("MPU6050 dectected error!\r\n");
        return 0;
    } else {
        MPU_INFO("MPU6050 ID = %d\r\n", Re);
        return 1;
    }
}

void HMC_Init(void) {
    //先配置MPU6050的寄存器，将其挂载到总线上(这是怎么知道的)
    // MPU6050_WriteReg(MPU6050_RA_INT_PIN_CFG, 0x02);
    // MPU6050_WriteReg(MPU6050_RA_USER_CTRL, 0x00);
    HMC_WriteReg(HMC_CONFIGA, 0x78);  //平均采样数为8, 速率为75Hz
    HMC_WriteReg(HMC_CONFIGB, 0x20);  //增益1090
    HMC_WriteReg(HMC_MODE, 0x00);     //连续测量模式
}

/**
 * @brief   读取MPU6050的加速度数据
 * @param
 * @retval
 */
void MPU6050ReadAcc(short *accData) {
    u8 buf[6];
    MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
 * @brief   读取MPU6050的角加速度数据
 * @param
 * @retval
 */
void MPU6050ReadGyro(short *gyroData) {
    u8 buf[6];
    MPU6050_ReadData(MPU6050_GYRO_OUT, buf, 6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

/**
 * @brief   读取MPU6050的原始温度数据
 * @param
 * @retval
 */
void MPU6050ReadTemp(short *tempData) {
    u8 buf[2];
    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H, buf, 2);  //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}

/**
 * @brief   读取MPU6050的温度数据，转化成摄氏度
 * @param
 * @retval
 */
void MPU6050_ReturnTemp(float *Temperature) {
    short temp3;
    u8 buf[2];

    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H, buf, 2);  //读取温度值
    temp3 = (buf[0] << 8) | buf[1];
    *Temperature = ((double)temp3 / 340.0) + 36.53;
}

void HMC_ReadMa(short *MaData) {
    u8 buf[6];
    HMC_ReadData(HMC_DATA_XMSB, buf, 6);
    MaData[0] = (buf[0] << 8) + buf[1];
    MaData[2] = (buf[2] << 8) + buf[3];
    MaData[1] = (buf[4] << 8) + buf[5];
}

const float alpha = 0.02f;
void gyroLowPassFilter(float *gyro, float *gyroFiltered) {
    static float lastOutput[3] = {0.0f, 0.0f, 0.0f};
    gyroFiltered[0] = alpha * gyro[0] + (1.0f - alpha) * lastOutput[0];
    gyroFiltered[1] = alpha * gyro[1] + (1.0f - alpha) * lastOutput[1];
    gyroFiltered[2] = alpha * gyro[2] + (1.0f - alpha) * lastOutput[2];
    lastOutput[0] = gyroFiltered[0];
    lastOutput[1] = gyroFiltered[1];
    lastOutput[2] = gyroFiltered[2];
}