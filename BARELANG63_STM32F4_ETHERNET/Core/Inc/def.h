/*
 * def.h
 *
 *  Created on: Jun 15, 2024
 *      Author: abiq
 */

#ifndef INC_DEF_H_
#define INC_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif

//struct motor{
//	int kiri, kanan, belakang;
//	int leftDribble, rightDribble;
//};
//

//struct dataSensor{
//	int Imu;
//	uint8_t leftPot, rightPot;
//	uint8_t Battery;
//	uint8_t lifterReady;
//};
//
//struct Extend{
//	int Kiri, Kanan, Atas;
//};
//
//struct STMtoPC{
//	float batrePC, batreMotor;
//	struct motor Encoder;
//	struct dataSensor Sensor;
//	struct Extend encExtend;
//	struct Extend limitExtend;
//	uint8_t isKicked;
//	uint8_t Reset;
//};
//
//
//struct PCtoSTM{
//	struct motor Speed;
//	struct Extend extendSpeed;
//	int lifterTarget;
//	uint8_t kickerPower;
//};
//
//struct STMtoPC Send;
//struct PCtoSTM Command;

#define SetTick(_Tick) (_Tick = HAL_GetTick())
#define ElapsedTime(_Tick, _Delay) (HAL_GetTick() - _Tick >= _Delay)
#define LoopDelay 33
#define DisconnectTime 300



struct razor{
	float Yaw;
	float AccelX;
	float AccelY;
};

struct STM32toPC{
	float batrePC, batreMotor;
	float Yaw, AccelX, AccelY;
	int EKiri, Ekanan, Ebelakang;
	int EDKiri, EDKanan;
	int PotKiri, PotKanan;
	uint8_t kicked;
	uint8_t lifterState;
	uint8_t proxy;
	uint8_t PushButton;
	uint8_t CapacitroFull;
};


struct PCtoSTM32{
	int16_t MKiri, MKanan, MBelakang;
	int16_t DKiri, DKanan;

	uint8_t lifter_target;
	uint8_t kickPower;
};


extern PCtoSTM32 dataPC;
extern STM32toPC dataSTM;
extern razor Imu_Razor;


#ifdef __cplusplus
}
#endif

#endif /* INC_DEF_H_ */
