/*
 * barelangUDP.hpp
 *
 *  Created on: May 21, 2025
 *      Author: dika
 */

#ifndef INC_BARELANGUDP_H_
#define INC_BARELANGUDP_H_
#include "socket.h"
#include "wizchip_conf.h"
#include "main.h"
#include "def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UDP_SOCKET			1

extern SPI_HandleTypeDef hspi1;

class BarelangW5500 {
public:
	BarelangW5500();

	void init();
	uint8_t udpConnect();
	void udpProcess(); // Call this regularly from main loop
	void udpReceive();
	uint8_t udpSend(const void *_buff, size_t _len);
	uint8_t getSocketStatus();
//	void checkConnection();
	uint8_t isDataReceived(){
		return dataReceived;
	}

	void ClearDataReceived(){
		dataReceived = 0;
	}

private:

	uint8_t buf[sizeof(dataPC)];
	uint8_t socketCreated = 0;       	              // Socket status flag
	uint8_t dataReceived;               // Flag indicating new data is available
	uint32_t lastSendTime = 0;              // Timestamp of last successful send
	uint32_t lastRecvTime = 0;         // Timestamp of last successful reception
	uint32_t lastReconnectAttempt = 0; // Timestamp of last reconnection attempt

	uint8_t dest_ip[4] = { 192, 168, 0, 63 };
	uint16_t dest_port = 2023;

	uint16_t my_port = 2002;
	wiz_NetInfo myIpConf = { .mac = { 0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef },
			.ip = { 192, 168, 0, 4 },   // stm32 client by my choice
			.sn = { 255, 255, 255, 0 }, .gw = { 192, 168, 0, 1 }, .dns = { 8, 8,
					8, 8 }, .dhcp = NETINFO_STATIC };
	static void W5500_Select(void) {
		HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
	}

	static void W5500_Unselect(void) {
		HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
	}

	static void W5500_ReadBuff(uint8_t *buff, uint16_t len) {
		HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
	}

	static void W5500_WriteBuff(uint8_t *buff, uint16_t len) {
		HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
	}

	static uint8_t W5500_ReadByte(void) {
		uint8_t byte;
		W5500_ReadBuff(&byte, sizeof(byte));
		return byte;
	}

	static void W5500_WriteByte(uint8_t byte) {
		W5500_WriteBuff(&byte, sizeof(byte));
	}

};

#ifdef __cplusplus
}
#endif

#endif /* INC_BARELANGUDP_H_ */
