/*
 * barelangUDP.cpp
 *
 *  Created on: May 21, 2025
 *      Author: dika
 */
#include "barelangUDP.h"
#include "string.h"
PCtoSTM32 dataPC;
STM32toPC dataSTM;
razor Imu_Razor;
//extern PCtoSTM32 dataPC;
//extern STM32toPC dataSTM;
extern int count;
int32_t recv_len;

BarelangW5500::BarelangW5500() {
	init();
}
void BarelangW5500::init() {
	reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
	reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
	reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);
	this->udpConnect();
}

uint8_t BarelangW5500::udpConnect() {
	wizchip_setnetinfo(&myIpConf);

	if (getSn_SR(UDP_SOCKET) != SOCK_CLOSED)
		close(UDP_SOCKET);

	if (socket(UDP_SOCKET, Sn_MR_UDP, my_port, 0) == UDP_SOCKET) {
		socketCreated = 1;
		return 1;
	} else {
		socketCreated = 0;
		return 0;
	}
}

void BarelangW5500::udpProcess() {
	if (!socketCreated) {
		if (ElapsedTime(lastReconnectAttempt, DisconnectTime)) {
			this->udpConnect();
			SetTick(lastReconnectAttempt);
		}
		return;
	}
	udpReceive();
}

void BarelangW5500::udpReceive() {
	if (!socketCreated)
		return;
	if (getSn_RX_RSR(UDP_SOCKET) > 0) {

		recv_len = recvfrom(UDP_SOCKET, buf, sizeof(buf), dest_ip, &dest_port);
		if (recv_len > 0) {
			memcpy(&dataPC, buf, sizeof(buf));
			buf[recv_len] = '\0';  // Null-terminate the received string
			dataReceived = 1;
		}
	}
	else{
		dataReceived = 0;
		dataPC.DKanan = 0;
		dataPC.DKiri = 0;
		dataPC.MBelakang = 0;
		dataPC.MKiri = 0;
		dataPC.MKanan = 0;
		dataPC.kickPower  = 0;
		dataPC.lifter_target = 0;
	}
}

uint8_t BarelangW5500::udpSend(const void *_buff, size_t _len) {
	if (!socketCreated)
		return 0;

	int32_t result = sendto(UDP_SOCKET, (uint8_t* )_buff, _len, dest_ip,
			dest_port);

	if (result > 0) {
		SetTick(lastSendTime);
		return 1; // Success
	} else {
		// Handle send error
		if (result == SOCKERR_SOCKSTATUS) {
			// Socket not in proper state, try to reconnect
			socketCreated = 0;
		}
		return 0; // Failed
	}
}

uint8_t BarelangW5500::getSocketStatus() {
	return getSn_SR(UDP_SOCKET);
}

