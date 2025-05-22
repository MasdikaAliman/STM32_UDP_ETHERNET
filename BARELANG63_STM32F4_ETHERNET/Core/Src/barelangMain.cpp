/*
 * barelangMain.cpp
 *
 *  Created on: May 21, 2025
 *      Author: dika
 */
#include "barelangMain.hpp"

uint32_t mainTick = 0;
int asep = 0;
BarelangMain::BarelangMain() {
	stm_udp = new BarelangW5500();
}

BarelangMain::~BarelangMain() {
	delete stm_udp;
}

void BarelangMain::Loop() {
	if (ElapsedTime(mainTick, LoopDelay)) {
		SetTick(mainTick);
		dataSTM.AccelX = 150;
		dataSTM.AccelY = 100;
		dataSTM.Yaw = 180;
		dataSTM.EKiri = 100;
		dataSTM.Ebelakang = 100;
		dataSTM.Ekanan = 100;
		stm_udp->udpProcess();
		stm_udp->udpSend(&dataSTM, sizeof(dataSTM));
	}
}

