/*
 * barelangMain.hpp
 *
 *  Created on: May 21, 2025
 *      Author: dika
 */

#ifndef INC_BARELANGMAIN_HPP_
#define INC_BARELANGMAIN_HPP_

#include "barelangUDP.h"

class BarelangMain{
public:
	BarelangMain();
	~BarelangMain();
	void Loop();
private:
	BarelangW5500 *stm_udp;
	uint8_t  startFirst;
};



#endif /* INC_BARELANGMAIN_HPP_ */
