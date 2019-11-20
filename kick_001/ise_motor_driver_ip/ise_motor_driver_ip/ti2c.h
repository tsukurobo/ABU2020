/*
 * TI2C.h
 *
 * Created: 2018/02/18 18:51:12
 *  Author: ip.sakura
 */ 


#ifndef TI2C_H_
#define TI2C_H_

#include "I2CSlave.h"

#define SIZE 512

union Data {
	uint8_t uint_buf[SIZE];
	char str_buf[SIZE];
};

union Data_Bit {
	uint8_t uint_buf;
	char str_buf;
};

	//Data TI2C_buf;
	void TI2C_init(uint8_t address, void (*recv)(char*), void (*req)(char*));
	void TI2C_init_sync(uint8_t address, void (*recv)(char*), void (*req)(char*));
	void TI2C_send_str(char* buf);
	void TI2C_get_str(char* buf);
	void TI2C_received(char* str);
	void TI2C_requested();
	void TI2C_char_received(uint8_t received_data);

//class TI2C
//{
//	Data buf;
//	public:
//	TI2C(uint8_t address, void (*recv)(char*), void (*req)());
//	void send_str(char* buf);
//	void get_str(char* buf);
//	void received(char* str);
//	void requested();
//	private:
//	void char_received(uint8_t received_data);
//	void (*recv_cb)(char*);
//	void (*req_cb)();
//};


#endif /* TI2C_H_ */