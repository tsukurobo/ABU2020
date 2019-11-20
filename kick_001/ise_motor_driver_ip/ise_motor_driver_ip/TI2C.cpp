/*
 * TI2C.cpp
 *
 * Created: 2018/02/18 18:50:52
 *  Author: ip.sakura
 */ 
#include "TI2C.h"

const char postfix = '$';
Data TI2C_buf;
Data TI2C_buf_send;
void (*TI2C_recv_cb)(char*);
void (*TI2C_req_cb)(char*);


void TI2C_init(uint8_t address, void (*recv)(char*), void (*req)(char*)){
	TI2C_recv_cb = recv;
	TI2C_req_cb = req;
	I2C_init(address);
	I2C_setCallbacks(TI2C_char_received, TI2C_requested);
}

void TI2C_init_sync(uint8_t address, void (*recv)(char*), void (*req)(char*)){
	TI2C_recv_cb = recv;
	TI2C_req_cb = req;
	I2C_init_sync(address);
	I2C_setCallbacks(TI2C_char_received, TI2C_requested);
}


void TI2C_received(char *str) //slave <-
{
	TI2C_recv_cb(str);
}

void TI2C_requested() //→master
{
	static int i = 0;
	
	if (i == 0){
		// initialize
		TI2C_req_cb(TI2C_buf_send.str_buf);
	}
	// send data
	if (TI2C_buf_send.uint_buf[i] == '\0') { 
		I2C_transmitByte(postfix);
		i = 0;
	} else {
		I2C_transmitByte(TI2C_buf_send.uint_buf[i]);
		i++;
	}
}

void TI2C_char_received(uint8_t received_data) {
	//DDRC = 0b00000010;
	static int i = 0;
	// データに追加
	TI2C_buf.uint_buf[i] = received_data;
	// postfixが来た場合にコールバックを呼んで初期化
	if (TI2C_buf.str_buf[i] == postfix) {
		TI2C_buf.str_buf[i] = '\0';
		TI2C_received(TI2C_buf.str_buf);
		i = 0;
	} else {
		i++;
	}	
}

