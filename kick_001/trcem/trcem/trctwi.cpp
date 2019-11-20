/*
 * TRCTWI.cpp
 *
 * Created: 2016/09/18 1:59:44
 *  Author: kazuma
 */ 

#include "trctwi.hpp"

TRCTWI::TRCTWI()
{
	
}

TRCTWI::~TRCTWI()
{
	
}

int TRCTWI::initSlaveSender(uint8_t id)
{
	//TWIビット速度レジスタ
	//TWBR = 0xff;
	TWBR = 2;
	TWSR = 0x02;
	
	//TWI(従装置)アドレスレジスタ
	TWAR = (id << 1);
	return 0;
}

int TRCTWI::slaveSend(volatile const long &data, int data_byte)
{
	union Udata
	{
		long l;	
		uint8_t u[8];
	};
	
	//TWI制御レジスタ（TWEA:確認応答(ACK)許可 TWEN:TWI動作許可）
	TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
	
	//自宛SLA+R受信まで待機
	while( !(TWCR & (1 << TWINT)));
	
	//自宛SLA+R受信,ACK応答
	if( (TWSR & 0xF8) != ST_SLA_ACK)
		return -1;
	
	Udata send_data;
	send_data.l = data;
	
	for(int i = 0; i < data_byte - 1; i++) {
		//データバイト送信
		TWDR = send_data.u[i];
		TWCR = (1<<TWINT) | (1 << TWEA) | (1 << TWEN);
		
		//ACK受信待機
		while( !(TWCR & (1<<TWINT)));
		
		//データバイト送信,ACK受信
		if( (TWSR & 0xF8) != ST_DATA_ACK)
			return -2;
			
	}
		
	//最終データバイト送信, NACK待機
	TWDR = send_data.u[data_byte - 1];
	TWCR = (1<<TWINT) | (1 << TWEA) |(1 << TWEN);
	
	//NACK受信待機
	while( !(TWCR & (1<<TWINT)));
	
	//最終データバイト送信,NACK受信
	if( (TWSR & 0xF8) != ST_DATA_NACK)
		return -2;
	
	//正常終了
	return 0;
}