/*
 * TRCI2c.cpp
 *
 * Created: 2016/09/18 1:59:21
 *  Author: kazuma
 */ 

#include <avr/io.h>

//送信従装置動作の状態符号
#define ST_SLA_ACK 0xA8			//SLA_R 受信チェック
#define ST_DATA_ACK 0xB8		//送信パケットチェック
#define ST_DATA_NACK 0xC0		//終了orリピートチェック
//TWI状態値
#define SR_SLA_ACK  0xA8		
#define	SR_DATA_ACK	0xC0
#define	SR_ENDP_ACK	0xA0

class TRCTWI
{
	public:
	TRCTWI();
	~TRCTWI();
	
	int initSlaveSender(uint8_t id);
	int slaveSend(volatile const long &data, int data_byte);
	
	

};