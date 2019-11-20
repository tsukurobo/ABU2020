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
	//TWI�r�b�g���x���W�X�^
	//TWBR = 0xff;
	TWBR = 2;
	TWSR = 0x02;
	
	//TWI(�]���u)�A�h���X���W�X�^
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
	
	//TWI���䃌�W�X�^�iTWEA:�m�F����(ACK)���� TWEN:TWI���싖�j
	TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
	
	//����SLA+R��M�܂őҋ@
	while( !(TWCR & (1 << TWINT)));
	
	//����SLA+R��M,ACK����
	if( (TWSR & 0xF8) != ST_SLA_ACK)
		return -1;
	
	Udata send_data;
	send_data.l = data;
	
	for(int i = 0; i < data_byte - 1; i++) {
		//�f�[�^�o�C�g���M
		TWDR = send_data.u[i];
		TWCR = (1<<TWINT) | (1 << TWEA) | (1 << TWEN);
		
		//ACK��M�ҋ@
		while( !(TWCR & (1<<TWINT)));
		
		//�f�[�^�o�C�g���M,ACK��M
		if( (TWSR & 0xF8) != ST_DATA_ACK)
			return -2;
			
	}
		
	//�ŏI�f�[�^�o�C�g���M, NACK�ҋ@
	TWDR = send_data.u[data_byte - 1];
	TWCR = (1<<TWINT) | (1 << TWEA) |(1 << TWEN);
	
	//NACK��M�ҋ@
	while( !(TWCR & (1<<TWINT)));
	
	//�ŏI�f�[�^�o�C�g���M,NACK��M
	if( (TWSR & 0xF8) != ST_DATA_NACK)
		return -2;
	
	//����I��
	return 0;
}