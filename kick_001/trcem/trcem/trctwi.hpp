/*
 * TRCI2c.cpp
 *
 * Created: 2016/09/18 1:59:21
 *  Author: kazuma
 */ 

#include <avr/io.h>

//���M�]���u����̏�ԕ���
#define ST_SLA_ACK 0xA8			//SLA_R ��M�`�F�b�N
#define ST_DATA_ACK 0xB8		//���M�p�P�b�g�`�F�b�N
#define ST_DATA_NACK 0xC0		//�I��or���s�[�g�`�F�b�N
//TWI��Ԓl
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