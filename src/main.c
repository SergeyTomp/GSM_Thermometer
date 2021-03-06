#include <avr/io.h>
#include <util/delay.h>		// ��������
#include <avr/interrupt.h>	// ����������
#include <stdlib.h>
#include <avr/pgmspace.h>	// ������ �� ����
#include <avr/eeprom.h>		// ������ � �����
#include <string.h>			// ������� ������ �� ��������
#include <ctype.h>			// ��� isdigit()

//#define DEBUG
//���� define ��� LCD
#define RS					PORTD3		// ����� ������ �����, �� �������� ��������� ������� RS � ���
#define EN					PORTD2		// ����� ������ �����, �� �������� ��������� ������� EN � ���
#define LCD_COM_PORT		PORTD		// ���� ��� ������ ������ � ��� (������� � ������ - ����� ���� �� �� ����� �����!!!)
//#define LCD_COM_PORT_DDR		DDRD 		// ������� ����������� ������ � �����, ���� ���������� ����� ������ ���, ��.����
#define LCD_DAT_PORT		PORTD		// ���� �������� ������ (� ������ � ������ ������)� ���
#define LCD_DAT_PORT_DDR	DDRD		// ������� ����������� ������ �����, ���� ��������� ��� ������� ������ (� ������ � ������ ������)
/* ���-����� ��������� ������ � �������� �� � ���� LCD_DAT_PORT ������������ �������������� ������� �������, � �������
��������� ���. � ������ ������ ������������ ������ 4-7 �����. */

//���� define ��� OW
#define DDR_OW_PORT		DDRB			// ������� ����������� ������ �����, � ������ �� ������� �������� ���������� ����� 1-Wire
#define OW_PORT			PORTB			// ����, � ������ �� ������� �������� ���������� ����� 1-Wire
#define OW_PIN			PINB			// ������� ����� ������ ����� 1-Wire, � ������ �� ������� �������� ���������� ��� �����
#define OW_PIN_NUM		0				// ����� PIN, � �������� ���������� ����� 1-Wire, ��� ������� _BV()
#define bit_msk			(1<<OW_PIN_NUM) // ������� ����� ��� �������� ������� �� ����� 1-Wire �� ��������������� ����

//���� ������ define
#define N_DIGS			5				// ������ ������� ���� ��� �������������� ����� � ������ ��� LCD, ������������ ������������ ������������� ���������� �� LCD ����� +1 ��� ������������ 0
#define N_NAME			8				// ������ ������� ��� ����� ����������
#define EEP_MEM			1024			// ����� ����� ��� ���������� ������ ��������� � ������������� ������, ����� ��� ����������� ������������� ���������� ��������� � �����
#define IND_PAUSE		120 			// ����� �� ��������� ���������� �����, 2�
#define WAIT_LIM		900 			// ����� ��� �������� �������� ������������, 15�

//���� define ��� ������������� ������
#define SRC_PORT		PORTB			// ����, � ������ �� ������� �������� ���������� ������ ������ ������ (����� ���� ���� � OW line)
#define DDR_SRC_PORT	DDRB			// ������� ����������� ������ �����, � ������ �� ������� �������� ���������� ������ ������ ������ (����� ���� ���� � OW line)
#define SRC_PIN			PINB			// ������� ����� ��������� ������
#define SRC_PIN_NUM		1				// ����� PIN, � �������� ���������� ������, ��� ������� _BV()
#define src_msk			(1<<SRC_PIN_NUM)// ������� ����� ��� �������� ������� �� ������ �� ��������������� ����

//���� define ��� ������ ������
#define CNT_QUICK		5				// ���������� ������������ �� �����������, ���� ������ - ������ �����
#define CNT_SLOW		15				// ���������� ������������ �� ������� �������
#define QUICK			1				// ������������ ��������� �������
#define SLOW			2				// ������������ �������� �������
#define RELEASED		0				// ��������� ������ - ��������
#define PRESSED			1				// ��������� ������ - ������

// ���� define ��� ���������� ��������� gsm ������
#define LED_PORT		PORTB			// ����, � ������ �� ������� �������� ��������� ��������� (����� ���� ���� � OW line)
#define DDR_LED_PORT	DDRB			// ������� ����������� ������ �����, � ������ �� ������� �������� ��������� ��������� (����� ���� ���� � OW line)
#define LED_PIN_NUM		5				// ����� PIN, � �������� ��������� ���������, ��� ������� _BV()
#define GSM_LVL_TIME	3600			// �������� ������� ������ ������� 60c

//���� define ��� USART-GSM
#define MYUBRR			103					// �������� usart 9600
#define RX_RING_SIZE	128					// ������ ������ ���� 128, ����� ������ � ������� ��.��� ������ 64 ����, � ����� ���������� ������ ������ ������� 2
#define RX_IND_MSK		(RX_RING_SIZE - 1)	// ����� �������� ���������� ������ �������� ��� ��������� ������� ��� ��������  RX_Index ����� 0
#define TX_RING_SIZE	64					// ������ ������ ���� 64, ������������ ����� ������� � ����� 37 ����, �� ����� ���������� ������ ������ ������� 2
#define TX_IND_MSK		(TX_RING_SIZE - 1)	// ����� �������� ���������� ������ ����������� ��� ��������� ������� ��� �������� TX_Index ����� 0

//���� define ��� ������ � �������
#define QUEUE_SIZE			16					// ������ ������ ������� ������������
#define INC_TASK_SIZE		8 					// ������ ������ ����� �� ������ ���
#define OUT_TASK_SIZE		4					// ������ ������ ����� �� �������� ���
#define CMD_TASK_SIZE		8					// ������ ������ ����� �� �������� ������
#define QUEUE_IND_MSK		(QUEUE_SIZE - 1)	// ����� ������ ������� ������������
#define INC_TASK_IND_MSK	(INC_TASK_SIZE - 1)	// ����� ������ ����� �� ������ ���
#define OUT_TASK_IND_MSK	(OUT_TASK_SIZE - 1)	// ����� ������ ����� �� �������� ���
#define CMD_TASK_IND_MSK	(CMD_TASK_SIZE - 1)	// ����� ������ ����� �� �������� ������
#define PWR_UP_ANS_TIMER	900					// ������ ��������� ������ ������ ��� ���������
#define PAUSE_CNT_MAX		60					// ������������ ����� ����� ����� ���������� ����� �� RX ����� XOFF (PAUSE_CNT_MAX/60)c
#define MSG_SIZE			(RX_RING_SIZE + 1)	// ������ ������� ��� �������� �� ������ ��������; +1� � Rx-������ �.�. � ����� ������ ����� 0 ���� ��� �������� 128�
#define TODO_MAX			25					// ������ ������� ��� ������ ������� �����������, ������ ���� ������ RX_RING_SIZE
#define RST_PORT			PORTB				// ����, � ������ �� ������� �������� ��������� ����� ������ ������(����� ���� ���� � OW line)
#define DDR_RST_PORT		DDRB				// ������� ����������� ������ �����, � ������ �� ������� �������� ��������� ����� ������ ������(����� ���� ���� � OW line)
#define RST_PIN_NUM			2 					// ����� ������, � �������� ��������� ����� ������ ������, ��� ������� _BV()

// ���� ��������� ��������� �� ����
unsigned char absence[] 		PROGMEM = "��� ��������";		// ������ �� ����� ������ ������������� �������� ����� ������ � �������� ���������� �������
unsigned char no_answer[] 		PROGMEM = "��� ������ ����.";	// ������ �� ������ ���������� ������� ����� ������ ������ ���� ����-������
unsigned char present_n[] 		PROGMEM = "������� ����. ";		// ���������� ���������, ���������� ��� ��������� ���������� �������
unsigned char dev_excess[] 		PROGMEM = "����� ��������";		// ������ � �������� ��������� ���������� �������, ���� ���������� �������� �������� 50
unsigned char error[] 			PROGMEM = "������ CRC-ID ";		// ������ �� ����� �������� CRC ID ����� ��������� ���������� �������, ��������� � �������,
unsigned char init_n[]			PROGMEM = "�������.����. ";		// ���������� ��������, ��������� �������� CRC ����� ��������� ���������� �������
unsigned char init_srch[] 		PROGMEM = "��������� �����?";	// ������ ��� ���������� � ����� ������ ��������, ���� �� ����������� ��������� ���������� �������
unsigned char scratch_err[] 	PROGMEM = "��.CRC-����. ";		// ������ ��� �������� CRC ������, ����������� �� ��������, ��������� � �������
unsigned char no_answer_n[] 	PROGMEM = "��� ������ ";		// ������ �� ����� ������ ������ �������� ��� ���������� � main - ���� ������ �� �������, �� ����.���� == FF, ��������� ��� �������
unsigned char ow_check[] 		PROGMEM = "����� �����";		// ��������� � main ��� ���������� ����� �� ����� ����������
unsigned char correction[] 		PROGMEM = "�������������";		// ��������� � add_ID ��� �����
unsigned char subst_add[] 		PROGMEM = "������/������-�";	// ��������� � add_ID ��� ����� � ���� ������/��������
unsigned char total_qty[] 		PROGMEM = "����� ���������";	// ��������� � add_ID ����� ������� ������
unsigned char plug_in[] 		PROGMEM = "���������� ����.";	// ��������� � add_ID ����� ������� ������
unsigned char press_btn[] 		PROGMEM = "� ������� ������";	// ��������� � add_ID ����� ������� ������
unsigned char mem_full[]		PROGMEM = "������ ���������";	// ��������� � add_ID, ���� ���������� �������� ��� 50, � ���������� ���
unsigned char new_dev_fnd[] 	PROGMEM = "������� �����";		// ������� ������ ��������� � add_ID, ���� ������� �����
unsigned char nev_dev_add[] 	PROGMEM = "��������� �����";	// ������� ������ ��������� � add_ID � ����� ����� ����������
unsigned char element[] 		PROGMEM = "����������";			// ������ ������ ��������� � add_ID, ���� ������� ��� ��������� �����
unsigned char substitute[] 		PROGMEM = "��������?";			// ��������� � add_ID � ����� ������, ���� ����� ���������� ����-��, ��������� ��� �������
unsigned char add_to_end[] 		PROGMEM = "��������?";			// ��������� � add_ID � ������ ����� ����������
unsigned char done[] 			PROGMEM = "������ ���������";	// ��������� � add_ID ����� ������� ������ �����������
unsigned char no_new[] 			PROGMEM = "����� ����-� ���";	// ��������� � add_ID ���� ������ ���� ���� ������ ������ ID
unsigned char delete[] 			PROGMEM = "�������?";			// ��������� - ������ ����� ���� �������������
unsigned char del_done[] 		PROGMEM = "�������";			// ��������� �� ����� �������� ����������
unsigned char t_error[] 		PROGMEM = "Wrong temp.limit";	// ��������� ��� ������� �������� ������������ ����������� � ������
unsigned char com_error[] 		PROGMEM = "Incorrect command";	// ��������� ��� ������ � ������ �������
unsigned char frame_err[]		PROGMEM = "������ �����!"; 	// ��������� ��� ������ ����� ����� �� Rx
unsigned char name_error_ren[] 	PROGMEM = "REN-Incorrect name";	// ��������� ��� ��������� ����� � ������ ������� REN...
unsigned char name_error_al[] 	PROGMEM = "AL-Incorrect name";	// ��������� ��� ��������� ����� � ������ ������� AL...
unsigned char name_error_sms[] 	PROGMEM = "SMS-Incorrect name";	// ��������� ��� ��������� ����� � ������ ������� SMS...
unsigned char name_error[] 		PROGMEM = "Incorrect name";		// ��������� ��� ��������� ����� � ������ �������
unsigned char rename_ok[] 		PROGMEM = "Rename done";		// ��������� ��� �������� ��������� �����
unsigned char t_min[] 			PROGMEM = "Tmin";				// ����� ������ ��� ������������� ��������� ������������ ������� �����������
unsigned char t_max[] 			PROGMEM = "Tmax";				// ����� ������ ��� ������������� ��������� ������������� ������ �����������
unsigned char t_low[] 			PROGMEM = "T<";					// ����� ������ ��� ���������� � �������� ����������� ���� ������������ ������
unsigned char t_high[] 			PROGMEM = "T>";					// ����� ������ ��� ���������� � ��������� ����������� ���� ������������� ������
unsigned char sms_send []		PROGMEM = "SMS";				// ����� ������ ��� ������������� ��������� sms � ������ ����������� �� �������
unsigned char on []				PROGMEM = "ON";					// ����� ������ ��� ������������� ��������� sms � ������ ����������� �� �������
unsigned char off []			PROGMEM = "OFF";				// ����� ������ ��� ������������� ��������� sms � ������ ����������� �� �������
unsigned char blank []			PROGMEM = " ";					// ������, ��� �����
unsigned char crash []			PROGMEM = "FAIL!";				// ����� ������ ��� ��������� �� ������ �� �����
unsigned char tx_ring_ovf[]		PROGMEM = "TX-���. ������!";	// ��������� �� lcd ��� ������������ TX_ring
unsigned char rx_ring_ovf[]		PROGMEM = "RX-���. ������!";	// ��������� �� lcd ��� ������������ RX_ring
unsigned char quick[]			PROGMEM = "QUICK";				// �������� ���������
unsigned char slow[]			PROGMEM = "SLOW";				// �������� ���������
/* unsigned char sim900[]			PROGMEM = "SIM900 ";			// �������� ���������, �������������� ��� ������� ���900 */
/* unsigned char not_rdy[]			PROGMEM = "�� �����";			// �������� ���������, �������������� ��� ������� ���900 */
unsigned char out_ring_ovf[]	PROGMEM = "���.���.������!";	// ��������� �� lcd ��� ������������ ������� ����� �� ��������
unsigned char inc_ring_ovf[]	PROGMEM = "��.���.������!";		// ��������� �� lcd ��� ������������ ������� ����� �� ����/������
unsigned char cmd_ring_ovf[]	PROGMEM = "���.���.������!";	// ��������� �� lcd ��� ������������ ������� ������
unsigned char q_ring_ovf[]		PROGMEM = "����.������!";		// ��������� �� lcd ��� ������������ �������� �����
unsigned char gsm_scale[]		PROGMEM = {0,3,6,10,13,16,19,23,26,29,32,35,39,42,45,48,52,55,58,61,65,68,71,74,77,81,84,87,90,94,97,100};//������ �������
unsigned char gsm[]				PROGMEM = "GSM";				// ����� ������ � ��������� ��� ������� �������
unsigned char BEELINE[]			PROGMEM = "Bee";				// ������
unsigned char MTS[]				PROGMEM = "MTS";				// ���
unsigned char MEGAFON[]			PROGMEM = "Meg";				// �������
unsigned char TELE2[]			PROGMEM = "Tel";				// ����2
unsigned char USERS[]			PROGMEM = "USERS";				// ������ ��� �� ������� ��������� �������������
unsigned char DELETE[]			PROGMEM = "DELETE";				// ������ ��� �� �������� �������������
unsigned char DENIED[]			PROGMEM = "ACCESS DENIED";		// ��� �� ������ � �������

//���� ��������� ����� �� ���� ��� ������ � �������
unsigned char ANS_OK[]			PROGMEM = "\r\nOK\r\n";		// �������� ��
unsigned char AT_CMGD[]			PROGMEM = "AT+CMGD=";		// ������� ��� ("=1,4" - ������� ���)
unsigned char AT_CMGS[]			PROGMEM = "AT+CMGS=\"";		// ������� ���
unsigned char ANS_ENT[]			PROGMEM = "\r\n> ";			// ����� ������ �����
unsigned char ANS_CMGS[]		PROGMEM = "\r\n+CMGS: ";	// ��� ����������
unsigned char CTRL_Z[]			PROGMEM = {0x1A, 0};		// Ctrl-Z - ������ ���������� ���
unsigned char AT_CMGR[]			PROGMEM = "AT+CMGR=";		// ��������� ���
unsigned char ANS_CMGR[]		PROGMEM = "\r\n+CMGR: ";	// �����
unsigned char AT_BUSY[]			PROGMEM = "AT+GSMBUSY=1";	// ������ ���� �������� ������ (1-���������, 0 - ���������)
unsigned char ANS_CMTI[]		PROGMEM = "\r\n+CMTI: ";	// �������� ���
unsigned char AT_CSQ[]			PROGMEM = "AT+CSQ";			// ������ ������ �������
unsigned char ANS_CSQ[]			PROGMEM = "\r\n+CSQ: ";		// ����� �� ������ ������ �������
unsigned char CRLF[]			PROGMEM = "\r\n";			// ������� CRLF
unsigned char NEW_LN[]			PROGMEM = "\n";				// ������� ������ ��� ���, ������� �� ��������� �� ������� \r\n
unsigned char TEXT_1_4[]		PROGMEM = "1,4";			// "1,4" - ������� ��� ���
//unsigned char PHONE[]			PROGMEM = "+79052135678";	// ����� ��������, ���� �����
unsigned char QUOTES[]			PROGMEM = "\"";				// ����������� ������� ��� ���������� � ����� �������� ��� �������� ���
unsigned char CALL_RDY[]		PROGMEM = "Ready\r\n";		// Call Ready - ��������� URC ������ ����� ��������� ��� ������
unsigned char BALANCE[]			PROGMEM = "BALANCE";		// ����� ��� ��� ������� �������
unsigned char AT_CUSD[]			PROGMEM = "AT+CUSD=1,";		// USSD ������ ������� � ������������ ��������� ������ ��������
//unsigned char TEXT_100[]		PROGMEM = "#100#\"";		// ����� �������, ���� �����
unsigned char ANS_CUSD[]		PROGMEM = "\r\n+CUSD: ";	// ����� �� USSD ������ �������
unsigned char AT_COPS[]			PROGMEM = "AT+COPS?";		// ������ ���������
unsigned char ANS_COPS[]		PROGMEM = "\r\n+COPS: ";	// ����� �� ������ ���������
unsigned char AT_CPBF[]			PROGMEM = "AT+CPBF=";		// ������ ������ �� ��� �� �����
unsigned char ANS_CPBF[]		PROGMEM = "\r\n+CPBF: ";	// ����� �� ������ ������ �� ��� �� �����
unsigned char USER_0[]			PROGMEM = "ADMIN";			// ��� ������
unsigned char USER_1[]			PROGMEM = "USER_1";			// ��� ������������ 1
unsigned char USER_2[]			PROGMEM = "USER_2";			// ��� ������������ 2
unsigned char BALANS[]			PROGMEM = "BALANS";			// ����� balans
unsigned char AT_CPBW[]			PROGMEM = "AT+CPBW=";		// ������ �� ������ � ��� � ��������� ������
unsigned char CELL_1[]			PROGMEM = "1,\"";			// ������ �� ������ � ��� � ������ 1 � ������������ ��������� ������ ��������
unsigned char CELL_2[]			PROGMEM = "2,\"";			// ������ �� ������ � ��� � ������ 2 � ������������ ��������� ������ ��������
unsigned char CELL_3[]			PROGMEM = "3,\"";			// ������ �� ������ � ��� � ������ 3 � ������������ ��������� ������ ��������
unsigned char CELL_4[]			PROGMEM = "4,\"";			// ������ �� ������ � ��� � ������ 4 � ������������ ��������� ������ ��������
unsigned char TEXT_145[]		PROGMEM = "\",145,\"";		// ����� ������� �� ������ �������� � ��� � ����. ���. ������ � ����. ���. �����
unsigned char TEXT_129[]		PROGMEM = "\",129,\"";		// ����� ������� �� ������ ���� ������� � ��� � ����. ���. ������ � ����. ���. �����


// ���� ���������� ���������� � ��������
typedef struct //������� ���� ��� ������
{
    uint8_t active : 1;		// ������� ���������� �� �����
    uint8_t lt_alarm : 1;	// ������ ����� ����������� ������� ����
    uint8_t ht_alarm : 1;	// ������� ����� ����������� ��������
    uint8_t line_alarm : 1;	// ������� ���������� � �������� ���������
    uint8_t sms_T_0 : 1;	// ��������� sms ������ ��� � < tmin ��� � > tmax
    uint8_t sms_T_1 : 1;	// ��������� sms �����1 ��� � < tmin ��� � > tmax
    uint8_t sms_T_2 : 1;	// ��������� sms �����2 ��� � < tmin ��� � > tmax
    uint8_t relay : 1;		// �������/��������� ���� ��� ����� �������
} bit_set;

typedef struct //��������� ��� ���������� ����������
{
    unsigned char name[N_NAME];		// ��� 1W ����������
    unsigned char code[8];			// ��� 1W ����������
    char tmax;						// ������������ �
    char tmin;						// ����������� �
    bit_set flags;					// ����� ���������
} device;							// ��������� ��� ���������� 1W ����������

/*typedef union //����������� ��� ���������� ����������
{
	unsigned char name[N_NAME]; // ��� 1W ����������
	unsigned char code[8]; // ��� 1W ����������
	char tmax; // ������������ �
	char tmin; // ����������� �
	bit_set flags;// ����� ���������
} dev_param;*/

device buffer;																// ���������� ��� ������ ��� <-> �����
device ee_arr [2] EEMEM;													// o�������� ������ �������� � �����, 2 - ���� ��������, ��� ������
unsigned char *location;													// ��������� �� ��� 1W ����������, ����������� ������ ������ ����� �� ���������
unsigned char dev_name[N_NAME] = {'S', 'e', 'n', 's', '.', '_', '_', '_'};	// "Dxxxxxx" - ��� ���������� �� ���������, � ���� ��������� ASCII ��� ����������� ������
const uint16_t dev_qty = (EEP_MEM - 1);										// ���������, ���������� �������� ������ ��������� ������ � ����� ��� ���������� ���������� ������������� ���������
const uint16_t dev_last_n = (EEP_MEM - 2);									// ���������, ���������� �������� ������ ������������� ������ � ����� ��� ���������� ���������� ������ ����������, ������������ � ������ � ������
/*����� ��� ����������� ����������� �������������� ��������� � ������ ��� ���������� � ������������� ������ ����� �������� ��������� ����� ���� �������������� �������� */
const uint8_t n_max = (uint8_t)((EEP_MEM - 2) / sizeof(device));			// ����������� ���������� ���������� �������� � �����
uint8_t scratchpad [9];														// ������ ������, ����������� �� �������� DS18B20
int8_t *t_all = NULL;														// ���������� ��������� �� ��������� (� main) ������ ����� ������ ��������� ���������� ���������� �� ������ (last_t[n_max])
/* ��-�� ������������ ������� ������� last_t[n_max] ��� �� ������� ���������� �����, ����� ������ �������� � �������, � ��������� ���������� ����� ���� ��������� */

//���� ����������-��������� ��� ��������� ������ gsm
volatile uint8_t flash_cnt; 	// ������� ���������� ��� ��������� ������ gsm ��������� �����
volatile uint8_t flash_num; 	// ����� ��������� �������
volatile uint8_t pause_num; 	// ����� ��������� �������� ���� ����� ������� �������
volatile uint8_t gsm_lvl; 		// ������� gsm �������
volatile uint16_t time_gsm; 	// ������� ��� �������� ������ ������� gsm-������
volatile uint8_t gsm_lvl_req;	// ���� ������� ������ gsm �������

typedef struct // ��������� ��� ������ �������, �� ��� �������� ���� (�����)
{
    uint8_t name[N_NAME];	// ��� ����������
    unsigned char dig_1;	// ������ ����� �����������
    unsigned char dig_2;	// ������ ����� �����������
    unsigned char dig_3;	// ���������� ����� �����������
    unsigned char sign;		// ���� �����������
} line;
line line_up; // ������� ������ �����
line line_dn; // ������ ������ �����

volatile uint16_t btn_cnt = 0;				// ������� ���������� ��������� ������
volatile uint8_t btn_state = 0;				// ��������� ������ - ������/��������
volatile uint8_t btn_time = 0;				// ����� ������� ������, ��������� ������ � ����������� ���������� �� ������������, �� ����� ������������ ������� ��� �������� �� �������� ���� �� ����������
volatile uint8_t press_time = 0;			// ��� ����� ������� ������ ������� ������
volatile uint16_t int_cnt = 0;				// ������� ������������ ������� TIMER0, ����� ������ ����� ������ �� ������������ �������
volatile uint8_t delay_cnt = IND_PAUSE;		// ������ �������� �� ��������� ����� �������� ����� � lcd
volatile uint16_t wait_timer = WAIT_LIM + 1;// ������ �������� �������� ������������ ����������, +1 ����� ��� ������ ����� � ���� �� ��������� �������� if(wait_timer==WAIT_LIM)
//volatile uint16_t time_gsm = 0;			// ������� ��� �������� gsm-������

//���� ����������, �������� � �������� ��� ������ � �������
typedef struct	//��������� �������� ������ ������������
{
    uint8_t flag_1 : 1;	// ��� 1
    uint8_t flag_2 : 1;	// ��� 2
    uint8_t flag_3 : 1;	// ������
    uint8_t flag_4 : 1;	// ������
    uint8_t flag_5 : 1;	// ������
    uint8_t flag_6 : 1;	// ������
    uint8_t flag_7 : 1;	// ������
    uint8_t flag_8 : 1;	// ������
}tracker;

typedef struct	//��������� ����������� ���
{
    uint8_t sms_type;					// ������� ������� ��� (������������ ��������)
    uint8_t dev_num;					// ����� ���������� (����������� ��� �������������)
    int8_t param;						// ��������, ���� ������ ����������� �� ������� ����������� ��������� ��� ��������� �� ����������(����������� ��� �������������)
    uint8_t *ptr;						// ��������� �� ������ �������� (���� ��� �������� ���� �������)
    uint8_t person;						// ��� ��������, ����������� ����������� ������ � ����� �������
}sms_mask;

typedef struct 	//��������� ������ ����� ���, 4 �����
{
    tracker step;						// ����� ��������
    uint8_t	sms_num[3];					// ������ �������� ����������� ������ ��� ��� ������
}inc_task;

typedef struct //��������� ������ �������� ���, 21 ����, ������� �� 8 ����� �������� ���� � ����������, ���������� (����� ����� ���) � ������� �������� (������� �������� ���������).
{
    tracker step;						// ����� �������� ��������
    sms_mask sms_txt; 					// ��������� ��� ������ ���������� ���������� ���, ������ ������ ������ ��� �������� �� ����������
}out_task;

typedef struct //��������� ������ �������� ������ � �����, 5 ����
{
    tracker step;						// ����� ��������
    uint8_t* cmd;						// ��������� �� AT-�������
    uint8_t* pgm_par_1;					// ��������� �� �������� AT-������� �� ����
    uint8_t* pgm_par_2;					// ��������� �� �������� AT-������� �� ����
    uint8_t* ram_par;					// ��������� �� �������� AT-������� �� ���
    uint8_t* pgm_par_3;					// ��������� �� �������� AT-������� �� ����
    uint8_t* pgm_par_4;					// ��������� �� �������� AT-������� �� ����
    uint8_t* pgm_par_5;					// ��������� �� �������� AT-������� �� ����
}cmd_task;

struct tel_list							// ��������� �������� ��������� ������������� � USSD-��������
{
    uint8_t balance [6];				// ��� USSD-������� �������
    uint8_t reserv [4];					// ��������� ��� ��������� ������ ��� ���� ���� �������
    uint8_t ans_to;						// ���� ��������� ��� � ��������
    uint8_t phone_0 [13];				// �������������
    uint8_t phone_1 [13];				// ������� ������������ ��� ���� ���������
    uint8_t phone_2 [13];				// ������� ������������ ��� ���� ���������
};

sms_mask sms_buff;						// ����� ��� ������ ���
volatile unsigned char pause_cnt;		// ������� ����� ����� ����� ���������� ����� � ������ ��������
volatile unsigned char msg_upld; 		// ���� ��������� ����� ������� �� ������ � ���������� �������� � �� ������ ��������� � msg
volatile unsigned char modem_rdy;		// ���� ���������� ������
unsigned char cmd_task_H, cmd_task_T, inc_task_H, inc_task_T, out_task_H, out_task_T, queue_H, queue_T = 0;//������� ������� � �������
volatile uint16_t ans_cnt;				// ������� ��������� �� ��������� ������ OK �� ������ �� �������
uint16_t ans_lim;						// ������ ��������� �� ��������� ������ ������ �� �������
uint8_t handl_res;						// ��������� ������ ����������� ������
uint8_t msg[MSG_SIZE]; 					// ������ ��� �������� �� ���������� ������     	#############
uint8_t todo_txt [TODO_MAX];			// ������ ��� �������� ������ ������� �����������	#############
uint8_t mod_ans;						// ������ ����������� �������� � ����������� �� ������ ������ �� ������� ( "��", ">" � ��.)
enum {OK = 1, INVITE};					// �������� �������� ��� mod_ans, ��.����
/* �������� �������� ��� �������� ��� */
enum {FAIL = 1, ALARM, DONE, ALL, REN_DONE, NAME_ERR, MIN_LIM_SET, MAX_LIM_SET, LIM_ERR,
    SMS_ON, SMS_OFF, T_LOW, T_HIGH, COM_ERR, MONEY, BAL_TEL, ADMIN, ABNT_1, ABNT_2, MEMBERS, DENY};
tracker RESET;							// ������ ������� ���� ��� ������ ������������ ������
struct tel_list phones = {	{'#','0','0','0','#','\0'}, {'0','0','0','\0'}, 0,				// ��������� � ����������
                              {'0','0','0','0','0','0','0','0','0','0','0','0','\0'},
                              {'0','0','0','0','0','0','0','0','0','0','0','0','\0'},
                              {'0','0','0','0','0','0','0','0','0','0','0','0','\0'}};
unsigned char gsm_sig;					// ������� �������� ������ ������� �� ������ ������ �� ������ (0...31->0...100)

//	���� ���������� � �������� ��� ������ � USART
/*	�������� ������ ����� Rx � Tx - ������� �������� ������, ����� �����/������
	������ ������� ������������ � ������� ����� �������� ��-�������!!! */
volatile uint8_t RX_IndexIN;	// �������� ������ ���������� ������ ��������, ����� ����������� volatile, �������� ������ � ����������
uint8_t RX_IndexOUT;			// ��������� ������ ���������� ������ ��������
uint8_t RX_ring[RX_RING_SIZE];	// ������ ��� ���������� ������ ��������
uint8_t TX_IndexIN;				// �������� ������ ���������� ������ �����������,
volatile uint8_t TX_IndexOUT;	// ��������� ������ ���������� ������ �����������; ����� ����������� volatile, �������� ������ � ����������
uint8_t TX_ring[TX_RING_SIZE];	// ������ ��� ���������� ������ �����������

//���� ������� � �������� ��� ������ � ���������
/*	�������� ������ ����� ������� � �����: ������� ������/�����, ����� �������� ������
	�������� ���������� �� ��������� ����� Rx � TX!!! */
inc_task RD_SMS[INC_TASK_SIZE];								//��������� ������ ����� ������ ���
out_task WR_SMS[OUT_TASK_SIZE];								//��������� ������ ����� �������� ���
cmd_task WR_CMD[CMD_TASK_SIZE];								//��������� ������ ����� �������� ������
uint8_t (*HANDLERS[QUEUE_SIZE])(void);						//��������� ������� ���������� �� �������-����������� �������
uint8_t read_sms (void);									//��������� HANDLER ������ ���
uint8_t send_sms (void);									//��������� HANDLER �������� ���
uint8_t parser(void);										//��������� HANDLER ������� ������
uint8_t send_cmd (void);									//��������� HANDLER �������� ������� � �����
void inc_to_queue(uint8_t*);								//��������� ������� ���������� � ������� ������ ������ ���
void out_to_queue(sms_mask*);								//��������� ������� ���������� � ������� ������ �������� ���
void cmd_to_queue(uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*);	//��������� ������� ���������� � ������� ������ �������� �������
void to_do(void);											//��������� ������� ������� � ���������� ������ �� ������ ���

// ������� ������ ������� � ���
void lcd_com(unsigned char p)
{
    LCD_COM_PORT &= ~(1 << RS);							// RS = 0 (������ ������)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p & 0xF0); 	// �������� ������� ����
    LCD_COM_PORT |= (1 << EN);  						// EN = 1 (������ ������ ������� � LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN); 						// EN = 0 (����� ������ ������� � LCD)

    LCD_COM_PORT &= ~(1 << RS); 						// RS = 0 (������ ������)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p << 4); 	// �������� ������� ����
    LCD_COM_PORT |= (1 << EN); 							// EN = 1 (������ ������ ������� � LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN); 						// EN = 0 (����� ������ ������� � LCD)
    _delay_us(50);
}

// ������� ������ ������ � ���, ������� ������� �� lcd
void lcd_dat(unsigned char p)
{
    LCD_COM_PORT |= (1 << RS);							// RS = 1 (������ ������)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p & 0xF0);	// �������� ������� ����
    LCD_COM_PORT |= (1 << EN);							// EN = 1 (������ ������ ������� � LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN);							// EN = 0 (����� ������ ������� � LCD)

    LCD_COM_PORT |= (1 << RS);							// RS = 1 (������ ������)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p << 4);		// �������� ������� ����
    LCD_COM_PORT |= (1 << EN);							// EN = 1 (������ ������ ������� � LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN);							// EN = 0 (����� ������ ������� � LCD)
    _delay_us(50);
}

//������� ������������� � ������� �������.
static const unsigned char convert_HD44780[64]PROGMEM =
        {
                0x41,0xA0,0x42,0xA1,0xE0,0x45,0xA3,0xA4,
                0xA5,0xA6,0x4B,0xA7,0x4D,0x48,0x4F,0xA8,
                0x50,0x43,0x54,0xA9,0xAA,0x58,0xE1,0xAB,
                0xAC,0xE2,0xAD,0xAE,0xAD,0xAF,0xB0,0xB1,
                0x61,0xB2,0xB3,0xB4,0xE3,0x65,0xB6,0xB7,
                0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0x6F,0xBE,
                0x70,0x63,0xBF,0x79,0xE4,0x78,0xE5,0xC0,
                0xC1,0xE6,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7
        };

// ������� ������������� ��������
static unsigned char lcd_rus(uint8_t c)
{
    if  (c > 191)
    {
        c -= 192;
        c = pgm_read_byte (&(convert_HD44780[c]));
    }
    return c;
}

// ������� ������ ������ �� ���
void send_string_to_LCD (const unsigned char *s)
{
    while(pgm_read_byte (s))
    {
        lcd_dat(lcd_rus(pgm_read_byte(s++)));
        _delay_ms (1);
    }
}

// ������� ������ ������� �������� �� ��� (� ������� ������ ���� ������ "����� ������" 0x00)
void send_arr_to_LCD (unsigned char *s)
{
    while(*s)
    {
        lcd_dat(lcd_rus (*s++));
        _delay_ms (1);
    }
}

// ������� ������������� ���
void lcd_init(void)
{
    // ����������� ����������������� ������
    lcd_com(0x28);		// 4 ��� ����� 2 ������, 5�8
    lcd_com(0x08);		// ���������� ������, ���������� ����������� �������, ������ �� ������ 0000 1000(bin)
    lcd_com(0x06);		// �����������������, ����� ����� ������ �������� 0000 0110(bin)
    lcd_com(0x01);		// ������� �������
    _delay_us(3000);	// ����� ���������� ������� �� ����� 1.5ms
    lcd_com(0x0C);		// ��������� ������ 0000 1100(bin)
}

void send_string_to_LCD_XY(const uint8_t *s, uint8_t x, uint8_t y)
{
    switch(y)
    {
        case 0: lcd_com(0x80 + x);
            break;
        case 1: lcd_com(0xC0 + x);
            break;
        default: lcd_com(0x80 + x);
    }
    send_string_to_LCD (s);
}

void send_arr_to_LCD_XY (uint8_t *s, uint8_t x, uint8_t y)
{
    switch(y)
    {
        case 0: lcd_com(0x80 + x);
            break;
        case 1: lcd_com(0xC0 + x);
            break;
        default: lcd_com(0x80 + x);
    }
    send_arr_to_LCD (s);
}

void lcd_dat_XY(uint8_t p, uint8_t x, uint8_t y)
{
    switch(y)
    {
        case 0: lcd_com(0x80 + x);
            break;
        case 1: lcd_com(0xC0 + x);
            break;
        default: lcd_com(0x80 + x);
    }
    lcd_dat(lcd_rus (p));
}

void lcd_clr(void)
{
    lcd_com(0x01);
    _delay_us(1500);
}

void str_clr(uint8_t place)	//������� ������ ����� ���� ��������
{
    switch(place)
    {
        case 0: lcd_com(0x80);	//������ ������
            break;
        case 1: lcd_com(0xC0);	//������ ������
            break;
        default: lcd_com(0x80);
    }
    for (uint8_t i=0; i<16; i++)
    {lcd_dat(0x20);}			//�������� �������
}

// ������� ������� ����� �� LCD, ���������� �� Frame
void Display (uint8_t line_qty)
{
    send_arr_to_LCD_XY (line_up.name, 0, 0);		// ������� ��� ���������� � 0-� ������� 1 ������ ������
    lcd_dat_XY(line_up.sign, 11, 0);				// ������� ����������� � 11-� (�� 0) ������� 1 ������ ������
    if (line_up.dig_1 != '0')						// ������� ����, ����� ������ ����� ���������� ������ �� �����
        lcd_dat(line_up.dig_1);
    lcd_dat(line_up.dig_2);
    lcd_dat('.');
    lcd_dat(line_up.dig_3);

    if (line_qty)									// ������� ������ ������, ���� ��� ����
    {
        send_arr_to_LCD_XY (line_dn.name, 0, 1);	// ������� ��� ���������� � 0-� ������� 2 ������ ������
        lcd_dat_XY(line_dn.sign, 11, 1);			// ������� ����������� � 11-� (�� 0) ������� 2 ������ ������
        if (line_dn.dig_1 != '0')					// ������� ����, ����� ������ ����� ���������� ������ �� �����
            lcd_dat(line_dn.dig_1);
        lcd_dat(line_dn.dig_2);
        lcd_dat('.');
        lcd_dat(line_dn.dig_3);
    }
}

// ������� ���������� ����� (���������� �� main), ��������� ������ ����� ��� ��������� �� LCD
void Frame (uint8_t Dig1, uint8_t Dig2, uint8_t Dig3, uint8_t sign, uint8_t i, uint8_t n)
//���������� ������ LCD: i ��� � ����-�: 0,2,4... �������, 1,3,5... ������
{
    uint8_t j=0;
    if (!(i & 0x01))
    {
        while (j < N_NAME)
        {
            line_up.name[j] = buffer.name[j];
            j++;
        }
        line_up.dig_1 = Dig1;
        line_up.dig_2 = Dig2;
        line_up.dig_3 = Dig3;
        line_up.sign = sign;
    }
    else
    {
        while (j < N_NAME)
        {
            line_dn.name[j] = buffer.name[j];
            j++;
        }
        line_dn.dig_1 = Dig1;
        line_dn.dig_2 = Dig2;
        line_dn.dig_3 = Dig3;
        line_dn.sign = sign;
    }
}

// ������� ������������� ��������
unsigned char init_device(void)
{
    unsigned char OK_Flag = 0;			// ���������� ���������� ������ ����������� ��������
    // unsigned char Flag = 0;
    cli();
    OW_PORT &= ~_BV(OW_PIN_NUM);		// � ����� ������ 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);		// 1 - ���� �� �����
    _delay_us(480);						// �������� 480 ���
    //���������� ������
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);	// 0 - ���� �� ����
    _delay_us(70);						// �������� 70 ��� ,����� ���� ������ ����� ��������
    if (!(OW_PIN & bit_msk))			// ��������� ���� ������ ��������, ���� PINB==0 - ����� ����
        OK_Flag = 1;					// ���� ����� ����
    else
        OK_Flag = 0;					// ���� ������ ���
    _delay_us(410);					// ��������� �������� 410 ��� ��������� �������� �����������
    sei();								// ��������� ����������
    return OK_Flag;
}
// ������� �������� 1 � �����
void send_1 (void)
{
    cli();								// �������� ����� ����������
    OW_PORT &= ~_BV(OW_PIN_NUM);		// � ����� ������ 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);		// 1 - ���� �� �����
    _delay_us(6);						// �������� 15 ���
    //����������
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);	// 0 - ���� �� ����
    _delay_us(64);						// �������� 45 ��� ,����� ���� ������
    sei();								// ��������� ����������
}
// ������� �������� 0 � �����
void send_0(void)
{
    cli(); 								// �������� ����� ����������
    OW_PORT &= ~_BV(OW_PIN_NUM);		// � ����� ������ 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);		// 1 - ���� �� �����
    _delay_us(60);						// �������� 120 ���
    //����������
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);	// 0 - ���� �� ����
    _delay_us(10);						// �������� 1 ��� ,����� ������� ���������� ����
    sei();								// ��������� ����������
}

//������� ������ ������ �� �������
void send_command (unsigned char command)
{
    unsigned char i;
    for (i=0; i < 8; i++)
    {
        if (command & 0x01)	// ���� ������� ���� 1, �� �������� 1
        {
            send_1 ();
        }
        else				//�������� 0
        {

            send_0 ();
        }
        command >>= 1;		//�������� ������ ��� ��������� ���������� ����
    }
}

//������� ������ �� ��������
unsigned char read_data(void)
{
    unsigned char bit;
    cli();							// �������� ����� ����������
    OW_PORT &= ~_BV(OW_PIN_NUM);	// � ����� ������ 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);	// 1 - ���� �� �����
    _delay_us(6);					// �������� 2 ���
    //����������, ���������� ���������� �������
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);// 0 - ���� �� ����
    _delay_us(9);					// �������� 9 ��� ,����� �����������
    bit = (OW_PIN & bit_msk);
    _delay_us(55);					// �������� 50 ��� ,����� ������� ���������� ����
    sei();							// ��������� ����������
    return bit;
}

// �������� CRC8
int8_t CRC_check(uint8_t *data, uint8_t crcbitN)
{
    uint8_t j;						// ������� ������ � ������� data, ������ ���������
    uint8_t crc8 = 0;				// ���������� ��� ����������� ���������� ������� CRC8 �� ������� (crcbitN-1) ������ � CRC8 �� crcbitN �����
    uint8_t data_crc;				// ���� � ������ ����� ������� ����� ������ ��������� ���� �� ����������� �� main �������
    uint8_t u;						// ������� ����� ��� ������� � ��������� ������� CRC8
    for(j = 0; j < crcbitN; j++)
    {
        unsigned char bit_crc;		// ��������� ����������
        data_crc = data[j];
        for (u = 0 ; u < 8; u++)
        {
            bit_crc = ((crc8 ^ data_crc) & 0x01);
            if (bit_crc == 0)
                crc8 >>= 1;
            else
            {
                crc8 ^= 0x18;		//  11000 , �� ������ �.�. ��� ��� 0 � 1 ����� 1
                crc8 >>= 1;			// ������� �����
                crc8 |= 0x80;		// + 1000 0000
            }
            data_crc >>=1;
        }
    }
    if (crc8 == data[j])			// ���� ��������� ���� � CRC ����� - ������
        return 0;
    else
        return 1;
}

// ������� ���������� ���������� ID-���� ��� ������ ������� ��������
void find_ID (uint8_t* data, uint8_t* New_conflict, uint8_t* last_conflict)
{
    uint8_t p = 1; 									// ���������� ��� ����� ����������� ����
    uint8_t bit = 0x01;								// ��������� ������� ���� � �����
    uint8_t bit1, bit2;								// ��� ��������� ����� ���� ���� ������
    uint8_t j = 0;									// ������� ������ �����-������

    if (!init_device())								// ����� ������� ������ � ��������� ����� ��������
    {
        // ���� �-��� ������ ������ 0
        send_string_to_LCD_XY (absence, 0, 0);		// ������� "��� ��������" � 1-� ������� ����� ������� 1 ������ ������
        abort ();
    }
    send_command(0xF0);								// ������� ������
    while (p <= 64)									// ���� �� ����� ��������� ��� 64 ����
    {
        bit1 = read_data();							// ������ ����-����
        // _delay_us(2);							// ��� ���� �������� ������ �� ��������
        bit2 = read_data();							// ������ ����-����
        if (bit1 && bit2)							// ���������� ���������� ���� , ���� ��� �������
        {
            send_string_to_LCD_XY (no_answer, 0 ,0);// ������� "��� ������ ����." � 1-� ������� ����� ������� 1 ������ ������
            abort ();
        }
        else if ((bit1) && (!bit2))					// ��� = 1
            data[j] |= bit;							// ���������� ���
        else if((!bit1) && (bit2))					// ��� = 0
            data[j] &= ~bit;
        else if((!bit1) && (!bit2))					// �������� ��� 0
        {
            //����� ����� ���������� ������� �����,  � ������� ������� ��������� ���������  � ���������� N_conflict
            if (p == *last_conflict)				// ���� ������� ������� ���� � ������ ��������� �������� ==  ������� � ���������� ���������� (������ ����� 0-�� �������), �� ������� � ������ 1
                data[j] |= bit;
            else if (p > *last_conflict)			// ���� ����� ������� ������, ������ ����������� ������, �� ������� 0 � ����� ������� ��������� �������
            {
                data[j] &= ~bit;
                *New_conflict = p;
            }
                //���� ����� ������� ����� ������� ������, ������ ������� ����������� ����������(���� ����� ������� ���, ��� ���������, ��� ��������� ���������� �� ����� ��� �� ������
                //��������� ����������� ����������,  �������� 0 , �� ��� ����� ��������� ����� ����������)
            else if (!(data[j] & bit))
            {
                *New_conflict = p;
                data[j] &= ~bit;
            }
            else
            {data[j] |= bit;}
        }

        // ����� ������� ��������������� ���, ������� ��� ��������� ���������� ������� ��������������� ����������
        if(data[j] & bit)
        {
            send_1 ();
        }
        else			// �������� 0
        {
            send_0 ();
        }

        p++;			// ����������� �� 1
        bit <<= 1;		// �������� �����

        if (!bit)		// ����� �������� ��� 8 ��� � �������� ����� 0
        {
            j++;
            _delay_ms(100);
            bit = 0x01;
        }

    }//������� �� ����� ����� ��������� 64-� �����
    *last_conflict = *New_conflict;
}
// ������� ��������� ����� ����� � ���� �������� ��� ���� ���� �������� ������� �������� � ����������
// ��������� ����� � ��������� �� ������ ��� ����� �������� ���� ����� �����
// � ��������� ������� ������� ������ ����������� 0, ������ ���� �������� ���� �� ��������� � ������
// ���������� ��������� �� ������ �������� ����� �������������� �����
uint8_t* utoa_fast_div(uint16_t value, uint8_t *buf)
{
    typedef struct
    {
        uint32_t quot;
        uint8_t rem;
    } divmod10_t;

    divmod10_t res;
    buf += N_DIGS-1;
    *buf = 0;
    do
    {
        // �������� �� 0.8
        res.quot = value >> 1;
        res.quot += res.quot >> 1;
        res.quot += res.quot >> 4;
        res.quot += res.quot >> 8;
        res.quot += res.quot >> 16;
        uint32_t qq = res.quot;
        res.quot >>= 3;													// ����� �� 8
        res.rem = (uint8_t) (value - ((res.quot << 1) + (qq & ~7ul)));	// ��������� �������
        if(res.rem > 9)													// ������������ ������� � �������, ���� ������� >=10
        {
            res.rem -= 10;
            res.quot++;
        }
        *--buf = res.rem + '0';
        value = res.quot;
    }
    while (value != 0);
    return buf;
}

// ������� ��������� ������� ����� � ����� � ����� � ������ �����
int8_t atoi_fast (uint8_t* s)
{ 	uint8_t j=0, n=0;
    if ((s[0] == '-')||(s[0] == '+'))
        j++;
    for (; s[j] >= '0' && s[j] <= '9'; j++)
        n = (n << 3) + (n << 1) + (s[j] - '0');
    if (s[0] == '-')
    {
        n = ~(n - 1);
        return n;
    }
    else
        return n;
}
// ������� ������ ���������, ������ � ������
void search_ID(void)
{
    unsigned char i, j = 0;				// ���������� ��������
    unsigned char n = 0;				// ���������� ��������, ���������� � �����
    unsigned char m = 0;				// ���������� ��������, ��������� �������� CRC8
    unsigned char data [8];				// �����-������ ��� �������� ����� ��������
    unsigned char New_conflict = 0;		// ���������� ��� ����� ������� ���� ���������
    unsigned char last_conflict = 0;	// ���������� ��� ������ ������� ���� ���������
    uint8_t digits[N_DIGS];				// ������ ��� �������� � utoa_fast_div ��� ���������� ��� ������ �������� ����

    for (j = 0; j < 8; j++)				// ������� �����-������
    {
        data[j] = 0x00;
    }
    j = 0;								// �������� ������� �����-�������

    do
    {
        New_conflict = 0;
        n++;
        if (n > n_max)									// ���� ����� ��������� ��������� 50, �� ������ � �����
        {
            send_string_to_LCD_XY (dev_excess, 0, 0);	// ������� "����� ��������" � 1-� ������� ����� ������� 1 ������ ������
            abort ();
        }

        find_ID (data, &New_conflict, &last_conflict);	// ������ �� ��������� ������ ��� ���������� ���������� ����������

        location = dev_name;							// �������� ������� ���������� ������������� ASCII ��� ����� �� ���������
        // utoa_fast_div((n-1), &dev_name[(sizeof dev_name)- N_DIGS]);// � ��� �� ��������� ����������� ASCII ��� ������ ���������� �� ������� ������, �� 0.
        for (j = 0; j < N_DIGS; j++)					// �������� ������ �������� ������ '0'
        {
            digits[j] = '0';
        }
        utoa_fast_div((n-1), digits);					// ��������� ����� ���������� � �������
        for (j = 1; j < 4; j++)							// ���������� 3 ��������� �������� digits � ��������� 3 �������� ������� ����� - ��� ����� ����������
        {
            dev_name[N_NAME-j] = digits[N_DIGS-j];
        }
        strncpy((void*)buffer.name, (void*)location, sizeof buffer.name); // ���������� ASCII ��� ����� � ���� name ������
        /*���������� ������������ ����� ���������� � void*, strncpy �� ��� ������, ����� char*, � � ��� unsigned char* */
        for (j=0; j < 8; j++)							// ������ ID-���� 1-�� 1W ���������� � ID_string � ����������� ��� � �����-���������
        {
            buffer.code[j] = data[j];
        }
        /* ��������� ���� tmax, tmin � flags ������ ���������� */
        buffer.tmax = 30;
        buffer.tmin = 6;
        buffer.flags.active = 1;	// ������ �������
        buffer.flags.lt_alarm = 0;
        buffer.flags.ht_alarm = 0;
        buffer.flags.line_alarm =0;
        buffer.flags.sms_T_0 = 0;
        buffer.flags.sms_T_1 = 0;
        buffer.flags.sms_T_2 = 0;
        buffer.flags.relay =0;
        cli();
        eeprom_update_block (&buffer, &ee_arr[n-1], sizeof(buffer)); // ���������� � ����� �������� �������� 1W ����������.
        sei();
    }
    while (last_conflict != 0);								// ���� ����� ���� ��������� �� ����� 0, ���� ����� �� ��� ������� �������

    lcd_clr(); 												// ������� �������
    send_string_to_LCD_XY (present_n, 0, 0);				// ������� "������� ����. "
    send_arr_to_LCD_XY (utoa_fast_div (n, digits), 14, 0);	// ������� ���������� ����-�, utoa_fast_div ����� ��������� �� ������ ��������� ������ ������� digits
    _delay_ms(2500);

    //������� �������, ������ ��������, ���������� ��������� ������������ ���������� ����������
    i = 0;			// �������, ������ �������� � 0-� ��������
    j = 0;
    m = n;			// ��������� �������� �����. �������� ��� �������� ��������� �������� CRC
    while(i != n)	// ������������ ���������� ���������(����� n �� ����� ������), �������� � ������� ����������
    {
        eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer));// ��������� �������� ��������� �� �����
        if (CRC_check (buffer.code, 0x07))						// ������� ��������� �� ������ � ID, ����� ����� CRC8; ���� �������� 1, CRC �� ��
        {
            m--;
            buffer.flags.active = 0;							// ����� ����� �����������
            send_string_to_LCD_XY (error, 0, 0);				// ������� "������ CRC-ID" � 1-� ������� ����� ������� 1 ������ ������
            send_arr_to_LCD_XY (buffer.name, 0, 1);
            _delay_ms(2000);
        }
        i++;
    }

    lcd_clr();												// ������� �������
    send_string_to_LCD_XY (init_n, 0 ,0);					// ������� "�������.����. " � 1-� ������� ����� ������� 1 ������ ������
    send_arr_to_LCD_XY (utoa_fast_div (m, digits), 14, 0);	// ������� ���������� ����-�, utoa_fast_div ����� ��������� �� ������ ��������� ������ ������� digits
    _delay_ms(2500);
    lcd_clr();												// ������� �������
    cli();
    eeprom_update_byte ((uint8_t*)dev_qty, n);
    eeprom_update_byte ((uint8_t*)dev_last_n, (n-1));
    sei();
}

// ������� ������ �������� �������
uint8_t scratchpad_rd (void)
{
    uint8_t j = 0;							// ������� ���� ��������
    uint8_t p;								// ������� 72 ����� ��� ����� ����������  ��������
    uint8_t bit = 0x01;						// ��������� ������� ���� ��� ������������ ����� ��������
    uint8_t bit_reg;						// ���� ������ ��������� ������ ���������� ����

    for (j = 0; j < 9; j++) 				// ������� ������
    {
        scratchpad[j] = 0x00;
    }
    j = 0;									// �������� �������
    init_device();							// ����� ������� ������ � ��������� ����� ��������
    send_command(0x55);						// ������� ������������

    for (j = 0; j < 8 ; j++)				// ������� ��� ���������� � �������� ����������
    {
        send_command (buffer.code[j]);
    }
    j = 0;									// �������� �������
    send_command(0xBE);						// ������ �������
    for (p = 1; p <= 72; p++)				// ���� �� ����� ��������� ��� 72 ����
    {
        bit_reg = read_data(); 				// ������ ���
        if (bit_reg)
            scratchpad[j] |= bit;			// ���������� ��� =1
        else
            scratchpad[j] &= ~bit;			// ���������� ��� = 0
        bit <<= 1;							// �������� �����
        if (!bit)							// ����� �������� ��� 8 ��� � �������� ����� 0
        {
            j++;
            _delay_ms(100);
            bit = 0x01;
        }
    }										// ������� �� ����� ����� ��������� 72-� �����
    init_device();
    if (scratchpad[4] == 0xFF)				// ������ �� �������, ��� ���� ����.����� == 1
        return 1;
    else if (CRC_check(scratchpad, 0x08))	// ���� CRC �� OK
        return 2;
    else
        return 0;
}

//������������ ������, ����������� ������������ �������, ���������� �� ISR(TIMER0_OVF_vect)
void BTN_SCAN(void)
{
    if (!(SRC_PIN & src_msk))										// ���� ������
    {
        if(btn_cnt < CNT_QUICK)										// ���� ������ ������ ������ ��������� �������
        {
            btn_cnt++;												// ������ ������ �������, ������ �� ��������
        }
        else if ((btn_cnt >= CNT_QUICK) && (btn_cnt < CNT_SLOW))	// ���� ������ ������ ������ ���������, �� �� ���������� �� �������� ����
        {
            if (!btn_state)											// ���� �������� ���� � ������ ���,�.�. ��� �� ���� ������������� �������
            {
                btn_state = PRESSED;								// ��������� ���� �������
                btn_time = QUICK;									// ��������� ���� ��� ������� ��������� �������
                btn_cnt++;											// ������ ������� ������
            }
            else													// ���� ��� ���� ������������� �������
            {
                btn_cnt++;											// ������ ������ ������� ������
            }
        }
        else if (btn_cnt == CNT_SLOW)								// ���� ������ ������ ������ �������� ����
        {
            btn_cnt = CNT_SLOW + 10;								// ����� � ��������� ���, ���� ������ ��� � �� ���������, ���� �� �����
            btn_time = SLOW;										// ��������� ���� �������� �������
        }
    }
    else if ((SRC_PIN & src_msk)&&(btn_state == PRESSED))			// ���� ������ �� ������, � ���� �������� ����, ������ ������ ���������
    {
        press_time = btn_time;										// ������� ������ ������������ ������� � �������� �� ��� ���������� �����
        btn_time = 0;
        btn_state = RELEASED;
        btn_cnt=0;
    }
}

/*					#######################		������ ����� ������� ������ � ��������� 		###########################*/

// ������� ������� ���������� ������ ��������
/*void Clear_RX_Ring(void)
{
	cli();
	RX_IndexIN = 0;
	RX_IndexOUT = 0;
	sei();
}*/

// ������� ���������� ���������� ������������� ���� � ��������� ������ ��������
uint8_t RX_IndexNumber(void)
{
    return (RX_IndexIN - RX_IndexOUT) & RX_IND_MSK;
}

// ������� �������� ����� ������ � ��������� ����� ��������
uint8_t UDR_to_RX_Ring(char value)
{
    if (((RX_IndexIN + 1) & RX_IND_MSK) == RX_IndexOUT)	// ���� RX_Ring ������������
    {
        RX_ring[RX_IndexIN] = '\n';	// ����� � ������ \r\nOK\r\n, ����� � ������� ����� ��� ��� ��, ����� �������� ������ ������
        RX_ring[(RX_IndexIN - 1) & RX_IND_MSK] = '\r';
        RX_ring[(RX_IndexIN - 2) & RX_IND_MSK] = 'K';
        RX_ring[(RX_IndexIN - 3) & RX_IND_MSK] = 'O';
        RX_ring[(RX_IndexIN - 4) & RX_IND_MSK] = '\n';
        RX_ring[(RX_IndexIN - 5) & RX_IND_MSK] = '\r';
        return 1; 					// ������������ ������, �� ����� ����� ������
    }
    else
    {
        RX_IndexIN++;
        RX_IndexIN &= RX_IND_MSK;
        RX_ring[RX_IndexIN] = value;
        return 0;
    }
}

// ������� �������� ������ ������ �� ���������� ������ � ������ ��� ������� ������
void RX_Ring_to_Str(uint8_t *str, uint8_t lenght)
{
    for(uint8_t i=0; i<lenght; i++)
    {
        RX_IndexOUT++;
        RX_IndexOUT &= RX_IND_MSK;
        *str = RX_ring[RX_IndexOUT];
        str++;
    }
}

/*// ������� �� ����������� ������ � ��������� ������ ��������
uint8_t Get_RX_Data(void)
{
	if(RX_IndexIN != RX_IndexOUT) return 1;
	return 0;
}*/

/*					#########################		����� ����� ������� ������ � ��������� 	################*/

/*					#########################		������ ����� ������� ������ � ������������ 	################*/

// ������� ������� ���������� ������ �����������
/*void Clear_TX_Ring(void)
{
	cli();
	TX_IndexIN = 0;
	TX_IndexOUT = 0;
	sei();
}*/

// ������� �� ����������� ������ � ��������� ������ �����������
uint8_t Get_TX_Data(void)
{
    if(TX_IndexIN != TX_IndexOUT) return 1;
    return 0;
}

void USART_TXD(uint8_t data) // �������� ����� �� UART
{
    while (!( UCSR0A & (1 << UDRE0))) {;} 	// ���� ���� �� ���������� ���������� ������
    UDR0 = data;							// ��������� ������� ������
}

/* void USART_CRLF(void) //�������� CR � LF
{
	USART_TXD('\r');
	USART_TXD('\n');
}*/

/*void arr_to_USART(uint8_t *s ) // �������� ������� �� UART
{
	while(*s)
	{
		USART_TXD(*s++);
	}
}*/

/*void string_to_USART(const uint8_t *s ) // �������� ����� �� ���� �� UART
{
	while(pgm_read_byte(s))
	{
		USART_TXD(pgm_read_byte(s++));
	}
}*/

//������� �������� ����� � ��������� ����� �����������
void byte_to_TX_Ring(uint8_t byte)
{
    if (((TX_IndexIN + 1) & TX_IND_MSK) == TX_IndexOUT)
    {
        send_string_to_LCD_XY(tx_ring_ovf, 0, 0);		//###############
        //string_to_USART (tx_ring_ovf);				//###############
    }
    while (((TX_IndexIN + 1) & TX_IND_MSK) == TX_IndexOUT) {;} //���� ������ ������� �����, ���
    TX_IndexIN++;
    TX_IndexIN &= TX_IND_MSK;
    TX_ring[TX_IndexIN] = byte;

}

//������� �������� ������� � ������ �����������
void arr_to_TX_Ring(uint8_t *s)
{
    while(*s)
        byte_to_TX_Ring(*s++);
}

//������� �������� ������ �� ���� � ������ �����������
void string_to_TX_Ring(const uint8_t *s)
{
    while(pgm_read_byte(s))
        byte_to_TX_Ring(pgm_read_byte(s++));
}

/*//������� �������� �������� CR � LF � ������ �����������
void CRLF_to_TX_Ring(void)
{
	byte_to_TX_Ring('\r');
	byte_to_TX_Ring('\n');
}*/

/*			################################			����� ����� ������� ������ � ������������ 			##################*/


void USART_Init( unsigned int ubrr)
{
    UBRR0H = (uint8_t)(ubrr >> 8);						//�������� 9600
    UBRR0L = (uint8_t)ubrr;								//�������� 9600
    UCSR0A = 0;											//�������� �����
    UCSR0B = (1 << RXEN0)|(1 << TXEN0)|(1 << RXCIE0);	//�������� ��������, ���� � ���������� �� �����
    UCSR0C = (1 << UCSZ01)|(1 << UCSZ00)|(1 << USBS0);	//8 ���, 1 ���� ���
}

void msg_clr(void)	//������� msg
{
    for (uint8_t j=0; j < MSG_SIZE; j++) {msg[j] = 0;}
}

void NRESET(void) //����� ������
{
    cli();
    TX_IndexIN = TX_IndexOUT = RX_IndexIN = RX_IndexOUT = 0;//������� ������ ����� � ��������
    modem_rdy = 0;						//����� �� �����
    gsm_lvl = 0;						//�������� ������ GSM ������������� ��� ��������� ���������� ������
    RESET.flag_1 = 0;					//����� � ������ ���� �������������
    ans_lim = PWR_UP_ANS_TIMER;			//������ �������� ������ 15�
    msg_clr();							//�������� msg
    mod_ans = 0;						//�������� �����, ����� �� ������� ��� �������� ��� ����������� ����������

    /*���� NRESET ������ �� 0 ����� ������� ����, �� ���: (� ����������������� ������ � main) */
    //RST_PORT |= _BV(RST_PIN_NUM);		//�������� ���� ������
    //_delay_us(50);
    //RST_PORT &= ~_BV(RST_PIN_NUM);	//��������� ���� ������

    /*���� NRESET ������ �� 0 ����� ������ ����, �� ���:  (� ���������������� ������ � main) */
    RST_PORT &= ~_BV(RST_PIN_NUM);		//������ 0 �� ������ ��� ������ ������
    DDR_RST_PORT |= _BV(RST_PIN_NUM);	//����� ��� ������ ������ ����������� �� ����� - �������� NRESET
    _delay_us(50);
    DDR_RST_PORT &= ~_BV(RST_PIN_NUM);	//����� ��� ������ ������ � 3-� ��������� - ��������� NRESET

    ans_cnt = 0;						//��������� ������ �������� ������
    sei();
}

uint8_t send_cmd (void)	//HANDLER �������� �������
{
    if (!WR_CMD[cmd_task_T].step.flag_1)			//���� ����� �������� ����
    {
        string_to_TX_Ring (WR_CMD[cmd_task_T].cmd);
        if (WR_CMD[cmd_task_T].pgm_par_1 != NULL)							//���� ��������_1 ������� �� ���� �� ������
        {
            string_to_TX_Ring (WR_CMD[cmd_task_T].pgm_par_1);				//��������� ����� ������� �� ����
        }
        if (WR_CMD[cmd_task_T].ram_par != NULL)								//���� �������� ������� �� ��� �� ������
        {
            arr_to_TX_Ring (WR_CMD[cmd_task_T].ram_par);					//��������� ����� ������� �� ���
        }
        if (WR_CMD[cmd_task_T].pgm_par_2 != NULL)							//���� ��������_2 ������� �� ���� �� ������
        {
            string_to_TX_Ring (WR_CMD[cmd_task_T].pgm_par_2);				//��������� ����� ������� �� ����
        }
        if (WR_CMD[cmd_task_T].pgm_par_3 != NULL)							//���� ��������_3 ������� �� ���� �� ������
        {
            string_to_TX_Ring (WR_CMD[cmd_task_T].pgm_par_3);				//��������� ����� ������� �� ����
        }
        if (WR_CMD[cmd_task_T].pgm_par_4 != NULL)							//���� ��������_4 ������� �� ���� �� ������
        {
            string_to_TX_Ring (WR_CMD[cmd_task_T].pgm_par_4);				//��������� ����� ������� �� ����
        }
        string_to_TX_Ring (CRLF);
        ans_lim = 180;								//��������� ������� �������� ������
        UCSR0B |= (1<<UDRIE0);						//���������� ���������� �� ����������� UDR �����������
        ans_cnt = 0;								//��������� ������ ������
        WR_CMD[cmd_task_T].step.flag_1 = 1;			//����� ������� ��������� � �����
        return 'S';
    }
    else if (WR_CMD[cmd_task_T].step.flag_1)
    {
        if (mod_ans == OK)	//���� � msg ���� ��
        {
            cmd_task_T = (cmd_task_T + 1) & CMD_TASK_IND_MSK;	//������� �� ������ ������
            queue_T = (queue_T + 1) & QUEUE_IND_MSK;			//������� ����������� ������ �� �������
            mod_ans = 0;										//�������� �����, ����� �� ������� ��� �������� ��� ����������� ����������
            return 'F';
        }
        else if (ans_cnt >= ans_lim)
        {
            WR_CMD[cmd_task_T].step.flag_1 = 0;	//����� ����� "����� ������� ��������� � �����"
            NRESET();							//����� ������
            return 'G';
        }
        else return 'H';
    }
    else return 'Z';
}

uint8_t send_sms (void)	//HANDLER �������� ���
{
    uint8_t name[N_NAME];			// ��������� ������ ����� ����������
    static uint8_t k = 0;			// ������� ������ ������� ��� ��� �������� �������, static �.�. ����� ���������� ����� ������� �� ������ UDRIE0
    static uint8_t n = 0;			// ���������� ���������� ���������, static ��� ������ �������� ������� ��� �������
    static uint8_t sms_type = 0;	// ���������� ��� ���� ���, static ��� ������ �������� ������� ��� �������
    int8_t t = 0;					// ���������� ����������� �� ������� ���������
    uint8_t digits[N_DIGS];			// ��������� ������ ���� ����������

    if (!WR_SMS[out_task_T].step.flag_1 && !WR_SMS[out_task_T].step.flag_2)	//���� ������ ���� � ������
    {
        if ((phones.phone_0[0] = '+')&&(phones.phone_0[1] = '7')&&(phones.phone_0[2] = '9'))//��������� ������� �������� ������ � ������� (+79...)
        {
            string_to_TX_Ring (AT_CMGS);
            switch (sms_buff.person)
            {
                case ADMIN:
                    arr_to_TX_Ring (phones.phone_0);
                    break;
                case ABNT_1:
                    arr_to_TX_Ring (phones.phone_1);
                    break;
                case ABNT_2:
                    arr_to_TX_Ring (phones.phone_2);
                    break;
            }
            string_to_TX_Ring (QUOTES);
            string_to_TX_Ring (CRLF);
            ans_lim = 180;									// ������ �������� ������ 3�
            UCSR0B |= (1<<UDRIE0);							// ���������� ���������� �� ����������� UDR �����������
            ans_cnt = 0;									// ��������� ������ �������� ������ ">"
            WR_SMS[out_task_T].step.flag_1 = 1;				// ������ �� �������� ��� ���������
            sms_type = WR_SMS[out_task_T].sms_txt.sms_type;	// ��������� ��� ��� ��� ����������� ����������
            if (sms_type == ALL)
            {
                n = eeprom_read_byte((uint8_t*)dev_qty);	// ��������� ���������� ��������� ��� ������� ���
            }
            return 'A';
        }
        else
        {
            out_task_T = (out_task_T + 1) & OUT_TASK_IND_MSK;	// ������� ���������� ������ �� ������ ����� �� �������� ���
            queue_T = (queue_T + 1) & QUEUE_IND_MSK;			// ������� ���������� ������ �� �������
            return 'W';
        }
    }
    else if (WR_SMS[out_task_T].step.flag_1)			// ���� ������ �� �������� ��� ��� ��������� � �����
    {
        if (mod_ans == INVITE)	//���� � msg ���� ">"
        {
            switch (sms_type)
            {
                case FAIL:
                    k = WR_SMS[out_task_T].sms_txt.dev_num;						// �������� ����� ����������
                    eeprom_read_block (name, &ee_arr[k].name, sizeof name);		// ������ ��� ���������� �� ��������� ������
                    arr_to_TX_Ring (name);										// ��� � ������ ���
                    string_to_TX_Ring (blank);									// ��� � ������ ������
                    string_to_TX_Ring (crash);									// ��� � ������ FAIL!
                    break;

                case ALL:
                    if (UCSR0B & (1<<UDRIE0)) {return 0;}						// ��� ���� �� ��������� ���� ����� ����������� ���������� ����������
                    while (k < n)												// ���������� ��� �������� last_t � TX_Ring
                    {
                        eeprom_read_block (name, &ee_arr[k].name, sizeof name);	// ������ ��� ���������� �� ��������� ������
                        arr_to_TX_Ring (name);									// ��� � ������ ���
                        string_to_TX_Ring (blank);								// ��� � ������ ������
                        t = t_all[k];											// ������ ����������� �� ������� ���������
                        byte_to_TX_Ring(((t & 0b10000000) ? '-' : '+'));		// ��� � ������ ���� �����������
                        arr_to_TX_Ring(utoa_fast_div (((t & 0b10000000) ? ((~t) + 1) : t), digits)); // ��� � ������ ����� ����-��
                        string_to_TX_Ring (NEW_LN);								// ��� � ������ ������ ����� ������
                        UCSR0B |= (1<<UDRIE0);									// ���������� ���������� �� ����������� UDR �����������
                        k++;
                        return 0;
                    }
                    string_to_TX_Ring (gsm);									// ���  � ������ ������ GSM
                    string_to_TX_Ring (blank);									// ��� � ������ ������
                    arr_to_TX_Ring(utoa_fast_div (pgm_read_byte (&(gsm_scale[gsm_sig])), digits)); // ��� � ������ ������� �������
                    UCSR0B |= (1<<UDRIE0);										// ���������� ���������� �� ����������� UDR �����������
                    break;

                case REN_DONE:
                    string_to_TX_Ring (rename_ok);
                    break;

                case NAME_ERR:
                    string_to_TX_Ring (name_error);
                    break;

                case MIN_LIM_SET:
                    k = WR_SMS[out_task_T].sms_txt.dev_num;						// �������� ����� ����������
                    eeprom_read_block (name, &ee_arr[k].name, sizeof name);		// ������ ��� ���������� �� ��������� ������
                    t = WR_SMS[out_task_T].sms_txt.param;						// �������� �������� ����������� �� ������
                    arr_to_TX_Ring (name);
                    string_to_TX_Ring (blank);
                    string_to_TX_Ring (t_min);
                    string_to_TX_Ring (blank);
                    byte_to_TX_Ring ((t & 0b10000000) ? '-' : '+');
                    arr_to_TX_Ring (utoa_fast_div (((t & 0b10000000) ? ((~t) + 1) : t), digits));
                    break;

                case MAX_LIM_SET:
                    k = WR_SMS[out_task_T].sms_txt.dev_num;						// �������� ����� ����������
                    eeprom_read_block (name, &ee_arr[k].name, sizeof name);		// ������ ��� ���������� �� ��������� ������
                    t = WR_SMS[out_task_T].sms_txt.param;						// �������� �������� ����������� �� ������
                    arr_to_TX_Ring (name);
                    string_to_TX_Ring (blank);
                    string_to_TX_Ring (t_max);
                    string_to_TX_Ring (blank);
                    byte_to_TX_Ring ((t & 0b10000000) ? '-' : '+');
                    arr_to_TX_Ring (utoa_fast_div (((t & 0b10000000) ? ((~t) + 1) : t), digits));
                    break;

                case LIM_ERR:
                    string_to_TX_Ring (t_error);
                    break;

                case SMS_ON:
                    k = WR_SMS[out_task_T].sms_txt.dev_num;						// �������� ����� ����������
                    eeprom_read_block (name, &ee_arr[k].name, sizeof name);		// ������ ��� ���������� �� ��������� ������
                    arr_to_TX_Ring (name);
                    string_to_TX_Ring (blank);
                    string_to_TX_Ring (sms_send);
                    string_to_TX_Ring (blank);
                    string_to_TX_Ring (on);
                    break;

                case SMS_OFF:
                    k = WR_SMS[out_task_T].sms_txt.dev_num;						// �������� ����� ����������
                    eeprom_read_block (name, &ee_arr[k].name, sizeof name);		// ������ ��� ���������� �� ��������� ������
                    arr_to_TX_Ring (name);
                    string_to_TX_Ring (blank);
                    string_to_TX_Ring (sms_send);
                    string_to_TX_Ring (blank);
                    string_to_TX_Ring (off);
                    break;

                case T_LOW:
                    k = WR_SMS[out_task_T].sms_txt.dev_num;						// �������� ����� ����������
                    eeprom_read_block (name, &ee_arr[k].name, sizeof name);		// ������ ��� ���������� �� ��������� ������
                    t = WR_SMS[out_task_T].sms_txt.param;						// �������� �������� ����������� �� ������
                    arr_to_TX_Ring (name);
                    string_to_TX_Ring (blank);
                    string_to_TX_Ring (t_low);
                    byte_to_TX_Ring (((t & 0b10000000) ? '-' : '+'));
                    arr_to_TX_Ring (utoa_fast_div (((t & 0b10000000) ? ((~t) + 1) : t), digits));
                    break;

                case T_HIGH:
                    k = WR_SMS[out_task_T].sms_txt.dev_num;						// �������� ����� ����������
                    eeprom_read_block (name, &ee_arr[k].name, sizeof name);		// ������ ��� ���������� �� ��������� ������
                    t = WR_SMS[out_task_T].sms_txt.param;						// �������� �������� ����������� �� ������
                    arr_to_TX_Ring (name);
                    string_to_TX_Ring (blank);
                    string_to_TX_Ring (t_high);
                    byte_to_TX_Ring (((t & 0b10000000) ? '-' : '+'));
                    arr_to_TX_Ring (utoa_fast_div (((t & 0b10000000) ? ((~t) + 1) : t), digits));
                    break;

                case MONEY:
                    string_to_TX_Ring (BALANCE);
                    string_to_TX_Ring (blank);
                    arr_to_TX_Ring (WR_SMS[out_task_T].sms_txt.ptr);			// ���������� ����� �������
                    break;

                case BAL_TEL:
                    string_to_TX_Ring (BALANCE);
                    break;

                case COM_ERR:
                    string_to_TX_Ring (com_error);
                    break;

                case ADMIN:
                    string_to_TX_Ring (USER_0);
                    string_to_TX_Ring (NEW_LN);
                    arr_to_TX_Ring (phones.phone_0);
                    break;

                case MEMBERS:								// �������� �� ����� ������� ���
                    if (UCSR0B & (1<<UDRIE0)) {return 0;}	// ��� ���� �� ��������� ���� ����� ����������� ���������� ����������
                    switch (k)								// ����� "�" - ������� �����, ����� �� ������� ��� ���� static ����������
                    {
                        case 0:
                            string_to_TX_Ring (USER_0);
                            string_to_TX_Ring (NEW_LN);
                            arr_to_TX_Ring (phones.phone_0);
                            string_to_TX_Ring (NEW_LN);
                            k = 1;
                            UCSR0B |= (1<<UDRIE0);			// ���������� ���������� �� ����������� UDR �����������
                            return 0;
                        case 1:
                            string_to_TX_Ring (USER_1);
                            string_to_TX_Ring (NEW_LN);
                            arr_to_TX_Ring (phones.phone_1);
                            string_to_TX_Ring (NEW_LN);
                            k = 2;
                            UCSR0B |= (1<<UDRIE0);			// ���������� ���������� �� ����������� UDR �����������
                            return 0;
                        case 2:
                            string_to_TX_Ring (USER_2);
                            string_to_TX_Ring (NEW_LN);
                            arr_to_TX_Ring (phones.phone_2);
                            string_to_TX_Ring (NEW_LN);
                            k = 2;
                            UCSR0B |= (1<<UDRIE0);			// ���������� ���������� �� ����������� UDR �����������
                            return 0;
                            break;
                        case 3:
                            string_to_TX_Ring (BALANS);
                            string_to_TX_Ring (NEW_LN);
                            arr_to_TX_Ring (phones.balance);
                            string_to_TX_Ring (NEW_LN);
                            k = 4;
                            UCSR0B |= (1<<UDRIE0);			// ���������� ���������� �� ����������� UDR �����������
                            return 0;
                        case 4:
                            break;
                    }
                    break;

                case DENY:
                    string_to_TX_Ring (DENIED);
                    break;

                default:							// ���� �� ���� ������ �� ���������
                    string_to_TX_Ring (sms_send);	// ���� ��� ����� "SMS FAIL!"
                    string_to_TX_Ring (crash);
                    break;
            }
            string_to_TX_Ring (CTRL_Z);				// ���������� \0 � ����� ������ ��� � .sms_txt!!!
            ans_lim = 600;							// ����� �������� ������ 10�
            UCSR0B |= (1<<UDRIE0);					// ���������� ���������� �� ����������� UDR �����������
            ans_cnt = 0;							// ��������� ������ ������
            WR_SMS[out_task_T].step.flag_1 = 0;		// ����� ����� "������ �� �������� ��� ��������� � �����"
            WR_SMS[out_task_T].step.flag_2 = 1;		// ������ ����� "����� ��� ��������� � �����"
            mod_ans = 0;							// �������� �����, ����� �� ������� ��� �������� ��� ����������� ����������
            k = 0;									// �������� ������ ���������� ��� ���������� ��������� ������
            return 'B';
        }
#ifdef DEBUG
            else if (ans_cnt < ans_lim)
			{return 'C';}
#endif
        else if (ans_cnt >= ans_lim)
        {
            WR_SMS[out_task_T].step.flag_1 = 0;		//����� ����� "������ �� �������� ��� ��������� � �����"
            NRESET();								//����� ������
            return 'D';
        }
        else return 'X';
    }
    else if (WR_SMS[out_task_T].step.flag_2)					//���� ����� ��� ��� ��������� � �����
    {
        if (mod_ans == OK)										//���� � msg ���� ".....OK"
        {
            out_task_T = (out_task_T + 1) & OUT_TASK_IND_MSK;	//������� ������ �� ������ ����� �� �������� ���
            queue_T = (queue_T + 1) & QUEUE_IND_MSK;			//������� ����������� ������ �� �������
            mod_ans = 0;										//�������� �����, ����� �� ������� ��� �������� ��� ����������� ����������
            return 'E';
        }
#ifdef DEBUG
            else if (ans_cnt < ans_lim)
			{return 'F';}
#endif
        else if (ans_cnt >= ans_lim)
        {
            WR_SMS[out_task_T].step.flag_2 = 0;				//����� ������ ������
            NRESET();										//����� ������
            return 'G';
        }
        else return 'Y';
    }
    else
        return 'H';
}

uint8_t read_sms (void)	//HANDLER ������ ���
{
    if (!RD_SMS[inc_task_T].step.flag_1 && !RD_SMS[inc_task_T].step.flag_2) //���� ������ ���� � ������
    {
        string_to_TX_Ring (AT_CMGR);
        arr_to_TX_Ring (RD_SMS[inc_task_T].sms_num);
        string_to_TX_Ring (CRLF);
        ans_lim = 120;						//������ ������ 2�
        UCSR0B |= (1<<UDRIE0);				//���������� ���������� �� ����������� UDR �����������
        ans_cnt = 0;						//��������� ������ �������� ������ "/r/nOK/r/n"
        RD_SMS[inc_task_T].step.flag_1 = 1;	//������ ����� "������ �� ������ ��� ��������� � �����"
        return 'A';
    }
    else if (RD_SMS[inc_task_T].step.flag_1) //���� ������ �� ������ ��� ��������� � �����
    {
        if (mod_ans == OK)	//���� � msg ���� "/r/nOK/r/n"
        {
            string_to_TX_Ring (AT_CMGD);
            arr_to_TX_Ring (RD_SMS[inc_task_T].sms_num);
            string_to_TX_Ring (CRLF);
            ans_lim = 120;							//������ �������� ������ 2�
            UCSR0B |= (1<<UDRIE0);					//���������� ���������� �� ����������� UDR �����������
            ans_cnt = 0;							//��������� ������ ������
            RD_SMS[inc_task_T].step.flag_1 = 0;		//����� ���� "������ �� ������ ��� ��������� � �����"
            RD_SMS[inc_task_T].step.flag_2 = 1;		//������ ����� "������ �� �������� ������������ ��� ��������� � �����"
            mod_ans = 0;							//�������� �����, ����� �� ������� ��� �������� ��� ����������� ����������
            return 'B';
        }
#ifdef DEBUG
            else if (ans_cnt < ans_lim)
			{return 'C';}
#endif
        else if (ans_cnt >= ans_lim)
        {
            RD_SMS[inc_task_T].step.flag_1 = 0;	//����� ����� "������ �� ������ ��� ��������� � �����"
            NRESET();							//����� ������
            return 'D';
        }
        else return 'X';
    }
    else if (RD_SMS[inc_task_T].step.flag_2) //���� ������ �� �������� ������������ ��� ��������� � �����
    {
        if (mod_ans == OK) //���� � msg ���� "/r/nOK/r/n"
        {
            inc_task_T = (inc_task_T + 1) & INC_TASK_IND_MSK;	//������� ������ �� ������ ����� �� ������ ���
            queue_T = (queue_T + 1) & QUEUE_IND_MSK;			//������� ����������� ������ �� �������
            mod_ans = 0;										//�������� �����, ����� �� ������� ��� �������� ��� ����������� ����������
            return 'E';
        }
#ifdef DEBUG
            else if (ans_cnt < ans_lim)
			{return 'F';}
#endif
        else if (ans_cnt >= ans_lim)
        {
            RD_SMS[inc_task_T].step.flag_2 = 0;	//����� ������ ������
            NRESET();							//����� ������
            return 'G';
        }
        else return 'Y';
    }
    else
        return 'H';
}

uint8_t parser(void) // ������ ������ msg
{
    char* txt_ptr;			// ��������� �� ������ �������� ������ � ������ ���
    uint8_t n = 0;			// ����� ��������� +cmti:, �� ����� ���� ���������
    uint8_t  number[3];		// ��������� ������ ��� ����������� ������ ���
    uint8_t pars_res = 'Z';	// ��������� ������ �������
    static uint8_t money[6];// ������ ��� ���� �������, static, ����� ����� ���� �������� �� ���� ���������

    if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_OK))!=NULL) 		// ���� � msg ���� r/n/OKr/n:
    {
        mod_ans = OK;
        pars_res = 'O';
    }
    else if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_ENT))!=NULL)	// ���� � msg ���� r/n>:
    {
        mod_ans = INVITE;
        pars_res = 'I';
    }

    if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_CMGR))!=NULL)		// ���� � msg ���� r/n/+cmgr:
    {
        txt_ptr += 8;														// ������� ��������� �� +CMGR:_
        uint8_t q = 0;														// ������� �������
        if (strstr((const char*)txt_ptr, (char*)phones.phone_0)!=NULL)		// ���� ������� ������
        {sms_buff.person = ADMIN;}
        else if (strstr((const char*)txt_ptr, (char*)phones.phone_1)!=NULL)	// ���� ������� �����1
        {sms_buff.person = ABNT_1;}
        else if (strstr((const char*)txt_ptr, (char*)phones.phone_2)!=NULL)	// ���� ������� �����2
        {sms_buff.person = ABNT_2;}
        if (sms_buff.person)												// ���� ����������� ��������
        {
            while ((q != 8)&&(*txt_ptr != '\r'))// ���� ��������� ����������� (�������) �������, �� ���� ����� ����� ���
            {									// ���� ���� �� \r\n ����� ������, ���� �� �������������� \r\n��\r\n � ����� msg (��� ������������ RX_Ring)
                if (*txt_ptr == '\"')										// ���� ���������� �� �������
                {q += 1;}												// ������ �������
                txt_ptr += 1;												// ��������� ������� � ����� ������
            }
            if (q == 8)														// ������ ���� ��������� 8 �������, ����� ����� ��� ���
            {
                txt_ptr += 2;
                for (uint8_t j = 0; j < TODO_MAX; j++) {todo_txt [j] = 0;}	// ������� ������� ������� �����������
                uint8_t j = 0;
                while (((*txt_ptr) != '\r') && (j < TODO_MAX))				// �������� ����� ������� ����������� �� ���
                {
                    todo_txt[j] = *(txt_ptr + j);
                    j++;
                }
                to_do(); 													// �������� ������� ������������� �������
            }
            pars_res = 'R';
        }
            // ���� ������� (�� �������) �� ������ � ������ ������, �� � ������� ������� admin �� �������, ��������� ����� �� ��� �� admin, ���������� � ������ � �� ���
        else if ((phones.phone_0[0] != '+')&&(phones.phone_0[1] != '7')&&(phones.phone_0[2] != '9'))//���������� �������� ������ ��� ��������
        {
            while ((q != 3)&&(*txt_ptr != '\r'))// ���� ������  �������, �����������, "REC (UN)READ","+79..., �� ���� ����� �������
            {									// ���� ���� �� \r\n ����� ������, ���� �� �������������� \r\n��\r\n � ����� msg (��� ������������ RX_Ring)
                if (*txt_ptr == '\"')										// ���� ���������� �� �������
                {q += 1;}												// ������ �������
                txt_ptr += 1;												// ��������� ������� � ����� ������
            }
            if (q == 3)														// ������ ���� ��������� 3 �������, ����� ����� ��� ���
            {
                for (uint8_t i = 0; i < 12; i++)
                {
                    phones.phone_0[i] = txt_ptr[i];
                }
                cmd_to_queue (AT_CPBW, CELL_2, phones.phone_0, TEXT_145, USER_0, QUOTES);	//���������� ���� ������� �� ���
                sms_buff.sms_type = ADMIN;									// ���������� ������������� �����������
                sms_buff.person = ADMIN;
                out_to_queue (&sms_buff);
            }
        }
    }

    if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_CSQ))!= NULL) // ���� � ������ ���� r/n/+csq:_
    {
        txt_ptr += 8;									// ������ ��������� �� ������ �����
        uint8_t tmp[3];									// ��������� ������ ����� �������� ���� ������ �������
        uint8_t j = 0;
        while (isdigit(*(txt_ptr + j)))					// ��������� �������� ������� ����� r/n/+csq:_ � ������
        {
            tmp[j] = *(txt_ptr + j);
            j++;
        }
        uint8_t lev = atoi_fast (tmp);					// ��������� ������� � ����� == ������ �������
        if (lev >= 0 && lev <= 31) {gsm_sig = lev;}		// ���������� ������� � ���������� ���������� ��� �������� � ���, ��� ������ ��� ������� �������
        else {gsm_sig = 0;}
        if (lev > 0 && lev < 6)        	{gsm_lvl = 2;}	// 0-25% - ���� �������
        else if (lev >= 6 && lev < 9)   {gsm_lvl = 4;}	// 25-50% - ��� �������
        else if (lev >= 9 && lev < 16)  {gsm_lvl = 6;}	// 50-75% - ��� �������
        else if (lev >= 16 && lev <= 31){gsm_lvl = 8;}	// 75-100% - ������ �������
        else {gsm_lvl = 0;}								// ��� �������, �� �������
        pars_res = 'Q';
    }

    if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_CUSD))!= NULL) 	// ���� � ������ ���� \r\n+CUSD:_
    {
        txt_ptr = txt_ptr + 11;
        while (!((isdigit(*txt_ptr))||(*txt_ptr == '-')))// ���� �������� ������� ��� '-' ����� "r/n/+CUSD:_0,"
        {
            txt_ptr++;
        }
        uint8_t j = 0;
        while (((isdigit(*(txt_ptr + j)))||(*(txt_ptr + j) == '-'))&&(j < 5))// ��������� �������� ������� � '-' (����� <= 5) � ������
        {
            money[j] = *(txt_ptr + j);
            j++;
        }
        money[j] = '\0';								// ����������� 0
        sms_buff.sms_type = MONEY;
        sms_buff.ptr = money;
        sms_buff.person = phones.ans_to;				// ��������� ����������, ������� ���������� ������
        out_to_queue (&sms_buff);
    }

    if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_COPS))!= NULL) 	// ���� � ������ ���� "\r\n+COPS: "
    {
        txt_ptr = txt_ptr + 14;
        if (strstr_P(txt_ptr, (PGM_P) MTS) != NULL)				// ���� ���
        {
            phones.balance[1] = '1';							// ��������� � ��� 0
        }
        else if (strstr_P(txt_ptr, (PGM_P) BEELINE)	!= NULL)	// ���� ������
        {
            phones.balance[1] = '1';
            phones.balance[3] = '2';							// � �������� � ��� 0
        }
        else if (strstr_P(txt_ptr, (PGM_P) MEGAFON)	!= NULL)	// ���� �������
        {
            phones.balance[0] = '1';							// ��������� � ��� 0
        }
        else if (strstr_P(txt_ptr, (PGM_P) TELE2)	!= NULL)	// ���� ����2
        {
            phones.balance[1] = '1';
            phones.balance[3] = '5';							// � �������� � ��� 0
        }
        else if ((phones.balance[1] == '0')&&(phones.balance[2] == '0')&&(phones.balance[3] == '0'))//���� � ������� ����
        {
            sms_buff.sms_type = BAL_TEL;						// ����������� ������ �������� ��� �������
            sms_buff.dev_num = 0;
            sms_buff.param = 0;
            sms_buff.ptr = NULL;
            sms_buff.person = ADMIN;
            out_to_queue(&sms_buff);
        }
    }

    if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_CPBF))!= NULL) 	// ���� � ������ ������ ���� "\r\n+CPBF: "
    {
        txt_ptr = txt_ptr + 12;												// ������ ��������� �� ������ ���� �������� � ������ ������
        if ((strstr_P((const char*)txt_ptr, (PGM_P) USER_0)) != NULL)		// ���� � ������ ������ ������ ADMIN, �������� ������� � ������ ������
        {
            for (uint8_t i = 0; i < 12; i++)
            {
                phones.phone_0[i] = *(txt_ptr + i);
            }
            cmd_to_queue (AT_CPBF, NULL, NULL, QUOTES, USER_1, QUOTES);		// ������ ������ ������ �����1
            cmd_to_queue (AT_CPBF, NULL, NULL, QUOTES, USER_2, QUOTES);		// ������ ������ ������ �����2
        }
        else if	((strstr_P((const char*)txt_ptr, (PGM_P) USER_1)) != NULL)	// ���� � ������ ������ ������ USER_1, �������� ������� � ������ ������������
        {
            for (uint8_t i = 0; i < 12; i++)
            {
                phones.phone_1[i] = *(txt_ptr + i);
            }
        }
        else if	((strstr_P((const char*)txt_ptr, (PGM_P) USER_2)) != NULL)	// ���� � ������ ������ ������ USER_2, �������� ������� � ������ ������������
        {
            for (uint8_t i = 0; i < 12; i++)
            {
                phones.phone_2[i] = *(txt_ptr + i);
            }
        }
        else if	((strstr_P((const char*)txt_ptr, (PGM_P) BALANS)) != NULL)	// ���� � ������ ������ ������ ���� balans
        {
            phones.reserv[0] = *txt_ptr;									// ��������� ������� �� ��������� ������
            phones.reserv[1] = *(txt_ptr + 1);
            phones.reserv[2] = *(txt_ptr + 2);
            phones.balance[1] = phones.reserv[0];							// �������� � ������� ������
            phones.balance[2] = phones.reserv[1];
            phones.balance[3] = phones.reserv[2];
        }
    }

    do	//���� ��� r/n/+cmti:_
        if ((txt_ptr = strstr_P((const char*)(msg + n), (PGM_P) ANS_CMTI))!=NULL)	// ���� � msg ���� r/n/+cmti:
        {
            uint8_t j = 0;
            while (isdigit(*(txt_ptr + 14 + j)))	// ���� ������� ����� r/n/+cmti: "SM", - �����, ������� � ��������� ��� � ������ ������ ���
            {
                number[j] = *(txt_ptr + 14 + j);
                j++;
            }
            number[j] = '\0';		// ����������� 0
            inc_to_queue (number);	// ������ � ������� ������� �� ������ ���
            n += 15;				// �������� ��� ������ +cmti: � ���������� ����� ������
            pars_res = 'T';
        }
    while (txt_ptr != NULL);
    msg_clr();
    return pars_res;
}

void inc_to_queue (uint8_t* arr)	//���������� � ������� ������ ������ ���
{
    if ((((queue_H + 1) & QUEUE_IND_MSK) != queue_T) && (((inc_task_H + 1) & INC_TASK_IND_MSK) != inc_task_T)) 	//���� ������ �� ������� �����
    {
        for (uint8_t j=0; j<3; j++)							//�������� ����� ��� � ��������� ������
        {
            RD_SMS[inc_task_H].sms_num[j] = arr[j];
        }
        //�������� ����� ��������
        RD_SMS[inc_task_H].step.flag_1 = RD_SMS[inc_task_H].step.flag_2 = RD_SMS[inc_task_H].step.flag_3 = RD_SMS[inc_task_H].step.flag_4
                = RD_SMS[inc_task_H].step.flag_5 = RD_SMS[inc_task_H].step.flag_6 = RD_SMS[inc_task_H].step.flag_7 = RD_SMS[inc_task_H].step.flag_8 = 0;
        inc_task_H = (inc_task_H + 1) & INC_TASK_IND_MSK;	//�������� ������ ������ ������ ������� �� ������ ���
        HANDLERS[queue_H] = read_sms;						//������ � ������� ������� ��������� ���
        queue_H = (queue_H + 1) & QUEUE_IND_MSK;			//�������� ������ ������ ������� �������
    }
    else
    {
        if  (((inc_task_H + 1) & INC_TASK_IND_MSK) == inc_task_T)
        {
            send_string_to_LCD_XY (inc_ring_ovf, 0, 0);	//������� "��.���.������!"
            _delay_ms(1000);
        }
        if  (((queue_H + 1) & QUEUE_IND_MSK) == queue_T)
        {
            send_string_to_LCD_XY (q_ring_ovf, 0, 1);	//������� "����.������!"
            _delay_ms(1000);
        }
    }
}

void out_to_queue (sms_mask *str)	//���������� � ������� ������ �������� ���
{
    if ((((queue_H + 1) & QUEUE_IND_MSK) != queue_T) && (((out_task_H + 1) & OUT_TASK_IND_MSK) != out_task_T)) 	//���� ������ �� ������� �����
    {
        WR_SMS[out_task_H].sms_txt.sms_type = str->sms_type;		//�������� ���������� � ���������� ��� � ��������� ������
        WR_SMS[out_task_H].sms_txt.dev_num = str->dev_num;
        WR_SMS[out_task_H].sms_txt.param = str->param;
        WR_SMS[out_task_H].sms_txt.ptr = str->ptr;
        WR_SMS[out_task_H].sms_txt.person = str->person;
        //�������� ����� ��������
        WR_SMS[out_task_H].step.flag_1 = WR_SMS[out_task_H].step.flag_2 = WR_SMS[out_task_H].step.flag_3 = WR_SMS[out_task_H].step.flag_4
                = WR_SMS[out_task_H].step.flag_5 = WR_SMS[out_task_H].step.flag_6 = WR_SMS[out_task_H].step.flag_7 = WR_SMS[out_task_H].step.flag_8 = 0;
        out_task_H = (out_task_H + 1) & OUT_TASK_IND_MSK;			//�������� ������ ������ ������ ������� �� �������� ���
        HANDLERS[queue_H] = send_sms;								//������ � ������� ������� ��������� ���
        queue_H = (queue_H + 1) & QUEUE_IND_MSK;					//�������� ������ ������ ������� �������
    }
    else
    {
        if  (((out_task_H + 1) & OUT_TASK_IND_MSK) == out_task_T)
        {
            send_string_to_LCD_XY (out_ring_ovf, 0, 0);	//������� "���.���.������!"
            _delay_ms(1000);
        }
        if  (((queue_H + 1) & QUEUE_IND_MSK) == queue_T)
        {
            send_string_to_LCD_XY (q_ring_ovf, 0, 1);	//������� "����.������!"
            _delay_ms(1000);
        }
    }
}

void cmd_to_queue (uint8_t *cmd, uint8_t *pgm_par_1, uint8_t *ram_par, uint8_t *pgm_par_2, uint8_t *pgm_par_3, uint8_t *pgm_par_4)	//���������� � ������� ������ �������� �������
{
    if ((((queue_H + 1) & QUEUE_IND_MSK) != queue_T) && (((cmd_task_H + 1) & CMD_TASK_IND_MSK) != cmd_task_T)) 	//���� ������ �� ������� �����
    {
        WR_CMD[cmd_task_H].cmd = cmd;									//���������� ��������� �� ����� ������� � ������
        WR_CMD[cmd_task_H].pgm_par_1 = pgm_par_1;						//���������� ���������1 �� ����-�������� ������� � ������
        WR_CMD[cmd_task_H].ram_par = ram_par;							//���������� ��������� �� ���-�������� ������� � ������
        WR_CMD[cmd_task_H].pgm_par_2 = pgm_par_2;						//���������� ���������1 �� ����-�������� ������� � ������
        WR_CMD[cmd_task_H].pgm_par_3 = pgm_par_3;						//���������� ���������2 �� ����-�������� ������� � ������
        WR_CMD[cmd_task_H].pgm_par_4 = pgm_par_4;						//���������� ���������2 �� ����-�������� ������� � ������
        //�������� ����� ��������
        WR_CMD[cmd_task_H].step.flag_1 = WR_CMD[cmd_task_H].step.flag_2 = WR_CMD[cmd_task_H].step.flag_3 = WR_CMD[cmd_task_H].step.flag_4
                = WR_CMD[cmd_task_H].step.flag_5 = WR_CMD[cmd_task_H].step.flag_6 = WR_CMD[cmd_task_H].step.flag_7 = WR_CMD[cmd_task_H].step.flag_8 = 0;
        cmd_task_H = (cmd_task_H + 1) & CMD_TASK_IND_MSK;				//�������� ������ ������ ������ ������� �� �������� �������
        HANDLERS[queue_H] = send_cmd;									//������ � ������� ������� ��������� �������
        queue_H = (queue_H + 1) & QUEUE_IND_MSK;						//�������� ������ ������ ������� �������
    }
    else
    {
        if  (((cmd_task_H + 1) & CMD_TASK_IND_MSK) == cmd_task_T)
        {
            send_string_to_LCD_XY (cmd_ring_ovf, 0, 0);	//������� "���.���.������!"
            _delay_ms(1000);
        }
        if  (((queue_H + 1) & QUEUE_IND_MSK) == queue_T)
        {
            send_string_to_LCD_XY (q_ring_ovf, 0, 0);	//������� "����.������!"
            _delay_ms(1000);
        }
    }
}

void to_do (void)			// ������ ������� � ���������� �������
{
    uint8_t chng_done = 0; 								// ������� ��������� ��������� ������� �� ���������
    uint8_t digits[N_DIGS];								// ��������� ������ ���� ����������
    uint8_t n = eeprom_read_byte((uint8_t*)dev_qty);	// ��������� ���������� ���������
    char* txt_ptr = NULL;								// ��������� �� ������ �������� ������ � ������ todo_txt

    if (todo_txt[0]=='T' && todo_txt[1]==' ' && todo_txt[2]=='A' && todo_txt[3]=='L' && todo_txt[4]=='L')
    {
        sms_buff.sms_type = ALL;
        out_to_queue (&sms_buff);
    }
    else if ((todo_txt[0]=='R')&&(todo_txt[1]=='E')&&(todo_txt[2]=='N')&&(todo_txt[3]==' '))
    {
        if (sms_buff.person == ADMIN)									// ���� ��� �� ������, ���������
        {
            for (uint8_t i = 0; i < n; i++)
            {
                eeprom_read_block (&buffer.name, &ee_arr[i].name, sizeof(buffer.name));
                if (!strncmp((void*)buffer.name, (void*)&todo_txt[4], sizeof(buffer.name)-1))
                {
                    location = &todo_txt[12];
                    strncpy((void*)buffer.name, (void*)location, sizeof (buffer.name)-1); // ���������� ASCII ��� ����� � ���� name ������
                    cli();
                    eeprom_update_block (&buffer.name, &ee_arr[i].name, sizeof(buffer.name)-1);
                    sei();
                    chng_done = 1;
                    sms_buff.sms_type = REN_DONE;
                }
            }
            if (!chng_done)
            {
                lcd_clr();
                send_string_to_LCD_XY (name_error_ren, 0, 0);
                sms_buff.sms_type = NAME_ERR;
            }
        }
        else
        {
            sms_buff.sms_type = DENY;
        }
        out_to_queue (&sms_buff);
    }
    else if ((todo_txt[0]=='T')&&((todo_txt[1]=='L')||(todo_txt[1]=='H'))&&(todo_txt[2]=='A')&&(todo_txt[3]=='L')&&(todo_txt[4]==' ')&&(todo_txt[4 + N_NAME]==' ')&&((todo_txt[5 + N_NAME]=='-')||(todo_txt[5 + N_NAME]=='+')))
    {
        if (sms_buff.person == ADMIN)																		// ���� �������� �����
        {
            int8_t t_lim = atoi_fast (&todo_txt[5 + N_NAME]);
            if ((t_lim >= -127) && (t_lim <= 127))
            {
                for (uint8_t i = 0; i < n; i++)
                {
                    eeprom_read_block (&buffer.name, &ee_arr[i].name, sizeof(buffer.name));
                    if (!strncmp((void*)buffer.name, (void*)&todo_txt[5], sizeof(buffer.name)-1))
                    {
                        if (todo_txt[1]=='L')
                        {
                            buffer.tmin = t_lim;
                            cli();
                            eeprom_update_block (&buffer.tmin, &ee_arr[i].tmin, sizeof buffer.tmin);
                            sei();
                            send_arr_to_LCD_XY(buffer.name, 0, 0);
                            send_string_to_LCD (blank);
                            send_string_to_LCD (t_min);
                            send_string_to_LCD (blank);
                            lcd_dat ((buffer.tmin & 0b10000000) ? '-' : '+');
                            send_arr_to_LCD (utoa_fast_div (((buffer.tmin & 0b10000000) ? ((~buffer.tmin) + 1) : buffer.tmin), digits));
                            chng_done = 1;
                            sms_buff.sms_type = MIN_LIM_SET;								// ��� ������ �������������
                            sms_buff.param = buffer.tmin;
                        }
                        else if (todo_txt[1]=='H')
                        {
                            buffer.tmax = t_lim;
                            cli();
                            eeprom_update_block (&buffer.tmax, &ee_arr[i].tmax, sizeof buffer.tmax);
                            sei();
                            send_arr_to_LCD_XY(buffer.name, 0, 0);
                            send_string_to_LCD (blank);
                            send_string_to_LCD (t_max);
                            send_string_to_LCD (blank);
                            lcd_dat ((buffer.tmax & 0b10000000) ? '-' : '+');
                            send_arr_to_LCD (utoa_fast_div (((buffer.tmax & 0b10000000) ? ((~buffer.tmax) + 1) : buffer.tmax), digits));
                            chng_done = 1;
                            sms_buff.sms_type = MAX_LIM_SET;								// ��� ������ �������������
                            sms_buff.param = buffer.tmax;
                        }
                        sms_buff.dev_num = i;
                    }
                }
                if (!chng_done)
                {
                    sms_buff.sms_type = NAME_ERR;
                    send_string_to_LCD_XY (name_error_al, 0, 0);
                }
            }
            else
            {
                send_string_to_LCD_XY (t_error, 0, 0);
                sms_buff.sms_type = LIM_ERR;
            }
        }
        else																				// ���� �������� ����
        {
            sms_buff.sms_type = DENY;														// ��� access denied
        }
        out_to_queue (&sms_buff);
    }


    else if ((todo_txt[0]=='S')&&(todo_txt[1]=='M')&&(todo_txt[2]=='S')&&(todo_txt[3]==' ')&&(todo_txt[4]=='T')&&((todo_txt[5]=='L')||(todo_txt[5]=='H'))&&((todo_txt[6]=='1')||(todo_txt[6]=='0'))&&(todo_txt[7]==' '))
    {
        for (uint8_t i = 0; i < n; i++)
        {
            eeprom_read_block (&buffer.name, &ee_arr[i].name, sizeof(buffer.name));
            eeprom_read_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags));
            if (!strncmp((void*)buffer.name, (void*)&todo_txt[8], sizeof(buffer.name)-1))
            {
                if ((todo_txt[5]=='L')&&(todo_txt[6]=='1'))
                {
                    if (sms_buff.person == ADMIN)
                    {buffer.flags.sms_T_0 = 1;}
                    else if (sms_buff.person == ABNT_1)
                    {buffer.flags.sms_T_1 = 1;}
                    else if (sms_buff.person == ABNT_2)
                    {buffer.flags.sms_T_2 = 1;}
                    cli();
                    eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                    sei();
                    send_arr_to_LCD_XY(buffer.name, 0, 0);
                    send_string_to_LCD_XY (t_low, 0, 1);
                    send_string_to_LCD(t_min);
                    send_string_to_LCD (blank);
                    send_string_to_LCD (sms_send);
                    send_string_to_LCD (blank);
                    send_string_to_LCD (on);
                    chng_done = 1;
                    sms_buff.sms_type = SMS_ON;
                }
                else if ((todo_txt[5]=='H')&&(todo_txt[6]=='1'))
                {
                    if (sms_buff.person == ADMIN)
                    {buffer.flags.sms_T_0 = 1;}
                    else if (sms_buff.person == ABNT_1)
                    {buffer.flags.sms_T_1 = 1;}
                    else if (sms_buff.person == ABNT_2)
                    {buffer.flags.sms_T_2 = 1;}
                    cli();
                    eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                    sei();
                    lcd_clr();
                    send_arr_to_LCD_XY(buffer.name, 0, 0);
                    send_string_to_LCD_XY (t_high, 0, 1);
                    send_string_to_LCD (t_max);
                    send_string_to_LCD (blank);
                    send_string_to_LCD (sms_send);
                    send_string_to_LCD (blank);
                    send_string_to_LCD (on);
                    chng_done = 1;
                    sms_buff.sms_type = SMS_ON;
                }
                else if ((todo_txt[5]=='L')&&(todo_txt[6]=='0'))
                {
                    if (sms_buff.person == ADMIN)
                    {buffer.flags.sms_T_0 = 0;}
                    else if (sms_buff.person == ABNT_1)
                    {buffer.flags.sms_T_1 = 0;}
                    else if (sms_buff.person == ABNT_2)
                    {buffer.flags.sms_T_2 = 0;}
                    cli();
                    eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                    sei();
                    lcd_clr();
                    send_arr_to_LCD_XY(buffer.name, 0, 0);
                    send_string_to_LCD_XY (t_low, 0, 1);
                    send_string_to_LCD(t_min);
                    send_string_to_LCD (blank);
                    send_string_to_LCD (sms_send);
                    send_string_to_LCD (blank);
                    send_string_to_LCD (off);
                    chng_done = 1;
                    sms_buff.sms_type = SMS_OFF;
                }
                else if ((todo_txt[5]=='H')&&(todo_txt[6]=='0'))
                {
                    if (sms_buff.person == ADMIN)
                    {buffer.flags.sms_T_0 = 0;}
                    else if (sms_buff.person == ABNT_1)
                    {buffer.flags.sms_T_1 = 0;}
                    else if (sms_buff.person == ABNT_2)
                    {buffer.flags.sms_T_2 = 0;}
                    cli();
                    eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                    sei();
                    send_arr_to_LCD_XY(buffer.name, 0, 0);
                    send_string_to_LCD_XY (t_high, 0, 1);
                    send_string_to_LCD(t_max);
                    send_string_to_LCD (blank);
                    send_string_to_LCD (sms_send);
                    send_string_to_LCD (blank);
                    send_string_to_LCD (off);
                    chng_done = 1;
                    sms_buff.sms_type = SMS_OFF;
                }
                sms_buff.dev_num = i;
            }
        }
        if (!chng_done)
        {
            sms_buff.sms_type = NAME_ERR;
            send_string_to_LCD_XY (name_error_sms, 0, 0);
        }
        out_to_queue (&sms_buff);
    }

    else if ((strstr_P((const char*)todo_txt, (PGM_P) USER_0)) != NULL)//���� � todo_txt ���� ADMIN
    {
        if (sms_buff.person == ADMIN)
        {
            uint8_t i = 0;
            uint8_t j = 0;
            while ((todo_txt [i] != '+')&&(i < TODO_MAX))	// ���� +
            {
                i++;
            }
            if (todo_txt [i] == '+')
            {
                phones.phone_0[0] = todo_txt [i];			// ���������� + ������ �������� � ������ �������� ������
                j += 1;
                i += 1;
                while (isdigit(todo_txt [i])&&(j < 12)) 	// ������������ ����� ����� + � ������ �������� ������
                {
                    phones.phone_0[j] = todo_txt [i];
                    i++;
                    j++;
                }
                if (j == 12)								// ���� ������� + � 12 ����
                {
                    cmd_to_queue (AT_CPBW, CELL_2, phones.phone_0, TEXT_145, USER_0, QUOTES);	// ���������� ���� ������� �� ���
                    sms_buff.sms_type = ADMIN;													// ���������� ������������� ������ (�������)
                }
            }
        }
        else
        {
            sms_buff.sms_type = DENY;															// ��� access denied
        }
        out_to_queue (&sms_buff);
    }

    else if ((strstr_P((const char*)todo_txt, (PGM_P) USER_1)) != NULL)//���� � todo_txt ���� user_1
    {
        if (sms_buff.person == ADMIN)
        {
            uint8_t i = 0;
            uint8_t j = 0;
            while ((todo_txt [i] != '+')&&(i < TODO_MAX))	// ���� +
            {
                i++;
            }
            if (todo_txt [i] == '+')
            {
                phones.phone_1[0] = todo_txt [i];								// ���������� + ������ �������� � ������ �������� user_1
                j += 1;
                i += 1;
                while (isdigit(todo_txt [i])&&(j < 12)) 						// ������������ ����� ����� + � ������ �������� user_1
                {
                    phones.phone_1[j] = todo_txt [i];
                    i++;
                    j++;
                }
                if (j == 12)													// ���� ������� + � 12 ����
                {
                    cmd_to_queue (AT_CPBW, CELL_3, phones.phone_1, TEXT_145, USER_1, QUOTES);	//���������� ���� ������� �� ���
                    sms_buff.sms_type = MEMBERS;								// ���������� ������������� ������
                }
            }
        }
        else
        {
            sms_buff.sms_type = DENY;															// ��� access denied
        }
        out_to_queue (&sms_buff);
    }

    else if ((strstr_P((const char*)todo_txt, (PGM_P) USER_2)) != NULL)//���� � todo_txt ���� user_2
    {
        if (sms_buff.person == ADMIN)
        {
            uint8_t i = 0;
            uint8_t j = 0;
            while ((todo_txt [i] != '+')&&(i < TODO_MAX))	// ���� +
            {
                i++;
            }
            if (todo_txt [i] == '+')
            {
                phones.phone_2[0] = todo_txt [i];								// ���������� + ������ �������� � ������ �������� user_2
                j += 1;
                i += 1;
                while (isdigit(todo_txt [i])&&(j < 12)) 						// ������������ ����� ����� + � ������ �������� user_2
                {
                    phones.phone_2[j] = todo_txt [i];
                    i++;
                    j++;
                }
                if (j == 12)													// ���� ������� + � 12 ����
                {
                    cmd_to_queue (AT_CPBW, CELL_3, phones.phone_2, TEXT_145, USER_2, QUOTES);	//���������� ���� ������� �� ���
                    sms_buff.sms_type = MEMBERS;								// ���������� ������������� ������
                }
            }
        }
        else
        {
            sms_buff.sms_type = DENY;															// ��� access denied
        }
        out_to_queue (&sms_buff);
    }

    else if (strstr_P((const char*)todo_txt, (PGM_P) USERS) != NULL)		//���� � todo_txt ���� USERS
    {
        if (sms_buff.person == ADMIN)
        {
            sms_buff.sms_type = MEMBERS;									// ��� ������ ������ �������������
        }
        else
        {
            sms_buff.sms_type = DENY;										// ��� ����������� ����� � �������
        }
        out_to_queue (&sms_buff);
    }

    else if ((txt_ptr = strstr_P((const char*)todo_txt, (PGM_P) BALANCE)) != NULL)	//���� � todo_txt ���� BALANCE
    {
        phones.ans_to = sms_buff.person;								// ���������, ��� ������� ������, ������ ��������� ����� ���
        txt_ptr = txt_ptr + 8;											// ������ ��������� ����� BALANCE � �������
        if ((*txt_ptr == '*')||(*txt_ptr == '#')||(isdigit(*txt_ptr)))  // ���� ����� BALANCE � ������� ���� *,# ��� �����
        {
            if (sms_buff.person == ADMIN)
            {
                if (!isdigit(*txt_ptr))									// ���� ����� BALANCE � ������� �� �����
                {
                    txt_ptr +=1;										// �������� ��� �� 1
                }
                uint8_t i = 0;
                while ((isdigit(txt_ptr[i]))&&(i < 3))
                {
                    phones.balance[i + 1] = txt_ptr[i];					// ���������� ��� ����� � ������� ������ �������� �������
                    phones.reserv [i] = txt_ptr[i];						// ���������� ��� ����� � ������ ��������� �������� ������� ��� ������ �� ���
                    i++;
                }
                if (i == 3)												// ���� ���������� ��� �����, �� ��
                {
                    cmd_to_queue (AT_CPBW, CELL_1, phones.reserv, TEXT_129, BALANS, QUOTES);// ���������� � ��� �������� ����� �������
                }
                else													// ���� �� ���������� ��� �����, �������� �������
                {
                    sms_buff.sms_type = COM_ERR;
                    out_to_queue (&sms_buff);
                }
            }
            else
            {
                sms_buff.sms_type = DENY;								// ��� ����������� ����� � �������
                out_to_queue (&sms_buff);
            }
        }
        cmd_to_queue (AT_CUSD, QUOTES, phones.balance, QUOTES, NULL, NULL);// � ����� ������ ������ �������, � ��������� �������� ������������ ���� �� �����
    }

    else if ((txt_ptr = strstr_P((const char*)todo_txt, (PGM_P) DELETE)) != NULL)		// ���� � todo_txt ���� DELETE
    {
        if (sms_buff.person == ADMIN)
        {
            txt_ptr = txt_ptr + 5;
            if (strstr((const char*)txt_ptr, (char*)phones.phone_1))					// ���� ����� �� ��� ����� ���� � �������
            {
                for (uint8_t i = 0; i < 12; i++)										// ������� ������������
                {phones.phone_1[i] = '0';}
                cmd_to_queue(AT_CPBW, CELL_3, phones.phone_1, TEXT_145, USER_1, QUOTES);// ������ � ��� ������� � ������
                sms_buff.sms_type = MEMBERS;											// ���������� ���������� ������ ������
            }
            else if (strstr((const char*)txt_ptr, (char*)phones.phone_2))				// ���� ����� �� ��� ����� ���� � �������
            {
                for (uint8_t i = 0; i < 12; i++)										// ������� ������������
                {phones.phone_2[i] = '0';}
                cmd_to_queue(AT_CPBW, CELL_4, phones.phone_2, TEXT_145, USER_2, QUOTES);// ������ � ��� ������� � ������
                sms_buff.sms_type = MEMBERS;											// ���������� ���������� ������ ������
            }
            else
            {
                sms_buff.sms_type = COM_ERR;
            }
        }
        else
        {
            sms_buff.sms_type = DENY;													// ��� ����������� ����� � �������
        }
        out_to_queue (&sms_buff);
    }

    else
    {
        sms_buff.sms_type = COM_ERR;
        send_string_to_LCD_XY (com_error, 0, 0);
        out_to_queue (&sms_buff);
    }
}

/*			################################			������ ����� ������������ ����������				##################*/

ISR(USART_RX_vect)	// ���������� ���������� ��� �������� �� ������� ������ � UDR0
{
    uint8_t temp;				// ��������� ���������� ��� ����� �� UDR0	#################
    uint8_t wr_err;				// ��� ���������� ������ � ������			#################
    pause_cnt = 0;				// ����� �������� ����� � ISR(TIMER0_OVF_vect) ����� ����� ���������� ����� #############
    if(UCSR0A & (1 << FE0))		// ������ ������������, �� ����� ����� ������, ��������
    {
        temp = UDR0;			// ����� ���� ���������, ����� ���� FE �� ���������, �� ������� �� �����
        send_string_to_LCD_XY(frame_err, 0, 0);
        //string_to_USART (frame_err);#############
        // _delay_ms(1500);
    }
    else
    {
        temp = UDR0;
        wr_err = UDR_to_RX_Ring(temp);		// ����������� ������ � ������ � ��������� ��� ����������
        if(wr_err == 1)						// ��� ������ ������������ ������� ���������, ������ � UDR_to_RX_Ring �� ��������
        {
            send_string_to_LCD_XY(rx_ring_ovf, 0, 0);
            //string_to_USART (rx_ring_ovf);	#############
            // _delay_ms(1500);
        }
    }
}

ISR (USART_UDRE_vect)  	// ���������� ���������� �� ����������� UDR �����������
{
    TX_IndexOUT++;
    TX_IndexOUT &= TX_IND_MSK;		// �������� ����� ���������� ������
    UDR0 = TX_ring[TX_IndexOUT];	// ������ �� ���������� ������ � UDR
    if (!Get_TX_Data())				// ���� ����� ��� ����
    {
        UCSR0B &= ~(1<<UDRIE0); 	// ������ ���������� �� ����������� UDR �����������
    }
}

ISR(TIMER0_OVF_vect) 				// ���������� ���������� ������� 0
{
    int_cnt++; 						//������� ����������
    if (int_cnt == 2) 				//��� ������������ �������� ������� 1024 ����� ���.�������� �� 2, ����� ����� 30��
    {
        BTN_SCAN (); 				//��������� ������
        int_cnt = 0; 				//�������� ����� ������
    }

    if (delay_cnt < IND_PAUSE)
    {delay_cnt++;}				//������� �������� 2c ��� ���������

    if (wait_timer < WAIT_LIM)
    {wait_timer++;}				//������� ����� 15c �� �������� ������������

    flash_cnt++;					//������� ��� ��������� ������� ��������� ������ �������
    if (flash_cnt == 10)
    {
        flash_cnt = 0;
        if (!pause_num)
        {
            if (flash_num < gsm_lvl)	//################### ����� ������� ����� gsm_lvl/2
            {
                flash_num++;			//###############
                PORTB ^= _BV(LED_PIN_NUM);
            }
            else
            {
                flash_num = 0;
                pause_num++;
                PORTB &= ~_BV(LED_PIN_NUM);
            }
        }
        else
        {
            if (pause_num >= 16){pause_num = 0;}	//###############
            else {pause_num++;}						//##############
        }
    }

    if (time_gsm < GSM_LVL_TIME)
    {time_gsm++;}					//������� ���������� �������� GSM #############
    else if (time_gsm == GSM_LVL_TIME)
    {
        time_gsm +=1;					//����� ��� ��������� ���������� ���� �� �������, ���� �� ��������� � main
        gsm_lvl_req = 1;				//������ ���� - ��������� ������ ������ GSM
    }

    if (pause_cnt < PAUSE_CNT_MAX)
    {pause_cnt++;} 					//������� ����� ����� ����� ���������� ����� � ������ �������� #############
    else if (pause_cnt == PAUSE_CNT_MAX)
    {
        //byte_to_TX_Ring(0x13);			//XOFF
        //UCSR0B |= (1<<UDRIE0);			//���������� ���������� �� ����������� UDR �����������
        USART_TXD (0x13);				//XOFF ������ ��������
        msg_upld = 1;					//������ ���� ������� ����� �������
        pause_cnt += 1;					//����� ��� ��������� ���������� ���� �� �������, ���� �� ��������� � ISR(USART_RX_vect)
    }

    if (ans_cnt < ans_lim) {ans_cnt++;}	//������� ��������� ��� ������ ������ �� ������� ��� ��� ������ ##############
}
/*			################################			����� ����� ������������ ����������				##################*/

void menu(uint8_t* qty, uint8_t* active) 	// �� ���� ��� ��������/����������/������� ���������
{
    uint8_t  i, j = 0;									// ��������, ��� �������� i - ������, j - �������
    static uint8_t  k = 0;								// ������� ��� ������ 3 � 8, static ����� ��-�� ������� � ���������� �� � �������� ������ ������������
    static uint8_t  n = 0;								// ���������� ��������, ���������� � �����, static ��� ���������� ����� ����������
    static uint8_t  data [8];							// �����-������ ��� �������� ����� ��������, static ��� ���������� ����� ����������
    uint8_t  New_conflict = 0;							// ���������� ��� ����� ������� ���� ���������
    uint8_t  last_conflict = 0;							// ���������� ��� ������ ������� ���� ���������
    uint8_t digits[N_DIGS];								// ������ ��� �������� � utoa_fast_div ��� ���������� ��� ������ �������� ����
    static uint8_t  stage, _stage = 0;					//����� ��������� ��, ������� � ����������

    if (!(*active)) return;								// ���� ���������, �� ������
    if (delay_cnt < IND_PAUSE) return;					// ���� ����� ��������� �� �������, �� ������
    if (wait_timer == WAIT_LIM)							// ���� �� ��������� �������� ������������
    {
        wait_timer = WAIT_LIM + 1;						// +1 ����� ��� ��������� ����� �� ��������� �������� if(wait_timer==WAIT_LIM)
        if (!(n = eeprom_read_byte((uint8_t*)dev_qty))) //���� n==0, ������� � ����
        {
            stage = 10;
            return;
        }
        else											//���� n!=0, ������ ������������ �� ���� � ������� � main
        {
            k = n = stage = _stage = *active = 0;		// �.�. ���� ��������, �� n �� ���������� � �� ����� ��������� ��� � main
            return;
        }

    }
    switch (stage)
    {
        case 0:
            str_clr(0);									// ������� 1-� ������
            str_clr(1);									// ������� 2-� ������
            send_string_to_LCD_XY (correction, 0, 0);	// ������� "�������������"
            n = eeprom_read_byte((uint8_t*)dev_qty);	// ������ ���������� ��������, ���������� � �����
            for (j = 0; j < 8; j++)   {data[j] = 0x00;} // ������� �����-������
            delay_cnt = 0;								// ��������� ������ ����� �� ���������
            stage = 1;
            break;

        case 1:
            str_clr(0);									// ������� 1-� ������
            str_clr(1);									// ������� 2-� ������
            if (!n)
            {
                send_string_to_LCD_XY (subst_add, 0 ,0);// ������� "������/����������"
                delay_cnt = 0;							// ��������� ������ ����� �� ���������
                stage = 5;
            }
            else
            {
                send_string_to_LCD_XY (delete, 0, 0);	// ������� "�������?"
                wait_timer = 0;
                stage = 2;
            }
            break;

        case 2:
            if (press_time == SLOW)
            {
                wait_timer = WAIT_LIM + 1;				//+1 ����� �������� (wait_timer==WAIT_LIM) �� ����� �� ��������� � main
                press_time = 0;
                stage = 3;
            }
            else if (press_time == QUICK)
            {
                wait_timer = WAIT_LIM + 1;				//+1 ����� �������� (wait_timer==WAIT_LIM) �� ����� �� ��������� � main
                press_time = 0;
                str_clr(0);								// ������� 1-� ������
                send_string_to_LCD_XY (subst_add, 0 ,0);// ������� "������/����������"
                delay_cnt = 0;							// ��������� ������ ����� �� ���������
                stage = 4;
            }
            break;

        case 3:
            if (_stage != stage)	//���� ������ ����
            {
                eeprom_read_block (&buffer, &ee_arr[k], sizeof(buffer));	// ��������� �������� 1-�� ��������� �� �����
                send_arr_to_LCD_XY (buffer.name, 0, 1);						// ������� � ������ 2 lcd ��� �������
                _stage = stage;
                wait_timer = 0;
                return;
            }
            if (press_time == SLOW)
            {
                wait_timer = WAIT_LIM + 1;									//+1 ����� �������� (wait_timer==WAIT_LIM) �� ����� �� ��������� � main
                press_time = 0;
                for (; k < n; k++)
                {
                    eeprom_read_block (&buffer, &ee_arr[k+1], sizeof(buffer));	// ��������� �������� ���������� ��������� �� �����
                    cli();
                    eeprom_update_block (&buffer, &ee_arr[k], sizeof(buffer));	// ���������� � ����� �������� ���������� ��������� �� ������� ������
                    sei();
                }
                cli();
                eeprom_update_byte ((uint8_t*)dev_qty, (n-1));
                sei();
                str_clr(0);								// ������� 1-� ������
                str_clr(1);								// ������� 2-� ������
                send_string_to_LCD_XY (del_done, 0, 0); // ������� "�������"
                delay_cnt = 0;							// ��������� ������ ����� �� ���������
                stage = 10;
            }
            else if (press_time == QUICK)
            {
                wait_timer = WAIT_LIM + 1;				//+1 ����� �������� (wait_timer==WAIT_LIM) �� ����� �� ��������� � main
                press_time = 0;
                if ((k + 1) < n)
                {
                    k += 1;
                    eeprom_read_block (&buffer, &ee_arr[k], sizeof(buffer));// ��������� �������� ��������� �� �����
                    str_clr(1);												// ������� 2-� ������
                    send_arr_to_LCD_XY (buffer.name, 0, 1);					// ������� � ������ 2 lcd ��� �������
                    wait_timer = 0;
                }
                else
                {
                    str_clr(0);										// ������� 1-� ������
                    str_clr(1);										// ������� 2-� ������
                    send_string_to_LCD_XY (subst_add, 0 ,0);		// ������� "������/����������"
                    delay_cnt = 0;									// ��������� ������ ����� �� ���������
                    stage = 4;
                }
            }
            break;

        case 4:
            str_clr(0);												// ������� 1-� ������
            str_clr(1);												// ������� 2-� ������
            send_string_to_LCD_XY (total_qty, 0, 0);				// ������� "����� ���������"
            send_arr_to_LCD_XY (utoa_fast_div (n, digits), 7, 1);	// ������� ���������� ����-�, utoa_fast_div ����� ��������� �� ������ ��������� ������ ������� digits
            delay_cnt = 0;											// ��������� ������ ����� �� ���������
            stage = 5;
            /*
            for (not_act = 0; not_act < n; not_act++)	// ��� ������� ������ � 5 � 8 ����� ������� �����������
            {
                eeprom_read_block (&buffer.flags, &ee_arr[k].flags, sizeof(buffer.flags));//��������� ���� 1W ���������� �� �����
                if (!buffer.flags.active)
                    {break;}						// ���� � ����� ���� ���������� ����-��, ����� � � == � �����������
                else if (not_act == (n - 1))		// ���� ��������� ���������
                    {not_act = n_max;}				// ���� ���������� ���, ������ ����������� ����-�� ������������� �������� ��������
            }
            */
            break;

        case 5:
            str_clr(0);		// ������� 1-� ������
            str_clr(1);		// ������� 2-� ������
            if (n == n_max) // ���� ����� ���������, ��������, ��� �� ����������
            {
                /* ������ �� for(){;} �� else ����, ������������ ��������� ������ ����������� �� 4
                if (not_act < n_max)
                {
                    send_string_to_LCD_XY (plug_in, 0, 0);		// ������� "���������� ����-��"
                    send_string_to_LCD_XY (press_btn, 0, 1);	// ������� "� ������� ������"
                    wait_timer = 0;
                    stage = 6;
                    return;
                }
                else
                {
                    send_string_to_LCD_XY (mem_full, 0, 0); 	// ������� "������ ���������"
                    delay_cnt = 0;								// ��������� ������ ����� �� ���������
                    stage = 10;
                    return;
                }
                */
                for (i = 0; i < n; i++)
                {
                    eeprom_read_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags));//��������� ���� 1W ���������� �� �����
                    if (!buffer.flags.active)					// ���� � ����� ���� ����������, ��������� � ������
                    {
                        send_string_to_LCD_XY (plug_in, 0, 0);	// ������� "���������� ����-��"
                        send_string_to_LCD_XY (press_btn, 0, 1);// ������� "� ������� ������"
                        wait_timer = 0;
                        stage = 6;
                        return;
                    }
                }
                send_string_to_LCD_XY (mem_full, 0, 0); 		// ������� "������ ���������"
                delay_cnt = 0;									// ��������� ������ ����� �� ���������
                stage = 10;
            }
            else
            {
                send_string_to_LCD_XY (plug_in, 0, 0);			// ������� "���������� ����-��"
                send_string_to_LCD_XY (press_btn, 0, 1);		// ������� "� ������� ������"
                wait_timer = 0;
                stage = 6;
            }
            break;

        case 6:
            if (press_time == 0) return;
            else
            {
                wait_timer = WAIT_LIM + 1;						//+1 ����� �������� (wait_timer==WAIT_LIM) �� ����� �� ��������� � main
                press_time = 0;
                do
                {
                    New_conflict = 0;
                    find_ID (data, &New_conflict, &last_conflict); 		// ������ �� ��������� ������ ��� ���������� ���������� ����������
                    if (CRC_check (data, 0x07)) 						// ������� ��������� �� ������ � ID, ����� ����� CRC8; ���� �������� 1, CRC �� ��
                    {
                        send_string_to_LCD_XY (error, 0, 0);			// ������� "������ ����." � 1-� ������� ����� ������� 1 ������ ������
                    }
                    //�������� ��������� ���������� ID � ���������� � �����
                    i = 0;
                    do //���� for(i=0;i<n;i++) ������� �� do-while(i<n), ����� ��� n==0 � ���� �� �������, �.�. i==n==0 �����.
                    {
                        eeprom_read_block (&buffer.code, &ee_arr[i].code, sizeof(buffer.code));		// ��������� ID ��������� �� �����
                        if (!strncmp((void*)data, (void*)buffer.code, sizeof data))					// ���� ��������� ID ������ � ���������� � ����� - ������ ������ ID
                            //���������� ������������ ����� ���������� � void*, �.�. strncmp �� ��� ������, ����� char*, � � ��� unsigned char*
                            break;	//����� � ������� do_while
                        else if (strncmp((void*)data, (void*)buffer.code, sizeof data) && ((i == (n-1))||(i == n))) // ���� ��������� ID �� ������ �� � ����� ���������� � ����� -> ������� ����� ����-��
                            //��� ������������ ���������� �� � ����� ���������� ��� ������ n==0 � ������� i==(n-1) ��������� ������� ||(i==n) , ����� ��� ������ ��� n==0 �� ��������
                        {
                            str_clr(0);	//������� 1-� ������
                            str_clr(1);	//������� 2-� ������
                            send_string_to_LCD_XY (new_dev_fnd, 0, 0);	//������� "������� �����" � 1-� ������� ����� ������� 1 ������ ������
                            send_string_to_LCD_XY (element, 0, 1);		//������� "����������" � 1-� ������� ����� ������� 2 ������ ������
                            delay_cnt = 0;
                            stage = 7;
                            return;
                        }
                        i++;
                    }
                    while (i < n);
                }
                while (last_conflict != 0); 			// ���� ����� ���� ��������� �� ����� 0, ���� ����� �� ��� ������� ���������
                str_clr(0);								// ������� 1-� ������
                str_clr(1);								// ������� 2-� ������
                send_string_to_LCD_XY (no_new, 0, 0);	// ������� "����� ���"
                delay_cnt = 0;							// ��������� ������ ����� �� ���������
                stage = 10;
            }
            break;

        case 7:
            str_clr(0);	// ������� 1-� ������
            str_clr(1);	// ������� 2-� ������
            if (n)		// ����� ��� � ������ ������, ���� n!=0
            {
                stage = 8;
            }
            else	// ���������� ������ ������, ���� n==0
            {
                send_string_to_LCD_XY (add_to_end, 0, 0);// ������� "��������?"
                wait_timer = 0;
                stage = 9;
            }
            break;

        case 8:
            if (stage != _stage)	// ���� ������ ����
            {
                /* ������ �� for(){;} �� else ����, ������������ ��������� ������ ����������� �� 4
                if (not_act < n_max)
                {
                    k = not_act;
                    eeprom_read_block (&buffer.name, &ee_arr[k].name, sizeof(buffer.name)); // ��������� ��� ����������� ����-��
                    send_string_to_LCD_XY (substitute, 0, 0);								// ������� "��������?"
                    send_arr_to_LCD_XY (buffer.name, 0, 1); 								// ������� ��� ����������� ����-��
                    wait_timer = 0;
                    press_time = 0;
                    _stage = stage;
                    return;
                }
                else
                {
                    send_string_to_LCD_XY (add_to_end, 0, 0);//������� "��������?"
                    wait_timer = 0;
                    stage = 9;
                    return;
                }
                */
                for (k = 0; k < n; k++)
                {
                    eeprom_read_block (&buffer.flags, &ee_arr[k].flags, sizeof(buffer.flags));	// ��������� ���� 1W ���������� �� �����
                    if (!buffer.flags.active) 													// ���� � ����� ���� ����������
                    {
                        eeprom_read_block (&buffer.name, &ee_arr[k].name, sizeof(buffer.name));	// ��������� ��� ����������� ����-��
                        send_string_to_LCD_XY (substitute, 0, 0);								// ������� "��������?"
                        send_arr_to_LCD_XY (buffer.name, 0, 1); 								// ������� ��� ����������� ����-��
                        wait_timer = 0;
                        press_time = 0;
                        _stage = stage;
                        return;
                    }
                }
                send_string_to_LCD_XY (add_to_end, 0, 0);//������� "��������?"
                wait_timer = 0;
                stage = 9;
            }
            else
            {
                if (press_time == SLOW)
                {
                    wait_timer = WAIT_LIM + 1;							//+1 ����� �������� (wait_timer==WAIT_LIM) �� ����� �� ��������� � main
                    for (j=0; j < 8; j++)   {buffer.code[j] = data[j];}	// ������ ID-���� 1-�� 1W ���������� � ID_string � ����������� ��� � �����-���������
                    buffer.flags.active = 1; // ������ �������
                    cli();
                    eeprom_update_block (&buffer.code, &ee_arr[k].code, sizeof(buffer.code));		// ���������� � ����� ID-��� ������ 1W ����������.
                    eeprom_update_block (&buffer.flags, &ee_arr[k].flags, sizeof(buffer.flags));	//���������� � ����� ���� ������ 1W ����������.
                    sei();
                    str_clr(0);							// ������� 1-� ������
                    str_clr(1);							// ������� 2-� ������
                    send_string_to_LCD_XY (done, 0, 0); // ������� "���������"
                    press_time = 0;
                    delay_cnt = 0;						// ��������� ������ ����� �� ���������
                    stage = 10;
                }
                else if (press_time == QUICK)			//���� ��� ��������, ���� ������ ���������� ����������
                {
                    wait_timer = WAIT_LIM + 1;			//+1 ����� �������� (wait_timer==WAIT_LIM) �� ����� �� ��������� � main
                    for (; k < n; k++)
                    {
                        eeprom_read_block (&buffer.flags, &ee_arr[k].flags, sizeof(buffer.flags));// ��������� ���� 1W ���������� �� �����
                        if (!buffer.flags.active) //���� � ����� ���� ����������
                        {
                            eeprom_read_block (&buffer.name, &ee_arr[k].name, sizeof(buffer.name));
                            str_clr(1);								// ������� 2-� ������
                            send_arr_to_LCD_XY (buffer.name, 0, 1); // ������� ��� ����������� �������
                            wait_timer = 0;
                            press_time = 0;
                            _stage = stage;
                            return;
                        }
                    }
                    str_clr(0);	// ������� 1-� ������
                    str_clr(1);	// ������� 2-� ������
                    send_string_to_LCD_XY (add_to_end, 0, 0);	// ������� "��������?"
                    wait_timer = 0;
                    stage = 9;
                }
            }
            break;

        case 9:
            if (press_time == SLOW)
            {
                wait_timer = WAIT_LIM + 1;					//+1 ����� �������� (wait_timer==WAIT_LIM) �� ����� �� ��������� � main
                if (n < n_max)
                {
                    for (j=0; j < 8; j++)  {buffer.code[j] = data[j];}
                    location = dev_name;					// �������� ������� ���������� ������������� ASCII ��� ����� �� ���������
                    cli();
                    eeprom_update_byte((uint8_t*)dev_last_n, (eeprom_read_byte((uint8_t*)dev_last_n)+1)); //�������������� �����, ����������� ����� ���������� ������������ ����-��
                    sei();
                    for (j = 0; j < N_DIGS; j++)			//�������� ������ �������� ������ '0'
                    {
                        digits[j] = '0';
                    }
                    utoa_fast_div (eeprom_read_byte((uint8_t*)dev_last_n), digits); // ��������� ����� � ������� ����
                    for (j = 1; j < 4; j++)// ���������� 3 ��������� �������� digits � ��������� 3 �������� ������� ����� - ��� ����� ����������
                    {
                        dev_name[N_NAME-j] = digits[N_DIGS-j];
                    }
                    //���������� ������������ ����� ���������� � void*, �.�. utoa � strncpy �� ���� ������, ����� char*, � � ��� unsigned char*
                    strncpy((void*)buffer.name, (void*)location, sizeof buffer.name); // ���������� ASCII ��� ����� � ���� name ������
                    buffer.tmax = 30;
                    buffer.tmin = 6;
                    buffer.flags.active = 1; // ������ �������
                    buffer.flags.lt_alarm = 0;
                    buffer.flags.ht_alarm = 0;
                    buffer.flags.line_alarm =0;
                    buffer.flags.sms_T_0 = 0;
                    buffer.flags.sms_T_1 = 0;
                    buffer.flags.sms_T_2 = 0;
                    buffer.flags.relay =0;
                    cli();
                    eeprom_update_block (&buffer, &ee_arr[n], sizeof(buffer)); // ���������� � ����� ������ � ����� �������� �������� 1W ����������.
                    sei();
                    n++;
                    cli();
                    eeprom_update_byte ((uint8_t*)dev_qty, n);
                    sei();
                    str_clr(0);	// ������� 1-� ������
                    str_clr(1);	// ������� 2-� ������
                    send_string_to_LCD_XY (nev_dev_add, 0, 0);	// ������� "��������� �����" � 1-�  ������� 1 ������ ������
                    send_string_to_LCD_XY (element, 0, 1); 		// ������� "����������" � 1-�  ������� 2 ������ ������
                    delay_cnt = 0;								// ��������� ������ ����� �� ���������
                    press_time = 0;
                    stage = 10;
                }
                else
                {
                    send_string_to_LCD_XY (mem_full, 0, 0); 	// ������� "������ ���������"
                    press_time = 0;
                    delay_cnt = 0;								// ��������� ������ ����� �� ���������
                    stage = 10;
                }
            }
            else if (press_time == QUICK)
            {
                wait_timer = WAIT_LIM + 1;						//+1 ����� �������� (wait_timer==WAIT_LIM) �� ����� �� ��������� � main
                stage = 10;
                press_time = 0;
            }
            break;

        case 10:
            str_clr(0);										// ������� 1-� ������
            str_clr(1);										// ������� 2-� ������
            n = eeprom_read_byte((uint8_t*)dev_qty);
            if (!n)											// ���� � �������� ������� ��� �������, ������� � ������ ����
            {
                send_string_to_LCD_XY (absence, 0, 0);		// ������� "��� ��������"
                delay_cnt = 0;								// ��������� ������ ����� �� ���������
                stage = 0;
            }
            else
            {
                *qty = n;									// ���� ����� �� ����� ����, ��������� �������� n ��� ����� ������
                k = n = stage = _stage = *active = 0;		// ������ ����� �� ���� � �����������
                wait_timer = WAIT_LIM + 1;					// +1 ����� ��� ��������� ����� �� ��������� �������� if(wait_timer==WAIT_LIM)
            }
            break;
    }
    return;
}



//������ ������� ���������
int main (void)
{
    uint8_t i = 0;								// ������� "�� ���������", � �.�. ���������� ����� ���������� � ������
    unsigned char j = 0;						// ������� "�� �����������"
    unsigned char temperature[5];				// ������ ������ ����������� LB � HB
    uint16_t temp_int = 0; 						// ����� ����� �����������
    unsigned char temp_float = 0; 				// ������� ����� �����������
    int8_t temp_int_signed; 					// ����� ����� �-�� �� ������ ��� ��������� � ���������� ����������
    unsigned int temp; 							// ��������� ���������� ��� �������� �� ��������������� ���� � ������ ��� "-" �����������
    unsigned char temp_sign = 0; 				// ������� ����� �����������
    uint16_t Number = 0; 						// ���� ������ �������� �����������
    uint8_t Dig_1, Dig_2, Dig_3, sign = 0;		// ���������� ��� ����� �������� ���� ����������� � ������� �����
    uint8_t n = 0;								// ��� ���������� ���������� � ����� ���������
    uint8_t srch_done = 0;						// ������� ���������� ��������� ����������
    uint8_t digits[N_DIGS];						// ������ ��� �������� � utoa_fast_div ��� ���������� ��� ������ �������� ���� �����������
    int8_t last_t[n_max];						// ������ ����� ������ ��������� ���������� ���������� �� ������
    uint8_t next_i = 0;							// ����� ���������� ����������, ������� ��������� �� lcd
    uint8_t stage = 0;							// ��������� �������� ���������
    uint8_t lines_n = 0;						// ���������� ����� �����, ��������� � ������� Display
    uint8_t menu_act = 0;						// ���� ����� � ����
    t_all = last_t;								// ���������� ���������� ��������� �� ��������� ������

    cli();
    modem_rdy = 0;								//��� ������ �� ��������� ����� �� �����
    RESET.flag_1 = 0;							//��� ������ ������ ������������� ������
    msg_upld = 0;								//������ �������� � msg
    pause_cnt = PAUSE_CNT_MAX + 1;				//������ ����� ����� ������ � ������ �������� ����������
    ans_lim = PWR_UP_ANS_TIMER;					//������ ������ ��� ������������� ������
    time_gsm = GSM_LVL_TIME + 1;				//������� ���������� �������� ������ gsm ����������
    gsm_lvl_req = 1;							//��������� ������ ������ gsm ����� ������������� ������

    // LCD_COM_PORT_DDR |= (1<<RS)|(1<<EN); 	// ����� RS � EN ������, �������. ���� DAT � COM ������� �� ������ �����
    // LCD_COM_PORT = 0x00;						// ������ 0 � RS � EN, ����������������� ���� DAT � COM ������� �� ������ �����
    LCD_DAT_PORT_DDR = 0xFF;					// ���� ������ (� ������ � ������ ������) ��� - �� �����
    LCD_DAT_PORT = 0x00;						// ������ 0 � ���� ������ � ������ ���
    _delay_ms(200);								// �������� ���������� ���
    lcd_init();									// ������������� �������

    DDR_LED_PORT |= _BV(LED_PIN_NUM); 			// ������ 1 - ��� led ����������� �� �����
    DDR_SRC_PORT &= ~_BV(SRC_PIN_NUM);			// ������ 0 - ��� ������ ����������� �� ����
    SRC_PORT |= _BV(SRC_PIN_NUM);				// ������ 1 - �����. �������� � +
    DDR_RST_PORT &= ~_BV(RST_PIN_NUM);			// ����� ��� ������ ������ � 3-� ��������� - ��������� NRESET
    /*�����������������, ���� NRESET �� 0 ����� ������� ����, ��. �-���. NRESET*/
    //DDR_RST_PORT |= _BV(RST_PIN_NUM);			//����� ��� ������ ������ ����������� �� �����

    TIMSK0 |= (1<<TOIE0);						// ��������� ���������� �� ������������
    TCCR0B |= (1<<CS02) | (1<<CS00);			// �������� ������� 1024
    TCCR0B &= ~(1<<CS01);						// �������� ������� 1024
    TCCR0A &= ~(1<<WGM00) & ~(1<<WGM01);		// ����� ������ "normal"

    USART_Init(MYUBRR);							//�������� UART
    TX_IndexIN = TX_IndexOUT = RX_IndexIN = RX_IndexOUT = 0;// �������� �/������ �������� � �����������
    msg_clr();									// ������� msg
    ans_cnt = 0;								// ������ ������� �������� ������ ������ ����� ���������/������
    cmd_to_queue (AT_CMGD, TEXT_1_4, NULL, NULL, NULL, NULL);	// ��������� � ��������� ������ ������ "������� ��� ���"
    cmd_to_queue (AT_CPBF, NULL, NULL, QUOTES, USER_0, QUOTES);	// ��������� � ��������� ������ ������ ������ ������ ������ ������
    cmd_to_queue (AT_CPBF, NULL, NULL, QUOTES, BALANS, QUOTES);	// ��������� � ��������� ������ ������ ������ ������ ������ �������
    cmd_to_queue (AT_COPS, NULL, NULL, NULL, NULL, NULL);	// ��������� � ��������� ������ ������ ������ ��������� ��� ����������� ���� �������

    sei();										// ��������� ��������� ����������

    if (!(SRC_PIN & src_msk))					// ���� �� ���� 0, �.� ������ ������, ������ ������ ����� ��������� �� ���� � ���������� ���������� � �����
    {
        while (!press_time) {;}					// ���� ������ �� ��������, ���
        press_time = 0;							// �������� ��������� ������������� ����� �������, ����� �� ��������� ��������
        search_ID();							// ��������� ���������� ���� ID-�����
        srch_done = 1;							// ������� ����������� ������ ����������
    }

    if (!(n = eeprom_read_byte((uint8_t*)dev_qty)))//���� ������ ���������� �� ����, ��������� � ��������� n, ���� 0 - ��� � �������������, ����� �� �����, ���� n==0;
    {
        lcd_clr();								// ������� �������
        send_string_to_LCD_XY (absence, 0, 0);	// ������� "��� ��������"
        send_string_to_LCD_XY (init_srch, 0, 1);// ������� "��������� �����?"
        while (press_time != SLOW)				// ���� �� ����� �������� �������, ���
        {press_time = 0;}					//���������� �������� �������
        menu_act = 1;							//���������� �� ����, ������������� � ������� ����, �� ������ ����� ����������
        srch_done = 1;							//������� ������������ ���������� ���������
    }

    if (!srch_done)	// ���� ���������� �� �����������, ��������� � ������ ����� ���������� � ����� � ����������
    {
        lcd_clr();								// ������� �������
        send_string_to_LCD_XY (ow_check, 0, 0);	// ��������� "����� �����" � 1-� ������� ����� ������� 1 ������ ������
        for (uint8_t i = 0; i< n; i++)			//�������� ���������� � ������� ������� (�� 0)
        {
            eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer)); // ��������� �������� ��������� �� �����
            uint8_t pad_res = scratchpad_rd(); // ��������� ������ ��������
            if (!pad_res)
            {
                buffer.flags.active = 1;		// ������ ����� �����������
            }
            else if (pad_res == 1)				//���� ������ ������ �� �������, ����.���� ����� == FF
            {
                buffer.flags.active = 0;		//����� ����� �����������
                lcd_clr();						// ������� �������
                send_string_to_LCD_XY (no_answer_n, 0, 0); // ������� "��� ������ " � 1-� ������� ����� ������� 1 ������ ������
                send_arr_to_LCD_XY (buffer.name, 0, 1);
                _delay_ms(2000);
                lcd_clr();						// ������� �������
            }
            else if (pad_res == 2)				//������ ������� i-�� ����., ���� CRC �� ��, ������� ��������� �� ������
            {
                buffer.flags.active = 0;		//����� ����� �����������
                lcd_clr();						// ������� �������
                send_string_to_LCD_XY (scratch_err, 0, 0); // ������� "��.CRC-����. " � 1-� ������� ����� ������� 1 ������ ������
                send_arr_to_LCD_XY (buffer.name, 0, 1);
                _delay_ms(2000);
                lcd_clr();						// ������� �������
            }
            cli();
            eeprom_update_block (&buffer, &ee_arr[i], sizeof(buffer)); // ���������� � ����� �������� �������� 1W ���������� � ���������� ������ ������.
            sei();
        }
        lcd_clr();								// ������� �������
    }

    while(1)
    {
        if (!menu_act)	//���� ������������ �� ����� � ����, �������� ����� � ���������.
        {
            //n = eeprom_read_byte((uint8_t*)dev_qty;
            eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer));	// ��������� �������� ��������� �� �����
            if (buffer.flags.active)									// ���� ���� ����������� ������, ����������� ����������� (���� �������, �� ������� � ��������� ����� ???.?)
            {
                init_device();		//������� ������ � �����������
                send_command(0x55);	//������� ������������
                // ����� ��������� ��� ���������� � �������� ����������
                for (j = 0; j < 8 ; j++)
                {
                    send_command (buffer.code[j]);
                }

                send_command (0x44);			// ������� ��������������
                while (!read_data()) ;			// ����������� ���� ���� �� ����� �� ����������� 1 - �������������� ���������
                init_device();					// ������� ������ � �����������
                send_command(0x55);				// ������� ������������
                for (j = 0; j < 8 ; j++)   		// ����� �������� ����� ����������, � �������� ����� ����������
                {
                    send_command (buffer.code[j]);
                }
                send_command (0xBE);			// ������� ������ ������
                for (j = 0; j < 5; j++) 		// ��������� ������ ��� ����� ����������� � ����.����
                {
                    unsigned char i;			// ��������� ���������� ��� ����������� �����
                    unsigned char data_temp = 0x00;
                    for (i = 0; i < 8; i++)
                    {
                        data_temp >>= 1;
                        if (read_data()) 		// ���� 1, �� �������������  ������� ��� 1
                            data_temp |= 0x80;
                    }
                    temperature[j] = data_temp;
                }
                init_device();						// ����� �������� ��� ����������� �������� ������
                // ����������� ���������� � �������� �����������
                if (!(temperature[4] == 0xFF))		// ���� ����.���� ������ ����� ==FF, �� ������ ��������.
                {
                    if (buffer.flags.line_alarm) 	// ���������� ���� ����������, ���� �� ��� ������ � �����-�� ���������� �����, � ������ ���������
                    {
                        buffer.flags.line_alarm = 0;
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags)); //���������� ����� � �����
                        sei();
                    }
                    if ((temperature[1]&0b10000000) == 0)	// �������� �� ��������������� �����������
                        temp_sign = 0; 						// ������ ���� �� ���
                    else 									// ��������� �� ���.���� � ������
                    {
                        temp = ((unsigned int)temperature[1]<<8) | temperature[0];
                        temp = ~temp + 1;
                        temperature[0] = temp;
                        temperature[1] = temp >> 8;
                        temp_sign = 1;						//������ ����� �� ���
                    }

                    temp_int = ((temperature[1]&0b00000111)<<4)|(temperature[0]>>4);	// �������  ����� ����. �����������
                    temp_float = (temperature[0]&0b00001111); 							// �������� � ������� ������� ����� ������� �����

                    temp_int_signed = (!temp_sign ? temp_int : ~(temp_sign - 1));		// ��������� ���� � temp_int ��� ��������� � tmin, ��� �����.����� ������� � ���.���
                    last_t[i] = temp_int_signed;										// ������� � ������ ��������� ���������� ��������
                    if ((temp_int_signed < buffer.tmin)&&(!buffer.flags.lt_alarm))		// ���� � ���� �������, � ���� �� ����������
                    {
                        buffer.flags.lt_alarm = 1; 					// ������ ����
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); // ���������� ����� � �����
                        sei();
                        if ((buffer.flags.sms_T_0)||(buffer.flags.sms_T_1)||(buffer.flags.sms_T_2)) // ���� ���������� ���� �������� sms
                        {
                            sms_buff.sms_type = T_LOW;
                            sms_buff.dev_num = i;
                            sms_buff.param = buffer.tmin;
                            if (buffer.flags.sms_T_0)					// ���� ���� �������� ������ ����������
                            {
                                sms_buff.person = ADMIN;				// ��� ������
                                out_to_queue (&sms_buff);
                            }
                            if (buffer.flags.sms_T_1)					// ���� ���� �������� �����1 ����������
                            {
                                sms_buff.person = ABNT_1;				// ��� �����1
                                out_to_queue (&sms_buff);
                            }
                            if (buffer.flags.sms_T_2)					// ���� ���� �������� �����2 ����������
                            {
                                sms_buff.person = ABNT_2;				// ��� �����2
                                out_to_queue (&sms_buff);
                            }
                            /* lcd_clr();
                            send_arr_to_LCD_XY (buffer.name, 0, 0);
                            send_string_to_LCD (blank);
                            send_string_to_LCD (t_low);
                            lcd_dat (((buffer.tmin & 0b10000000) ? '-' : '+'));
                            send_arr_to_LCD (utoa_fast_div (((buffer.tmin & 0b10000000) ? ((~buffer.tmin) + 1) : buffer.tmin), digits)); */
                        }
                    }
                    else if ((temp_int_signed > buffer.tmax)&&(!buffer.flags.ht_alarm))	//���� � ���� �������, � ���� �� ����������
                    {
                        buffer.flags.ht_alarm = 1; 					// ������ ����
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                        sei();
                        if (buffer.flags.sms_T_0) 					//���� ���������� ���� �������� sms, ���������� ��� � �� lcd
                        {
                            sms_buff.sms_type = T_HIGH;
                            sms_buff.dev_num = i;
                            sms_buff.param = buffer.tmax;
                            if (buffer.flags.sms_T_0)					// ���� ���� �������� ������ ����������
                            {
                                sms_buff.person = ADMIN;				// ��� ������
                                out_to_queue (&sms_buff);
                            }
                            if (buffer.flags.sms_T_1)					// ���� ���� �������� �����1 ����������
                            {
                                sms_buff.person = ABNT_1;				// ��� �����1
                                out_to_queue (&sms_buff);
                            }
                            if (buffer.flags.sms_T_2)					// ���� ���� �������� �����2 ����������
                            {
                                sms_buff.person = ABNT_2;				// ��� �����2
                                out_to_queue (&sms_buff);
                            }
                            /* lcd_clr();
                            send_arr_to_LCD_XY (buffer.name, 0, 0);
                            send_string_to_LCD (blank);
                            send_string_to_LCD (t_high);
                            lcd_dat (((buffer.tmin & 0b10000000) ? '-' : '+'));
                            send_arr_to_LCD (utoa_fast_div (((buffer.tmax & 0b10000000) ? ((~buffer.tmax) + 1) : buffer.tmax), digits)); */
                        }
                    }
                    else if ((temp_int_signed > buffer.tmin)&&(buffer.flags.lt_alarm))				//���� � ���� (T min), � ���� ����������
                    {
                        buffer.flags.lt_alarm = 0; 													// ������� ����
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                        sei();
                    }
                    else if ((temp_int_signed < buffer.tmax)&&(buffer.flags.ht_alarm))				//���� � ���� (T max) �������, � ���� ����������
                    {
                        buffer.flags.ht_alarm = 0; 													// ������� ����
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                        sei();
                    }
                }
                else 											//���� ������ ����� �������� (�� ������� � ��������� ����� ---.-)
                {
                    if (!buffer.flags.line_alarm)				//��������� ���� ����������, ���� �� �� ��� ������ � �����-�� ���������� �����
                    {
                        buffer.flags.line_alarm = 1;
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags)); //���������� ����� � �����
                        sei();
                        lcd_clr();
                        send_arr_to_LCD_XY(buffer.name, 0, 0); 	//�������� ������������ �� ������
                        send_string_to_LCD (blank);
                        send_string_to_LCD (crash);
                        sms_buff.sms_type = FAIL;				//���������� � �������� ��������� ��� ���
                        sms_buff.dev_num = i;					//���������� � �������� ��������� ����� ���������� ����������
                        sms_buff.param = 0;						//�������� ��������, ����� �� �����
                        sms_buff.ptr = NULL;
                        sms_buff.person = ADMIN;				// ���� ��� ������ ������
                        out_to_queue (&sms_buff);				//������ ������ �� �������� ���
                    }
                }
            }

            switch (stage)	//������� ������ �� lcd
            {
                case 0:
                    if (i == next_i)							// ���� ������� ����� ����������
                    {
                        if (!buffer.flags.active)				//���� ������ ���� ����������
                        {
                            Dig_1 = '-';
                            Dig_2 = '-';
                            Dig_3 = '-';
                            sign = '-';
                        }
                        else if (buffer.flags.line_alarm)		//���� ���� ����������� �������
                        {
                            Dig_1 = '?';
                            Dig_2 = '?';
                            Dig_3 = '?';
                            sign = '?';
                        }
                        else									//���� ������ �������
                        {
                            temp_int = (temp_int << 3) + (temp_int << 1); 		//�������� ����� ����� �� 10, ����� 0 � �������� ��� ���������� �����
                            temp_float = ((temp_float << 2) + temp_float) >> 3; //�������� ������� ����� �� (0,0625 * 10), ����� �� 5/8, �������� ���������� ����� *10
                            Number = temp_float + temp_int;						// �� ����� ������ ����� ������� �����, ����������� �� 10
                            /*		����������� ����� � �����, ����� �� � ���� �������� ����		*/
                            for (j = 0; j < N_DIGS; j++) 						//�������� ������ �������� ������ '0', �.�. ������� ���� �����
                            {
                                digits[j] = '0';
                            }
                            j = 0;							//�������� �������
                            utoa_fast_div (Number, digits); //����������� ����� � ������� ���� � ���������� � ������ digits
                            Dig_1 = digits[1];
                            Dig_2 = digits[2];
                            Dig_3 = digits[3];
                            switch (temp_sign)
                            {
                                case 0 : sign = '+';
                                    break;
                                case 1 : sign = '-';
                                    break;
                            }
                        }

                        Frame (Dig_1, Dig_2, Dig_3, sign, i, n);//������� ������ � ��������� �����

                        if (i == (n-1))							//���� ������ ���������
                        {
                            next_i = 0;							//�������� ����� ���� ���������
                            stage = 1;							//��������� � ������ ���������
                            if (!(i & 0x01)	)					//���� ������ �������
                            {
                                lines_n = 0;					//����� ����� ������� ������
                            }
                            else								//���� ������ ������
                            {
                                lines_n = 1;					//����� ���� �����
                            }
                        }
                        else									//���� ������ �� ���������
                        {
                            next_i += 1;						//����������� ����� ���������� ���������� ����-��
                            if (i & 0x01)						//���� ������ ������
                            {
                                lines_n = 1;					//����� ���� �����
                                stage = 1;						//��������� � ������ ���������
                            }
                        }
                    }
                    break;
                case 1:
                    if (delay_cnt >= IND_PAUSE)	// ���� ����� �� ��������� �������
                    {
                        str_clr(0);				// ������� 1-� ������
                        str_clr(1);				// ������� 2-� ������
                        Display (lines_n);		// ������� ���� �� lcd
                        delay_cnt = 0;			// ������������� ������ ���������
                        stage = 0;				// ������� � ������ ������������ �����
                    }
                    break;
            }	//����� �������� ���������

            i += 1;								//����������� ����� ����������
            if (i > (n - 1)){i = 0;}			//���� ����� ����� �� ������� �������������, ��������
        }	//����� ������ ���������� ����������

        if (!menu_act)						//���� �� ���� ���������,...
        {
            if (press_time == SLOW)			//... � ��������� ������� �������
            {
                menu_act = 1;				// ��������� �� ���� (���� ������ ����� ����������)
                press_time = 0;				// �������� ���� �������
                delay_cnt = IND_PAUSE;		// ����� ���� � ���� ����� ����� �������, ����� ����� ����� ��������� ����� ���������
                i = next_i = stage = 0;		// ����� ����� ������ �� ���� ����� � ��������� �������� � ������� ���������� (�� 0)
            }
            // else {press_time = 0;}			//���� ������� ��������, ���� ������ �� ������? �������� ���� �������
        }

        menu (&n, &menu_act);				//�������� �� ����, ������ �������� menu_act, ���� 0 - �����.

        if (msg_upld)						//���� ������ ���� ������� ����� �������
        {
            uint8_t pars_res = 0;			//��������� ������ �������
            uint8_t num = RX_IndexNumber(); //������� ���� � ��������� ������
            RX_Ring_to_Str(msg, num);		//��������� �� ������ � msg
            msg_upld = 0;					//�������� ���� ������� ����� �������
            //byte_to_TX_Ring(0x11);			//XON
            //UCSR0B |= (1<<UDRIE0);			//���������� ���������� �� ����������� UDR �����������
            USART_TXD (0x11);					//XON ������ ��������
            if (modem_rdy)					//������ ��������� ������ ��� �������� ������
            {
                pars_res = parser();		//��������� ����� � msg
#ifdef DEBUG
                lcd_dat_XY(pars_res, 7, 1); //������� ������� ������������ ������ ��������
#endif
            }
        }

        if (!modem_rdy)		//���� ����� �� �����(���-�� ����������� ����� � ���� ����)
        {
            if (!RESET.flag_1)	//������ ����� ����� ������
            {
                if (strstr_P((const char*)msg, (PGM_P) CALL_RDY) != NULL)//���� � msg ���� Ready\r\n
                {
                    msg_clr();
                    string_to_TX_Ring (AT_BUSY);
                    string_to_TX_Ring (CRLF);	//���������� AT+GSMBUSY=1
                    ans_lim = 120;				//������ ������ 1�
                    UCSR0B |= (1<<UDRIE0);		//���������� ���������� �� ����������� UDR �����������
                    ans_cnt = 0;				//��������� ������ �������� ������ �� ������
                    RESET.flag_1 = 1;			//��������� ���� "������ �������� ������� ��������� � �����"
#ifdef DEBUG
                    lcd_dat_XY((0x35), 4, 1);	//����� ������� 5 �� lcd
					_delay_ms(1500);
#endif
                }
#ifdef DEBUG
                    else if (ans_cnt < ans_lim)
				{
					lcd_dat_XY('i', 5, 1);		//����� ������� i �� lcd
					_delay_us(50);
				}
#endif
                else if (ans_cnt >= ans_lim)
                {
#ifdef DEBUG
                    lcd_dat_XY((0x36), 6, 1);	//����� ������� 6 �� lcd
					_delay_us(50);
#endif
                    NRESET();	//����� ������
                }
            }
            else if(RESET.flag_1)	//��� �� � ����� �� AT+GSMBUSY=1
            {
                if (strstr_P((const char*)msg, (PGM_P) ANS_OK) != NULL)	//���� � msg ���� "...OK"
                {
                    msg_clr();
                    modem_rdy = 1;	//����� �����
#ifdef DEBUG
                    lcd_dat_XY((0x39), 9, 1);	//����� ������� 9 �� lcd
#endif
                }
#ifdef DEBUG
                    else if (ans_cnt < ans_lim)
				{
					lcd_dat_XY('h', 5, 1);		//����� ������� h �� lcd
					//_delay_us(50);
				}
#endif
                else if (ans_cnt >= ans_lim)
                {
#ifdef DEBUG
                    lcd_dat_XY((0x37), 7, 1);	//����� ������� 7 �� lcd
#endif
                    NRESET();	//����� ������
                }
            }
        }

        else	//���� ����� �����, ��� � ���� �������� �����
        {
            if (queue_T != 	queue_H)
            {
                handl_res = HANDLERS[queue_T]();
#ifdef DEBUG
                lcd_dat_XY(handl_res, 8, 1); 			//����� �� lcd ����������� �� ������������
#endif
            }

            if (gsm_lvl_req)							//���� � ���������� ����������� ���� �������� ������� ������
            {
                cmd_to_queue (AT_CSQ, NULL, NULL, NULL, NULL, NULL);//������ ������
                gsm_lvl_req = 0;						//�������� ����
                time_gsm = 0;							//��������� ������ �������� ������ gsm � ����������
            }

        }
    }	//����� ������������ �����
}	//����� ������� main
