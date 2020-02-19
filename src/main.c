#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <string.h>
// #include <math.h> �����������������, ���� ����� �-� modf � �-�� �������������� ��� � ����� ��� �������� � ���

//���� define ��� LCD
#define RS PORTD3						// ����� ������ �����, �� �������� ��������� ������� RS � ���
#define EN PORTD2						// ����� ������ �����, �� �������� ��������� ������� EN � ���
#define LCD_COM_PORT PORTD				// ���� ��� ������ ������ � ��� (������� � ������ - ����� ���� �� �� ����� �����!!!)
//#define LCD_COM_PORT_DDR DDRD 		// ������� ����������� ������ � �����, ���� ���������� ����� ������ ���, ��.����
#define LCD_DAT_PORT PORTD				// ���� �������� ������ (� ������ � ������ ������)� ���
#define LCD_DAT_PORT_DDR DDRD			// ������� ����������� ������ �����, ���� ��������� ��� ������� ������ (� ������ � ������ ������)
/* ���-����� ��������� ������ � �������� �� � ���� LCD_DAT_PORT ������������ �������������� ������� �������, � �������
��������� ���. � ������ ������ ������������ ������ 4-7 �����. */

//���� define ��� OW
#define DDR_OW_PORT DDRB				// ������� ����������� ������ �����, � ������ �� ������� �������� ���������� ����� 1-Wire
#define OW_PORT PORTB					// ����, � ������ �� ������� �������� ���������� ����� 1-Wire
#define OW_PIN PINB						// ������� ����� ������ ����� 1-Wire, � ������ �� ������� �������� ���������� ��� �����
#define OW_PIN_NUM 0					// ����� PIN, � �������� ���������� ����� 1-Wire, ��� ������� _BV()
#define bit_msk (1<<OW_PIN_NUM) 		// ������� ����� ��� �������� ������� �� ����� 1-Wire �� ��������������� ����

//���� ������ define
#define N_DIGS 5						// ������ ������� ���� ��� �������������� ����� � ������ ��� LCD, ������������ ������������ ������������� ���������� �� LCD ����� +1 ��� ������������ 0
#define N_NAME 8						//������ ������� ��� ����� ����������
#define EEP_MEM 1024					// ����� ����� ��� ���������� ������ ��������� � ������������� ������, ����� ��� ����������� ������������� ���������� ��������� � �����
#define IND_PAUSE 120 					// ����� �� ��������� ���������� �����, 2�
#define WAIT_LIM 900 					//����� ��� �������� �������� ������������, 15�

//���� define ��� ������������� ������
#define SRC_PORT PORTB					// ����, � ������ �� ������� �������� ���������� ������ ������ ������ (����� ���� ���� � OW line)
#define DDR_SRC_PORT DDRB				// ������� ����������� ������ �����, � ������ �� ������� �������� ���������� ������ ������ ������ (����� ���� ���� � OW line)
#define SRC_PIN PINB					// ������� ����� ��������� ������
#define SRC_PIN_NUM 1					// ����� PIN, � �������� ���������� ������, ��� ������� _BV()
#define src_msk (1<<SRC_PIN_NUM)		// ������� ����� ��� �������� ������� �� ������ �� ��������������� ����

//���� define ��� ������ ������
#define CNT_QUICK 5						// ���������� ������������ �� �����������, ���� ������ - ������ �����
#define CNT_SLOW 15						// ���������� ������������ �� ������� �������
#define QUICK 1							// ������������ ��������� �������
#define SLOW 2							// ������������ �������� �������
#define RELEASED 0						// ��������� ������ - ��������
#define PRESSED 1						// ��������� ������ - ������

// ���� define ��� ���������� ��������� gsm ������
#define LED_PORT PORTB					// ����, � ������ �� ������� �������� ��������� ��������� (����� ���� ���� � OW line)
#define DDR_LED_PORT DDRB				// ������� ����������� ������ �����, � ������ �� ������� �������� ��������� ��������� (����� ���� ���� � OW line)
#define LED_PIN_NUM 5					// ����� PIN, � �������� ��������� ���������, ��� ������� _BV()

// ���� ��������� ��������� �� ����
unsigned char absence[] PROGMEM = "��� ��������";				// ������ �� ����� ������ ������������� �������� ����� ������ � �������� ���������� �������
unsigned char no_answer[] PROGMEM = "��� ������ ����.";			// ������ �� ������ ���������� ������� ����� ������ ������ ���� ����-������
unsigned char present_n[] PROGMEM = "������� ����. ";			// ���������� ���������, ���������� ��� ��������� ���������� �������
unsigned char dev_excess[] PROGMEM = "����� ��������";			// ������ � �������� ��������� ���������� �������, ���� ���������� �������� �������� 50
unsigned char error[] PROGMEM = "������ CRC-ID ";				// ������ �� ����� �������� CRC ID ����� ��������� ���������� �������, ��������� � �������,
unsigned char init_n[] PROGMEM = "�������.����. ";				// ���������� ��������, ��������� �������� CRC ����� ��������� ���������� �������
unsigned char init_srch[] PROGMEM = "��������� �����?";			// ������ ��� ���������� � ����� ������ ��������, ���� �� ����������� ��������� ���������� �������
unsigned char scratch_err[] PROGMEM = "��.CRC-����. ";			// ������ ��� �������� CRC ������, ����������� �� ��������, ��������� � �������
unsigned char no_answer_n[] PROGMEM = "��� ������ ";			// ������ �� ����� ������ ������ �������� ��� ���������� � main - ���� ������ �� �������, �� ����.���� == FF, ��������� ��� �������
unsigned char ow_check[] PROGMEM = "����� �����";				// ��������� � main ��� ���������� ����� �� ����� ����������
unsigned char correction[] PROGMEM = "�������������";			// ��������� � add_ID ��� �����
unsigned char subst_add[] PROGMEM = "������/������-�";			// ��������� � add_ID ��� ����� � ���� ������/��������
unsigned char total_qty[] PROGMEM = "����� ���������";			// ��������� � add_ID ����� ������� ������
unsigned char plug_in[] PROGMEM = "���������� ����.";			// ��������� � add_ID ����� ������� ������
unsigned char press_btn[] PROGMEM = "� ������� ������";			// ��������� � add_ID ����� ������� ������
unsigned char mem_full[]PROGMEM = "������ ���������";			// ��������� � add_ID, ���� ���������� �������� ��� 50, � ���������� ���
unsigned char new_dev_fnd[] PROGMEM = "������� �����";			// ������� ������ ��������� � add_ID, ���� ������� �����
unsigned char nev_dev_add[] PROGMEM = "��������� �����";		// ������� ������ ��������� � add_ID � ����� ����� ����������
unsigned char element[] PROGMEM = "����������";					// ������ ������ ��������� � add_ID, ���� ������� ��� ��������� �����
unsigned char substitute[] PROGMEM = "��������?";				// ��������� � add_ID � ����� ������, ���� ����� ���������� ����-��, ��������� ��� �������
unsigned char add_to_end[] PROGMEM = "��������?";				// ��������� � add_ID � ������ ����� ����������
unsigned char done[] PROGMEM = "������ ���������";				// ��������� � add_ID ����� ������� ������ �����������
unsigned char no_new[] PROGMEM = "����� ����-� ���";			// ��������� � add_ID ���� ������ ���� ���� ������ ������ ID
unsigned char delete[] PROGMEM = "�������?";					// ��������� - ������ ����� ���� �������������
unsigned char del_done[] PROGMEM = "�������";					// ��������� �� ����� �������� ����������
unsigned char t_error[] PROGMEM = "�������� ����-��";			// ��������� ��� ������� �������� ������������ ����������� � ������
unsigned char com_error[] PROGMEM = "�������� �������";			// ��������� ��� ������ � ������ �������
unsigned char name_error_ren[] PROGMEM = "�������� ��� REN";	// ��������� ��� ���������� ����� � ������ �������
unsigned char name_error_al[] PROGMEM = "�������� ��� AL";		// ��������� ��� ���������� ����� � ������ �������
unsigned char name_error_sms[] PROGMEM = "�������� ��� SMS";	// ��������� ��� ���������� ����� � ������ �������
unsigned char t_min[] PROGMEM = "Tmin";							// ����� ������ ��� ������������� ��������� ������������ ������� �����������
unsigned char t_max[] PROGMEM = "Tmax";							// ����� ������ ��� ������������� ��������� ������������� ������ �����������
unsigned char t_low[] PROGMEM = "T<";							// ����� ������ ��� ���������� � �������� ����������� ���� ������������ ������
unsigned char t_high[] PROGMEM = "T>";							// ����� ������ ��� ���������� � ��������� ����������� ���� ������������� ������
unsigned char sms_send []PROGMEM = "SMS";						// ����� ������ ��� ������������� ��������� sms � ������ ����������� �� �������
unsigned char on []PROGMEM = "���.";
unsigned char off []PROGMEM = "���.";
unsigned char blank []PROGMEM = " ";
unsigned char crash []PROGMEM = "������!";

// ���� ���������� ���������� � ��������
typedef struct //������� ���� ��� ������
{
    uint8_t active : 1;		// ������� ���������� �� �����
    uint8_t lt_alarm : 1;	// ������ ����� ����������� ������� ����
    uint8_t ht_alarm : 1;	// ������� ����� ����������� ��������
    uint8_t line_alarm : 1;	// ������� ���������� � �������� ���������
    uint8_t sms_T : 1;		// ��������� sms ��� � < tmin ��� � > tmax
    uint8_t relay_1 : 1;	// ���� 1
    uint8_t relay_2 : 1;	// ���� 2
    uint8_t reserved : 1;	// ���������
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



volatile uint8_t flash_cnt;		// ������� ���������� ��� ��������� ������ gsm ��������� �����
volatile uint8_t flash_num;		// ����� ��������� �������
volatile uint8_t pause_num;		// ����� ��������� �������� ���� ����� ������� �������
const uint8_t gsm_lvl = 6;		// ������� gsm �������, ���� const


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
        buffer.flags.sms_T = 0;
        buffer.flags.relay_1 = 0;
        buffer.flags.relay_2 = 0;
        buffer.flags.reserved =0;
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

void menu(uint8_t* qty, uint8_t* active) // �� ���� ��� ��������/����������/������� ���������
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
                    buffer.flags.sms_T = 0;
                    buffer.flags.relay_1 = 0;
                    buffer.flags.relay_2 = 0;
                    buffer.flags.reserved =0;
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

ISR(TIMER0_OVF_vect) //���������� ���������� ������� 0
{
    int_cnt++; //������� ����������
    if (int_cnt == 2) //��� ������������ �������� ������� 1024 ����� ���.�������� �� 2, ����� ����� 30��
    {
        BTN_SCAN (); //��������� ������
        int_cnt = 0; //�������� ����� ������
    }

    if (delay_cnt < IND_PAUSE)
    {delay_cnt++;}//������� �������� 2c ��� ���������

    if (wait_timer < WAIT_LIM)
    {wait_timer++;}//������� ����� 15c �� �������� ������������

    flash_cnt++; //������� ��� ������� ��������� ������ �������
    if (flash_cnt == 10)
    {
        flash_cnt = 0;
        if (!pause_num)
        {
            if (flash_num++ < gsm_lvl)
                PORTB ^= _BV(LED_PIN_NUM);
            else
            {
                flash_num = 0;
                pause_num++;
                PORTB &= ~_BV(LED_PIN_NUM);
            }
        }
        else
        {
            if (pause_num++ >= 16)
                pause_num = 0;
        }
    }
    //time_gsm++; //������� ���������� 30c �������� GSM, ������������� � main
}

//������ ������� ���������
int main (void)
{
    uint8_t i = 0;								// ������� "�� ���������", � �.�. ���������� ����� ���������� � ������
    unsigned char j = 0;						// ������� "�� �����������"
    unsigned char temperature[5];				// ������ ������ ����������� LB � HB
    uint16_t temp_int = 0; 							// ����� ����� �����������
    unsigned char temp_float = 0; 					// ������� ����� �����������
    int8_t temp_int_signed; 					// ����� ����� �-�� �� ������ ��� ��������� � ���������� ����������
    unsigned int temp; 							// ��������� ���������� ��� �������� �� ��������������� ���� � ������ ��� "-" �����������
    unsigned char temp_sign = 0; 					// ������� ����� �����������
    uint16_t Number = 0; 						// ���� ������ �������� �����������
    uint8_t Dig_1, Dig_2, Dig_3, sign = 0;		// ���������� ��� ����� �������� ���� ����������� � ������� �����
    uint8_t n = 0;								// ��� ���������� ���������� � ����� ���������
    uint8_t srch_done = 0;						// ������� ���������� ��������� ����������
    uint8_t digits[N_DIGS];						// ������ ��� �������� � utoa_fast_div ��� ���������� ��� ������ �������� ���� �����������
    int8_t last_t[n_max];						// ������ ����� ������ ��������� ���������� ���������� �� ������
    uint8_t next_i = 0;							// ����� ���������� ����������, ������� ��������� �� lcd
    uint8_t stage = 0;							// ��������� �������� ���������
    uint8_t lines_n = 0;							// ���������� ����� �����, ��������� � ������� Display
    uint8_t menu_act = 0;						// ���� ����� � ����

    // LCD_COM_PORT_DDR |= (1<<RS)|(1<<EN); 	// ����� RS � EN ������, �������. ���� DAT � COM ������� �� ������ �����
    // LCD_COM_PORT = 0x00;						// ������ 0 � RS � EN, ����������������� ���� DAT � COM ������� �� ������ �����
    LCD_DAT_PORT_DDR = 0xFF;					// ���� ������ (� ������ � ������ ������) ��� - �� �����
    LCD_DAT_PORT = 0x00;						// ������ 0 � ���� ������ � ������ ���
    _delay_ms(200);								// �������� ���������� ���
    lcd_init();									// ������������� �������

    DDR_LED_PORT |= _BV(LED_PIN_NUM); 			// ������ 1 - ��� led ����������� �� �����

    TIMSK0 |= (1<<TOIE0);						// ��������� ���������� �� ������������
    TCCR0B |= (1<<CS02) | (1<<CS00);			// �������� ������� 1024
    TCCR0B &= ~(1<<CS01);						// �������� ������� 1024
    TCCR0A &= ~(1<<WGM00) & ~(1<<WGM01);		// ����� ������ "normal"
    sei();

    DDR_SRC_PORT &= ~_BV(SRC_PIN_NUM);			// ������ 0 - ��� ������ ����������� �� ����
    SRC_PORT |= _BV(SRC_PIN_NUM);				// ������ 1 - �����. �������� � +

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
                        buffer.flags.lt_alarm = 1; // ������ ����
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); // ���������� ����� � �����
                        sei();
                        if (buffer.flags.sms_T) // ���� ���������� ���� �������� sms, ���������� �� lcd
                        {
                            lcd_clr();
                            send_arr_to_LCD_XY (buffer.name, 0, 0);
                            send_string_to_LCD (blank);
                            send_string_to_LCD (t_low);
                            lcd_dat (((buffer.tmin & 0b10000000) ? '-' : '+'));
                            send_arr_to_LCD (utoa_fast_div (((buffer.tmin & 0b10000000) ? ((~buffer.tmin) + 1) : buffer.tmin), digits));
                            _delay_ms(2000);
                        }
                    }
                    else if ((temp_int_signed > buffer.tmax)&&(!buffer.flags.ht_alarm))				//���� � ���� �������, � ���� �� ����������
                    {
                        buffer.flags.ht_alarm = 1; // ������ ����
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                        sei();
                        if (buffer.flags.sms_T) //���� ���������� ���� �������� sms, ���������� �� lcd
                        {
                            lcd_clr();
                            send_arr_to_LCD_XY (buffer.name, 0, 0);
                            send_string_to_LCD (blank);
                            send_string_to_LCD (t_high);
                            lcd_dat (((buffer.tmin & 0b10000000) ? '-' : '+'));
                            send_arr_to_LCD (utoa_fast_div (((buffer.tmax & 0b10000000) ? ((~buffer.tmax) + 1) : buffer.tmax), digits));
                            _delay_ms(2000);
                        }
                    }
                    else if ((temp_int_signed > buffer.tmin)&&(buffer.flags.lt_alarm))				//���� � ���� (T min), � ���� ����������
                    {
                        buffer.flags.lt_alarm = 0; // ������� ����
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                        sei();
                    }
                    else if ((temp_int_signed < buffer.tmax)&&(buffer.flags.ht_alarm))				//���� � ���� (T max) �������, � ���� ����������
                    {
                        buffer.flags.ht_alarm = 0; // ������� ����
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
                        _delay_ms(1500);
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
            else {press_time = 0;}			//���� ������� ��������, ���� ������ �� ������? �������� ���� �������
        }

        menu (&n, &menu_act);				//�������� �� ����, ������ �������� menu_act, ���� 0 - �����.
    }	//����� ������������ �����
}	//����� �������
