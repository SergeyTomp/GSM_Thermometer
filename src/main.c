#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> //��� sei() � cli()
#include <stdlib.h> // ��� abort()
#include <avr/pgmspace.h> //�����������������, ���� ������� ������ � ������ �� ����
#include <avr/eeprom.h> //��� ������ � ������
#include <string.h> //��� strncpy
// #include <math.h> �����������������, ���� ����� �-� modf � �-�� �������������� ��� � ����� ��� �������� � ���


//���� define ��� LCD
#define RS PORTD3 //  ����� ������ �����, �� �������� ��������� ������� RS � ���
#define EN PORTD2 //  ����� ������ �����, �� �������� ��������� ������� EN � ���
#define LCD_COM_PORT PORTD // ���� ��� ������ ������ � ��� (������� � ������ - ����� ���� �� �� ����� �����!!!)
//#define LCD_COM_PORT_DDR DDRD // ������� ����������� ������ � �����, ���� ���������� ����� ������ ���, ��.����
#define LCD_DAT_PORT PORTD // ���� �������� ������ (� ������ � ������ ������)� ���
#define LCD_DAT_PORT_DDR DDRD // ������� ����������� ������ �����, ���� ��������� ��� ������� ������ (� ������ � ������ ������)
/* ���-����� ��������� ������ � �������� �� � ���� LCD_DAT_PORT ������������ �������������� ������� �������, � �������
��������� ���. � ������ ������ ������������ ������ 4-7 �����. */

//���� define ��� OW
#define DDR_OW_PORT DDRB // ������� ����������� ������ �����, � ������ �� ������� �������� ���������� ����� 1-Wire
#define OW_PORT PORTB // ����, � ������ �� ������� �������� ���������� ����� 1-Wire
#define OW_PIN PINB // ������� ����� ������ ����� 1-Wire, � ������ �� ������� �������� ���������� ��� �����
#define OW_PIN_NUM 0 // ����� PIN, � �������� ���������� ����� 1-Wire, ��� ������� _BV()
#define bit_msk (1<<OW_PIN_NUM) // ������� ����� ��� �������� ������� �� ����� 1-Wire �� ��������������� ����

//���� ������ define
#define N_DIGS 5 // ������ ������� ���� ��� �������������� ����� � ������ ��� LCD, ������������ ������������ ������������� ���������� �� LCD ����� +1 ��� ������������ 0
#define N_NAME 8 //������ ������� ��� ����� ����������
#define EEP_MEM 1024 // ����� ����� ��� ���������� ������ ��������� � ������������� ������, ����� ��� ����������� ������������� ���������� ��������� � �����

//���� define ��� ������������� ������
#define SRC_PORT PORTB //����, � ������ �� ������� �������� ���������� ������ ������ ������ (����� ���� ���� � OW line)
#define DDR_SRC_PORT DDRB // ������� ����������� ������ �����, � ������ �� ������� �������� ���������� ������ ������ ������ (����� ���� ���� � OW line)
#define SRC_PIN PINB // ������� ����� ��������� ������
#define SRC_PIN_NUM 1 // ����� PIN, � �������� ���������� ������, ��� ������� _BV()
#define src_msk (1<<SRC_PIN_NUM) // ������� ����� ��� �������� ������� �� ������ �� ��������������� ����

//���� define ��� ������ ������
#define CNT_QUICK 5 //���������� ������������ �� �����������, ���� ������ - ������ �����
#define CNT_SLOW 15 //���������� ������������ �� ������� �������
#define QUICK 1 //������������ ��������� �������
#define SLOW 2 //������������ �������� �������
#define RELEASED 0 //��������� ������ - ��������
#define PRESSED 1 //��������� ������ - ������

// ���� define ��� ���������� ��������� gsm ������
#define LED_PORT PORTB //����, � ������ �� ������� �������� ��������� ��������� (����� ���� ���� � OW line)
#define DDR_LED_PORT DDRB // ������� ����������� ������ �����, � ������ �� ������� �������� ��������� ��������� (����� ���� ���� � OW line)
#define LED_PIN_NUM 5 // ����� PIN, � �������� ��������� ���������, ��� ������� _BV()

//���� define ��� GSM
#define MYUBRR 103 // �������� usart 9600
#define RX_RING_SIZE 256 // ������ ������ ���� 256, ����� ���� �������� ��� ����� ��� �������� ����� 255->0

// ���� ��������� ��������� �� ����
unsigned char absence[] PROGMEM = "��� ��������"; //������ �� ����� ������ ������������� �������� ����� ������ � �������� ���������� �������
unsigned char no_answer[] PROGMEM = "��� ������ ����."; //������ �� ������ ���������� ������� ����� ������ ������ ���� ����-������
unsigned char present_n[] PROGMEM = "������� ����. "; //���������� ���������, ���������� ��� ��������� ���������� �������
unsigned char dev_excess[] PROGMEM = "����� ��������"; //������ � �������� ��������� ���������� �������, ���� ���������� �������� �������� 50
unsigned char error[] PROGMEM = "������ CRC-ID "; //������ �� ����� �������� CRC ID ����� ��������� ���������� �������, ��������� � �������,
unsigned char init_n[] PROGMEM = "�������.����. "; //���������� ��������, ��������� �������� CRC ����� ��������� ���������� �������
unsigned char init_srch[] PROGMEM = "��������� �����?"; // ������ ��� ���������� � ����� ������ ��������, ���� �� ����������� ��������� ���������� �������
unsigned char scratch_err[] PROGMEM = "��.CRC-����. "; //������ ��� �������� CRC ������, ����������� �� ��������, ��������� � �������
unsigned char no_answer_n[] PROGMEM = "��� ������ "; //������ �� ����� ������ ������ �������� ��� ���������� � main - ���� ������ �� �������, �� ����.���� == FF, ��������� ��� �������
unsigned char ow_check[] PROGMEM = "����� �����"; //��������� � main ��� ���������� ����� �� ����� ����������
unsigned char correction[] PROGMEM = "�������������"; // ��������� � add_ID ��� �����
unsigned char subst_add[] PROGMEM = "������/������-�"; // ��������� � add_ID ��� ����� � ���� ������/��������
unsigned char total_qty[] PROGMEM = "����� ���������"; // ��������� � add_ID ����� ������� ������
unsigned char plug_in[] PROGMEM = "���������� ����."; // ��������� � add_ID ����� ������� ������
unsigned char press_btn[] PROGMEM = "� ������� ������"; // ��������� � add_ID ����� ������� ������
unsigned char mem_full[]PROGMEM = "������ ���������";//��������� � add_ID, ���� ���������� �������� ��� 50, � ���������� ���
unsigned char new_dev_fnd[] PROGMEM = "������� �����"; // ������� ������ ��������� � add_ID, ���� ������� �����
unsigned char nev_dev_add[] PROGMEM = "��������� �����"; //������� ������ ��������� � add_ID � ����� ����� ����������
unsigned char element[] PROGMEM = "����������";// ������ ������ ��������� � add_ID, ���� ������� ��� ��������� �����
unsigned char substitute[] PROGMEM = "��������?"; //��������� � add_ID � ����� ������, ���� ����� ���������� ����-��, ��������� ��� �������
unsigned char add_to_end[] PROGMEM = "��������?"; //��������� � add_ID � ������ ����� ����������
unsigned char done[] PROGMEM = "������ ���������"; // ��������� � add_ID ����� ������� ������ �����������
unsigned char no_new[] PROGMEM = "����� ����-� ���"; // ��������� � add_ID ���� ������ ���� ���� ������ ������ ID
unsigned char delete[] PROGMEM = "�������?"; // ��������� - ������ ����� ���� �������������
unsigned char del_done[] PROGMEM = "�������"; //��������� �� ����� �������� ����������
unsigned char t_error[] PROGMEM = "�������� ����-��"; //��������� ��� ������� �������� ������������ ����������� � ������
unsigned char com_error[] PROGMEM = "�������� �������"; //��������� ��� ������ � ������ �������
unsigned char name_error_ren[] PROGMEM = "�������� ��� REN"; //��������� ��� ���������� ����� � ������ �������
unsigned char name_error_al[] PROGMEM = "�������� ��� AL"; //��������� ��� ���������� ����� � ������ �������
unsigned char name_error_sms[] PROGMEM = "�������� ��� SMS"; //��������� ��� ���������� ����� � ������ �������
unsigned char t_min[] PROGMEM = "Tmin"; // ����� ������ ��� ������������� ��������� ������������ ������� �����������
unsigned char t_max[] PROGMEM = "Tmax"; // ����� ������ ��� ������������� ��������� ������������� ������ �����������
unsigned char t_low[] PROGMEM = "T<";  // ����� ������ ��� ���������� � �������� ����������� ���� ������������ ������
unsigned char t_high[] PROGMEM = "T>";   // ����� ������ ��� ���������� � ��������� ����������� ���� ������������� ������
unsigned char sms_send []PROGMEM = "SMS";  // ����� ������ ��� ������������� ��������� sms � ������ ����������� �� �������
unsigned char on []PROGMEM = "���.";
unsigned char off []PROGMEM = "���.";
unsigned char blank []PROGMEM = " ";
unsigned char crash []PROGMEM = "������!";
unsigned char frame_err []PROGMEM = "������ �����!";
unsigned char rx_ring_ovf []PROGMEM = "�/����� ������!";

// ���� ���������� ���������� � ��������
typedef struct //������� ���� ��� ������
{
    uint8_t active : 1; // ������� ���������� �� �����
    uint8_t lt_alarm : 1; // ������ ����� ����������� ������� ����
    uint8_t ht_alarm : 1; // ������� ����� ����������� ��������
    uint8_t line_alarm : 1; //������� ���������� � �������� ���������
    uint8_t sms_T : 1; //��������� sms ��� � < tmin ��� � > tmax
    uint8_t relay_1 : 1; //���� 1
    uint8_t relay_2 : 1; //���� 2
    uint8_t reserved : 1;//���������
} bit_set;

typedef struct //��������� ��� ���������� ����������
{
    unsigned char name[N_NAME]; // ��� 1W ����������
    unsigned char code[8]; // ��� 1W ����������
    char tmax; // ������������ �
    char tmin; // ����������� �
    bit_set flags;// ����� ���������
} device; // ��������� ��� ���������� 1W ����������

/*typedef union //����������� ��� ���������� ����������
{
	unsigned char name[N_NAME]; // ��� 1W ����������
	unsigned char code[8]; // ��� 1W ����������
	char tmax; // ������������ �
	char tmin; // ����������� �
	bit_set flags;// ����� ���������
} dev_param;*/

device buffer; // ���������� ��� ������ ��� <-> �����
device ee_arr [2] EEMEM; // o�������� ������ �������� � �����, 2 - ���� ��������, ��� ������
unsigned char *location; //��������� �� ��� 1W ����������, ����������� ������ ������ ����� �� ���������
unsigned char dev_name[N_NAME] = {'S', 'e', 'n', 's', '.', '_', '_', '_'}; //"Dxxxxxx" - ��� ���������� �� ���������, � ���� ��������� ASCII ��� ����������� ������
const uint16_t dev_qty = (EEP_MEM - 1);//���������, ���������� �������� ������ ��������� ������ � ����� ��� ���������� ���������� ������������� ���������
const uint16_t dev_last_n = (EEP_MEM - 2);//���������, ���������� �������� ������ ������������� ������ � ����� ��� ���������� ���������� ������ ����������, ������������ � ������ � ������
//����� ��� ����������� ����������� �������������� ��������� � ������ ��� ���������� � ������������� ������ ����� �������� ��������� ����� ���� �������������� ��������
const uint8_t n_max = (uint8_t)((EEP_MEM - 2) / sizeof(device)); //����������� ���������� ���������� �������� � �����
uint8_t scratchpad [9]; //������ ������, ����������� �� �������� DS18B20
volatile uint8_t RX_IndexIN;// �������� ������ ���������� ������, ����� ����������� volatile, �������� ������ � ����������
volatile uint8_t RX_IndexOUT; // ��������� ������ ���������� ������
volatile uint8_t RX_ring[RX_RING_SIZE]; //������ ��� ���������� ������
volatile uint8_t flash_cnt; //������� ���������� ��� ��������� ������ gsm ��������� �����
volatile uint8_t flash_num; //����� ��������� �������
volatile uint8_t pause_num; //����� ��������� �������� ���� ����� ������� �������
const uint8_t gsm_lvl = 6; //������� gsm �������, ���� const


typedef struct // ��������� ��� ������ �������, �� ��� �������� ���� (�����)
{
    uint8_t name[N_NAME]; //��� ����������
    unsigned char dig_1;//������ ����� �����������
    unsigned char dig_2;//������ ����� �����������
    unsigned char dig_3;//���������� ����� �����������
    unsigned char sign; //���� �����������
} line;
line line_up; // ������� ������ �����
line line_dn; // ������ ������ �����

volatile uint16_t btn_cnt = 0; //������� ���������� ��������� ������
volatile uint8_t btn_state = 0; //��������� ������ - ������/��������
volatile uint8_t btn_time = 0; //����� ������� ������, ��������� ������ � ����������� ���������� �� ������������, �� ����� ������������ ������� ��� �������� �� �������� ���� �� ����������
volatile uint8_t press_time = 0; //��� ����� ������� ������ ������� ������
volatile uint16_t int_cnt = 0; //������� ������������ ������� TIMER0, ����� ������ ����� ������ �� ������������ �������
volatile uint8_t delay_cnt = 0; //������� ��� �������� �� ��������� ����� �������� ���� � ���� �����
//volatile uint16_t time_gsm = 0; //������� ��� �������� gsm-������


// ������� ������ ������� � ���
void lcd_com(unsigned char p)
{
    LCD_COM_PORT &= ~(1 << RS); // RS = 0 (������ ������)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p & 0xF0); // �������� ������� ����
    LCD_COM_PORT |= (1 << EN);  // EN = 1 (������ ������ ������� � LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN); // EN = 0 (����� ������ ������� � LCD)

    LCD_COM_PORT &= ~(1 << RS); // RS = 0 (������ ������)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p << 4); // �������� ������� ����
    LCD_COM_PORT |= (1 << EN); // EN = 1 (������ ������ ������� � LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN); // EN = 0 (����� ������ ������� � LCD)
    _delay_us(50);
}

// ������� ������ ������ � ���, ������� ������� �� lcd
void lcd_dat(unsigned char p)
{
    LCD_COM_PORT |= (1 << RS); // RS = 1 (������ ������)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p & 0xF0); // �������� ������� ����
    LCD_COM_PORT |= (1 << EN); // EN = 1 (������ ������ ������� � LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN); // EN = 0 (����� ������ ������� � LCD)

    LCD_COM_PORT |= (1 << RS); // RS = 1 (������ ������)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p << 4); // �������� ������� ����
    LCD_COM_PORT |= (1 << EN); // EN = 1 (������ ������ ������� � LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN); // EN = 0 (����� ������ ������� � LCD)
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
    lcd_com(0x28); // 4 ��� ����� 2 ������, 5�8
    lcd_com(0x08); //���������� ������, ���������� ����������� �������, ������ �� ������ 0000 1000(bin)
    lcd_com(0x06); //�����������������, ����� ����� ������ �������� 0000 0110(bin)
    lcd_com(0x01); // ������� �������
    _delay_us(3000);// ����� ���������� ������� �� ����� 1.5ms
    lcd_com(0x0C); //��������� ������ 0000 1100(bin)
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

// ������� ������� ����� �� LCD, ���������� �� Frame
void Display (uint8_t line_qty)
{
    send_arr_to_LCD_XY (line_up.name, 0, 0); //������� ��� ���������� � 0-� ������� 1 ������ ������
    lcd_dat_XY(line_up.sign, 11, 0);// ������� ����������� � 11-� (�� 0) ������� 1 ������ ������
    if (line_up.dig_1 != '0') //������� ����, ����� ������ ����� ���������� ������ �� �����
        lcd_dat(line_up.dig_1);
    lcd_dat(line_up.dig_2);
    lcd_dat('.');
    lcd_dat(line_up.dig_3);

    if (line_qty)	//������� ������ ������, ���� ��� ����
    {
        send_arr_to_LCD_XY (line_dn.name, 0, 1); //������� ��� ���������� � 0-� ������� 2 ������ ������
        lcd_dat_XY(line_dn.sign, 11, 1);// ������� ����������� � 11-� (�� 0) ������� 2 ������ ������
        if (line_dn.dig_1 != '0') //������� ����, ����� ������ ����� ���������� ������ �� �����
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

        if (!(i & 0x01) && (i==(n-1))) //���� ������ �������, �� ���������, ����� ����� �� LCD
        {
            lcd_clr(); // ������� �������
            Display (0); // ������� � ������� ������ ���-�� ����� �����: 0->1 ������, 1->2 ������
        }
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

        lcd_clr(); // ������� �������
        Display (1);// ������� � ������� ������ ���-�� ����� �����: 0->1 ������, 1->2 ������
    }
}

// ������� ������������� ��������
unsigned char init_device(void)
{
    unsigned char OK_Flag = 0; // ���������� ���������� ������ ����������� ��������
    // unsigned char Flag = 0;
    cli();
    OW_PORT &= ~_BV(OW_PIN_NUM);//� ����� ������ 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);//1 - ���� �� �����
    _delay_us(480);//�������� 480 ���
    //���������� ������
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);//0 - ���� �� ����
    _delay_us(70);//�������� 70 ��� ,����� ���� ������ ����� ��������
    if (!(OW_PIN & bit_msk)) // ��������� ���� ������ ��������, ���� PINB==0 - ����� ����
        OK_Flag = 1; // ���� ����� ����
    else
        OK_Flag = 0; // ���� ������ ���
    _delay_us(410);//��������� �������� 410 ��� ��������� �������� �����������
    sei();// ��������� ����������
    return OK_Flag;
}
// ������� �������� 1 � �����
void send_1 (void)
{
    cli(); //�������� ����� ����������
    OW_PORT &= ~_BV(OW_PIN_NUM);//� ����� ������ 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);//1 - ���� �� �����
    _delay_us(6);//�������� 15 ���
    //����������
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);//0 - ���� �� ����
    _delay_us(64);//�������� 45 ��� ,����� ���� ������
    sei();// ��������� ����������
}
// ������� �������� 0 � �����
void send_0(void)
{
    cli(); //�������� ����� ����������
    OW_PORT &= ~_BV(OW_PIN_NUM);//� ����� ������ 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);//1 - ���� �� �����
    _delay_us(60);//�������� 120 ���
    //����������
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);//0 - ���� �� ����
    _delay_us(10);//�������� 1 ��� ,����� ������� ���������� ����
    sei();// ��������� ����������
}

//������� ������ ������ �� �������
void send_command (unsigned char command)
{
    unsigned char i;
    for (i=0; i < 8; i++)
    {
        if (command & 0x01) // ���� ������� ���� 1, �� �������� 1
        {
            send_1 ();
        }
        else //�������� 0
        {

            send_0 ();
        }
        command >>= 1;//�������� ������ ��� ��������� ���������� ����
    }
}

//������� ������ �� ��������
unsigned char read_data(void)
{
    unsigned char bit;
    cli(); //�������� ����� ����������
    OW_PORT &= ~_BV(OW_PIN_NUM);//� ����� ������ 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);//1 - ���� �� �����
    _delay_us(6);//�������� 2 ���
    //����������, ���������� ���������� �������
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);//0 - ���� �� ����
    _delay_us(9);//�������� 9 ��� ,����� �����������
    bit = (OW_PIN & bit_msk);
    _delay_us(55);//�������� 50 ��� ,����� ������� ���������� ����
    sei();// ��������� ����������
    return bit;
}

// �������� CRC8
int8_t CRC_check(uint8_t *data, uint8_t crcbitN)
{
    uint8_t j; // ������� ������ � ������� data, ������ ���������
    uint8_t crc8 = 0; //���������� ��� ����������� ���������� ������� CRC8 �� ������� (crcbitN-1) ������ � CRC8 �� crcbitN �����
    uint8_t data_crc; // ���� � ������ ����� ������� ����� ������ ��������� ���� �� ����������� �� main �������
    uint8_t u; //������� ����� ��� ������� � ��������� ������� CRC8
    for(j = 0; j < crcbitN; j++)
    {
        unsigned char bit_crc; //��������� ����������
        data_crc = data[j];
        for (u = 0 ; u < 8; u++)
        {
            bit_crc = ((crc8 ^ data_crc) & 0x01);
            if (bit_crc == 0)
                crc8 >>= 1;
            else
            {
                crc8 ^= 0x18;//  11000 , �� ������ �.�. ��� ��� 0 � 1 ����� 1
                crc8 >>= 1; //������� �����
                crc8 |= 0x80;//+ 1000 0000
            }
            data_crc >>=1;
        }
    }
    if (crc8 == data[j]) // ���� ��������� ���� � CRC ����� - ������
        return 0;
    else
        return 1;
}

// ������� ���������� ���������� ID-���� ��� ������ ������� ��������
void find_ID (uint8_t* data, uint8_t* New_conflict, uint8_t* last_conflict)
{
    uint8_t p = 1;  //���������� ��� ����� ����������� ����
    uint8_t bit = 0x01;// ��������� ������� ���� � �����
    uint8_t bit1, bit2; //��� ��������� ����� ���� ���� ������
    uint8_t j = 0;// ������� ������ �����-������

    if (!init_device())	// ����� ������� ������ � ��������� ����� ��������
    {
        // ���� �-��� ������ ����� 0
        send_string_to_LCD_XY (absence, 0, 0);//������� "��� ��������" � 1-� ������� ����� ������� 1 ������ ������
        abort ();
    }
    send_command(0xF0); // ������� ������
    while (p <= 64) // ���� �� ����� ��������� ��� 64 ����
    {
        bit1 = read_data();//������ ����-����
        // _delay_us(2); // ��� ���� �������� ������ �� ��������
        bit2 = read_data();//������ ����-����
        if (bit1 && bit2) // ���������� ���������� ���� , ���� ��� �������
        {
            send_string_to_LCD_XY (no_answer, 0 ,0);//������� "��� ������ ����." � 1-� ������� ����� ������� 1 ������ ������
            abort ();
        }
        else if ((bit1) && (!bit2))// ��� = 1
            data[j] |= bit;//���������� ���
        else if((!bit1) && (bit2))// ��� = 0
            data[j] &= ~bit;
        else if((!bit1) && (!bit2))//�������� ��� 0
        {
            //����� ����� ���������� ������� �����,  � ������� ������� ��������� ���������  � ���������� N_conflict
            if (p == *last_conflict)//���� ������� ������� ���� � ������ ��������� �������� ==  ������� � ���������� ���������� (������ ����� 0-�� �������), �� ������� � ������ 1
                data[j] |= bit;
            else if (p > *last_conflict)// ���� ����� ������� ������, ������ ����������� ������, �� ������� 0 � ����� ������� ��������� �������
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
        else //�������� 0
        {
            send_0 ();
        }

        p++;//����������� �� 1
        bit <<= 1;//�������� �����

        if (!bit) //����� �������� ��� 8 ��� � �������� ����� 0
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
        res.quot >>= 3;// ����� �� 8
        res.rem = (uint8_t) (value - ((res.quot << 1) + (qq & ~7ul)));// ��������� �������
        if(res.rem > 9)// ������������ ������� � �������, ���� ������� >=10
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
    unsigned char i, j = 0;// ���������� ��������
    unsigned char n = 0;  //���������� ��������, ���������� � �����
    unsigned char m = 0; //���������� ��������, ��������� �������� CRC8
    unsigned char data [8];// �����-������ ��� �������� ����� ��������
    unsigned char New_conflict = 0; //���������� ��� ����� ������� ���� ���������
    unsigned char last_conflict = 0;  //���������� ��� ������ ������� ���� ���������
    uint8_t digits[N_DIGS];//������ ��� �������� � utoa_fast_div ��� ���������� ��� ������ �������� ����

    for (j = 0; j < 8; j++) //������� �����-������
    {
        data[j] = 0x00;
    }
    j = 0;//�������� ������� �����-�������

    do
    {
        New_conflict = 0;
        n++;
        if (n > n_max) //���� ����� ��������� ��������� 50, �� ������ � �����
        {
            send_string_to_LCD_XY (dev_excess, 0, 0);//������� "����� ��������" � 1-� ������� ����� ������� 1 ������ ������
            abort ();
        }

        find_ID (data, &New_conflict, &last_conflict); //������ �� ��������� ������ ��� ���������� ���������� ����������

        location = dev_name;//�������� ������� ���������� ������������� ASCII ��� ����� �� ���������
        // utoa_fast_div((n-1), &dev_name[(sizeof dev_name)- N_DIGS]);// � ��� �� ��������� ����������� ASCII ��� ������ ���������� �� ������� ������, �� 0.
        for (j = 0; j < N_DIGS; j++)//�������� ������ �������� ������ '0'
        {
            digits[j] = '0';
        }
        utoa_fast_div((n-1), digits);//��������� ����� ���������� � �������
        for (j = 1; j < 4; j++)////���������� 3 ��������� �������� digits � ��������� 3 �������� ������� ����� - ��� ����� ����������
        {
            dev_name[N_NAME-j] = digits[N_DIGS-j];
        }
        strncpy((void*)buffer.name, (void*)location, sizeof buffer.name); // ���������� ASCII ��� ����� � ���� name ������
        //���������� ������������ ����� ���������� � void*, strncpy �� ��� ������, ����� char*, � � ��� unsigned char*
        for (j=0; j < 8; j++) // ������ ID-���� 1-�� 1W ���������� � ID_string � ����������� ��� � �����-���������
        {
            buffer.code[j] = data[j];
        }
        /* ��������� ���� tmax, tmin � flags ������ ���������� */
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
        eeprom_update_block (&buffer, &ee_arr[n-1], sizeof(buffer)); // ���������� � ����� �������� �������� 1W ����������.
        sei();
    }
    while (last_conflict != 0); // ���� ����� ���� ��������� �� ����� 0, ���� ����� �� ��� ������� �������

    lcd_clr(); // ������� �������
    send_string_to_LCD_XY (present_n, 0, 0); // ������� "������� ����. "
    send_arr_to_LCD_XY (utoa_fast_div (n, digits), 14, 0); //������� ���������� ����-�, utoa_fast_div ����� ��������� �� ������ ��������� ������ ������� digits
    _delay_ms(2500);

    //������� �������, ������ ��������, ���������� ��������� ������������ ���������� ����������
    i = 0;//�������, ������ �������� � 0-� ��������
    j = 0;
    m = n; //��������� �������� �����. �������� ��� �������� ��������� �������� CRC
    while(i != n)//������������ ���������� ���������(����� n �� ����� ������), �������� � ������� ����������
    {
        eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer));// ��������� �������� ��������� �� �����
        if (CRC_check (buffer.code, 0x07)) // ������� ��������� �� ������ � ID, ����� ����� CRC8; ���� ������� 1, CRC �� ��
        {
            m--;
            buffer.flags.active = 0; //����� ����� �����������
            send_string_to_LCD_XY (error, 0, 0);//������� "������ CRC-ID" � 1-� ������� ����� ������� 1 ������ ������
            send_arr_to_LCD_XY (buffer.name, 0, 1);
            _delay_ms(2000);
        }
        i++;
    }

    lcd_clr(); // ������� �������
    send_string_to_LCD_XY (init_n, 0 ,0); //������� "�������.����. " � 1-� ������� ����� ������� 1 ������ ������
    send_arr_to_LCD_XY (utoa_fast_div (m, digits), 14, 0); //������� ���������� ����-�, utoa_fast_div ����� ��������� �� ������ ��������� ������ ������� digits
    _delay_ms(2500);
    lcd_clr(); // ������� �������
    cli();
    eeprom_update_byte ((uint8_t*)dev_qty, n);
    eeprom_update_byte ((uint8_t*)dev_last_n, (n-1));
    sei();
}

// ������� ������ �������� �������
uint8_t scratchpad_rd (void)
{
    uint8_t j = 0;// ������� ���� ��������
    uint8_t p;  //������� 72 ����� ��� ����� ����������  ��������
    uint8_t bit = 0x01;// ��������� ������� ���� ��� ������������ ����� ��������
    uint8_t bit_reg; // ���� ������ ��������� ������ ���������� ����

    for (j = 0; j < 9; j++) //������� ������
    {
        scratchpad[j] = 0x00;
    }
    j = 0;//�������� �������
    init_device(); // ����� ������� ������ � ��������� ����� ��������
    send_command(0x55);//������� ������������

    for (j = 0; j < 8 ; j++)// ������� ��� ���������� � �������� ����������
    {
        send_command (buffer.code[j]);
    }
    j = 0;//�������� �������
    send_command(0xBE); // ������ �������
    for (p = 1; p <= 72; p++) // ���� �� ����� ��������� ��� 72 ����
    {
        bit_reg = read_data(); //������ ���
        if (bit_reg)
            scratchpad[j] |= bit; //���������� ��� =1
        else
            scratchpad[j] &= ~bit; // ���������� ��� = 0
        bit <<= 1;//�������� �����
        if (!bit) //����� �������� ��� 8 ��� � �������� ����� 0
        {
            j++;
            _delay_ms(100);
            bit = 0x01;
        }
    }//������� �� ����� ����� ��������� 72-� �����
    init_device();
    if (scratchpad[4] == 0xFF) // ������ �� �������, ��� ���� ����.����� == 1
        return 1;
    else if (CRC_check(scratchpad, 0x08))//���� CRC �� OK
        return 2;
    else
        return 0;
}

//������������ ������, ����������� ������������ �������, ���������� �� ISR(TIMER0_OVF_vect)
void BTN_SCAN(void)
{
    if (!(SRC_PIN & src_msk)) //���� ������
    {
        if(btn_cnt < CNT_QUICK) //���� ������ ������ ������ ��������� �������
        {
            btn_cnt++; // ������ ������ �������, ������ �� ��������
        }
        else if ((btn_cnt >= CNT_QUICK) && (btn_cnt < CNT_SLOW)) //���� ������ ������ ������ ���������, �� �� ���������� �� �������� ����
        {
            if (!btn_state) //���� �������� ���� � ������ ���,�.�. ��� �� ���� ������������� �������
            {
                btn_state = PRESSED; // ��������� ���� �������
                btn_time = QUICK; // ��������� ���� ��� ������� ��������� �������
                btn_cnt++; // ������ ������� ������
            }
            else //���� ��� ���� ������������� �������
            {
                btn_cnt++; //������ ������ ������� ������
            }
        }
        else if (btn_cnt == CNT_SLOW) //���� ������ ������ ������ �������� ����
        {
            btn_cnt = CNT_SLOW + 10; // ����� � ��������� ���, ���� ������ ��� � �� ���������, ���� �� �����
            btn_time = SLOW; // ��������� ���� �������� �������
        }
    }
    else if ((SRC_PIN & src_msk)&&(btn_state == PRESSED)) //���� ������ �� ������, � ���� �������� ����, ������ ������ ���������
    {
        press_time = btn_time; //������� ������ ������������ ������� � �������� �� ��� ���������� �����
        btn_time = 0;
        btn_state = RELEASED;
        btn_cnt=0;
    }
}

void add_ID(void) // ����� ��������/����������/������� ���������
{
    unsigned char i, j = 0;// ���������� ��������
    unsigned char n = 0;  //���������� ��������, ���������� � �����
    unsigned char m = 0;  //������� ��������� ���������
    unsigned char data [8];// �����-������ ��� �������� ����� ��������
    unsigned char New_conflict = 0; //���������� ��� ����� ������� ���� ���������
    unsigned char last_conflict = 0;  //���������� ��� ������ ������� ���� ���������
    uint8_t digits[N_DIGS]; //������ ��� �������� � utoa_fast_div ��� ���������� ��� ������ �������� ����

    n = eeprom_read_byte((uint8_t*)dev_qty); //������ ���������� ��������, ���������� � �����
    lcd_clr(); // ������� �������
    send_string_to_LCD_XY (correction, 0, 0); //������� "�������������"
    _delay_ms (2000);
    press_time = 0; //�������� ��������� ������������� ����� �������, ����� �� ��������� ��������

    lcd_clr(); // ������� �������
    if (n) // ���������� ������ ��������, ���� n==0
    {
        send_string_to_LCD_XY (delete, 0, 0); //������� "�������?"
        while (!press_time) {;} // ���� ������ �� ��������, ���
        if (press_time == SLOW)
        {
            press_time = 0; //�������� ��������� ������������� ����� �������, ����� �� ��������� ��������
            for (i=0; i < n; i++)
            {
                eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer));// ��������� �������� ��������� �� �����
                send_arr_to_LCD_XY (buffer.name, 0, 1);
                while (!press_time) {;} //���� ������ �� ��������, ���
                if (press_time == SLOW)
                {
                    press_time = 0;
                    cli();
                    for (; i < n; i++)
                    {
                        eeprom_read_block (&buffer, &ee_arr[i+1], sizeof(buffer));// ��������� �������� ���������� ��������� �� �����
                        eeprom_update_block (&buffer, &ee_arr[i], sizeof(buffer)); // ���������� � ����� �������� ���������� ��������� �� ������� ������
                    }
                    eeprom_update_byte ((uint8_t*)dev_qty, (n-1));
                    sei();
                    lcd_clr(); // ������� �������
                    send_string_to_LCD_XY (del_done, 0, 0); //������� "�������"
                    _delay_ms (2000);
                    lcd_clr(); // ������� �������
                    return;
                }
                press_time = 0;
            }
        }
        press_time = 0; //�������� ��������� ������������� ����� �������, ����� �� ��������� ��������
    }

    lcd_clr(); // ������� �������
    send_string_to_LCD_XY (subst_add, 0 ,0);//������� "������/����������"
    _delay_ms(1500);

    for (j = 0; j < 8; j++)   {data[j] = 0x00;} //������� �����-������
    j = 0;//�������� ������� �����-�������

    n = eeprom_read_byte((uint8_t*)dev_qty); //������ ���������� ��������, ���������� � �����

    lcd_clr(); // ������� �������
    send_string_to_LCD_XY (total_qty, 0, 0); //������� "����� ���������"
    send_arr_to_LCD_XY (utoa_fast_div (n, digits), 7, 1); //������� ���������� ����-�, utoa_fast_div ����� ��������� �� ������ ��������� ������ ������� digits
    _delay_ms(1500);

    if (n == n_max) //���� ����� ���������, ��������, ��� �� ����������
    {
        for (i = 0; i < n; i++)
        {
            eeprom_read_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags));//��������� ���� 1W ���������� �� �����
            if (!buffer.flags.active) //���� � ����� ���� ����������, ��������� � ������
                break;
            else if (i == (n - 1))// ���� ���������� ��� � ����� ������, ������� � main � ����������
            {
                lcd_clr(); // ������� �������
                send_string_to_LCD_XY (mem_full, 0, 0); //������� "������ ���������"
                _delay_ms(2000);
                lcd_clr();
                return;
            }
        }
    }
    lcd_clr(); // ������� �������
    send_string_to_LCD_XY (plug_in, 0, 0); //������� "���������� ����-��"
    send_string_to_LCD_XY (press_btn, 0, 1); //������� "� ������� ������"
    while (!press_time) {;} //��� �������
    press_time = 0;

    do
    {
        New_conflict = 0;
        m++;
        if (m > n_max) //���� ����� ��������� ��������� 50, �� ������ � �����, ����� ������, ��� �������� ���� ���� � ����������� � main, ���� ��� ���������� ���������
        {
            lcd_clr(); // ������� �������
            send_string_to_LCD_XY (dev_excess, 0 ,0);//������� "����� ��������" � 1-� ������� ����� ������� 1 ������ ������
            _delay_ms(1500);
            lcd_clr();
            return;
        }

        find_ID (data, &New_conflict, &last_conflict); //������ �� ��������� ������ ��� ���������� ���������� ����������
        if (CRC_check (data, 0x07)) // ������� ��������� �� ������ � ID, ����� ����� CRC8; ���� ������� 1, CRC �� ��
        {
            send_string_to_LCD_XY (error, 0, 0);//������� "������ ����." � 1-� ������� ����� ������� 1 ������ ������
            _delay_ms(2000);
        }

        //�������� ��������� ���������� ID � ���������� � �����
        i = 0;
        do //���� for(i=0;i<n;i++) ������ �� do-while(i<n), ����� ��� n==0 � ���� �� �������, �.�. i==n==0 �����.
        {
            eeprom_read_block (&buffer.code, &ee_arr[i].code, sizeof(buffer.code));// ��������� ID ��������� �� �����
            if (!strncmp((void*)data, (void*)buffer.code, sizeof data)) // ���� ��������� ID ������ � ���������� � ����� - ������ ������ ID
                //���������� ������������ ����� ���������� � void*, �.�. strncmp �� ��� ������, ����� char*, � � ��� unsigned char*
                break;	//����� � do_while
            else if (strncmp((void*)data, (void*)buffer.code, sizeof data) && ((i == (n-1))||(i == n))) // ���� ��������� ID �� ������ �� � ����� ���������� � ����� -> ������� ����� ����-��
                //��� ������������ ���������� �� � ����� ���������� ��� ������ n==0 � ������� i==(n-1) ��������� ������� ||(i==n) , ����� ��� ������ ��� n==0 �� ��������
            {
                lcd_clr(); // ������� �������
                send_string_to_LCD_XY (new_dev_fnd, 0, 0); //������� "������� �����" � 1-� ������� ����� ������� 1 ������ ������
                send_string_to_LCD_XY (element, 0, 1); //������� "����������" � 1-� ������� ����� ������� 2 ������ ������
                _delay_ms(1500);
                lcd_clr(); // ������� �������
                if (n) // ���������� ������ ������, ���� n==0
                {
                    for (i = 0; i < n; i++)
                    {
                        eeprom_read_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags));//��������� ���� �������� 1W ���������� �� �����
                        if (!buffer.flags.active) // ���� ���������� ����������
                        {
                            eeprom_read_block (&buffer.name, &ee_arr[i].name, sizeof(buffer.name));
                            send_string_to_LCD_XY (substitute, 0, 0); // ������� "��������?"
                            send_arr_to_LCD_XY (buffer.name, 0, 1); // ������� ��� ����������� �������
                            while (!press_time) {;} //��� �������
                            if (press_time == SLOW) // ���� ����� - �������� � ������� � main, ����� ���� ������ ��������� ���������� ��� ������� � ���� ����������
                            {
                                // ������ ID-���� 1-�� 1W ���������� � ID_string � ����������� ��� � �����-���������
                                for (j=0; j < 8; j++)   {buffer.code[j] = data[j];}
                                buffer.flags.active = 1; // ������ �������
                                cli();
                                eeprom_update_block (&buffer.code, &ee_arr[i].code, sizeof(buffer.code)); // ���������� � ����� ID-��� ������ 1W ����������.
                                eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags)); //���������� � ����� ���� ������ 1W ����������.
                                sei();
                                lcd_clr(); // ������� �������
                                send_string_to_LCD_XY (done, 0, 0); //������� "���������"
                                _delay_ms(1500);
                                lcd_clr(); // ������� �������
                                return; //����� ������� � main
                            } //���� ��� ��������, ���� ������ ���������� ����������
                            press_time = 0; // ����� � ����� ���������� ��������� �� ��������� �������� ���
                        }
                    } //���� ������ ���� ���� � �� ����� � main, ��������� � ���� ���������� � ����� ������ � �����
                }
                lcd_clr(); // ������� �������
                send_string_to_LCD_XY (add_to_end, 0, 0);//������� "��������?"
                while (!press_time) {;} //��� �������
                if (press_time == SLOW) // ���� ����� - ��������� � ����� ������, ���� ������, � ������� � main
                {
                    if (n == n_max)
                    {
                        lcd_clr(); // ������� �������
                        send_string_to_LCD_XY (mem_full, 0, 0); //������� "������ ���������"
                        _delay_ms(2000);
                        lcd_clr(); // ������� �������
                        return;
                    }
                    for (j=0; j < 8; j++)  {buffer.code[j] = data[j];}

                    location = dev_name;//�������� ������� ���������� ������������� ASCII ��� ����� �� ���������
                    cli();
                    eeprom_update_byte((uint8_t*)dev_last_n, (eeprom_read_byte((uint8_t*)dev_last_n)+1)); //�������������� �����, ����������� ����� ���������� ������������ ����-��
                    sei();
                    //utoa_fast_div (eeprom_read_byte((uint8_t*)dev_last_n), &dev_name[(sizeof dev_name)- N_DIGS ]); //� ��� �� ��������� ����������� ASCII ��� ������ ����������
                    for (j = 0; j < N_DIGS; j++)//�������� ������ �������� ������ '0'
                    {
                        digits[j] = '0';
                    }
                    utoa_fast_div (eeprom_read_byte((uint8_t*)dev_last_n), digits); //��������� ����� � ������� ����
                    for (j = 1; j < 4; j++)//���������� 3 ��������� �������� digits � ��������� 3 �������� ������� ����� - ��� ����� ����������
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

                    lcd_clr(); // ������� �������
                    send_string_to_LCD_XY (nev_dev_add, 0, 0); //������� "��������� �����"
                    send_string_to_LCD_XY (element, 0, 1); //������� "����������" � 1-� ������� ����� ������� 2 ������ ������
                    _delay_ms(1500);
                    lcd_clr(); // ������� �������
                    return;
                }
                else // ���� ������� - �����, ������ ������� � main ��� ���������
                {
                    lcd_clr(); // ������� �������
                    return;
                }
            }
            i++;
        }
        while (i < n);
    }
    while (last_conflict != 0); // ���� ����� ���� ��������� �� ����� 0, ���� ����� �� ��� ������� �������
    // ���� ������� ��� ����������, �� ������ ��� ���������� � ��������� � main �� ���������, ������� � main ��� ���������
    lcd_clr(); // ������� �������
    send_string_to_LCD_XY (no_new, 0, 0);//������� "����� ���"
    _delay_ms(2000);
    lcd_clr(); // ������� �������
    return;
}

// ������� ������� ���������� ������
void ClearRing(void)
{
    cli();
    RX_IndexIN = 0;
    RX_IndexOUT = 0;
    sei();
}

// ������� ���������� ���������� ������������� ���� � ��������� ������
uint8_t RX_IndexNumber(void)
{
    if (RX_IndexIN >= RX_IndexOUT)
    {
        return (RX_IndexIN - RX_IndexOUT);
    }
    else
    {
        return ((RX_RING_SIZE - RX_IndexOUT) + RX_IndexIN);
    }
}

// ������� �������� ����� ������ � ��������� �����
uint8_t UDR_to_RX_Ring(char value)
{
    if((UCSR0A & (1 << FE0)))
        return 1;   // ������ ������������, �� ����� ����� ������
    else
        RX_IndexIN++;
    //RX_IndexIN &= BUFFER_MASK;
    if(RX_IndexIN == RX_IndexOUT)
        return 2; // ������������ ������, �� ����� ����� ������
    else
    {
        RX_ring[RX_IndexIN] = value;
        return 0;
    }
}

// ������� �������� ������ ������ �� ���������� ������
void RX_Ring_to_Str(uint8_t *str, uint8_t lenght)
{
    for(uint8_t i=0; i<lenght; i++)
    {
        RX_IndexOUT++;
        //RX_IndexOUT &= BUFFER_MASK;
        *str = RX_ring[RX_IndexOUT];
        str++;
    }
}

// ������� �� ����������� ������ � ��������� ������
uint8_t GetData(void)
{
    if(RX_IndexIN != RX_IndexOUT) return 1;
    return 0;
}

ISR(TIMER0_OVF_vect) //���������� ���������� ������� 0
{
    int_cnt++; //������� ����������
    if (int_cnt == 2) //��� ������������ �������� ������� 1024 ����� ���.�������� �� 2, ����� ����� 30��
    {
        BTN_SCAN (); //��������� ������
        int_cnt = 0; //�������� ����� ������
    }
    delay_cnt++; //������� �������� 2c ��� ���������, ����������� ����� �������� ��������� �������� �� �����, ������������ � main
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

void USART_Init( unsigned int ubrr)
{
    UBRR0H = (uint8_t)(ubrr >> 8); //�������� 9600
    UBRR0L = (uint8_t)ubrr; //�������� 9600
    UCSR0A = 0; //�������� �����
    UCSR0B = (1 << RXEN0)|(1 << TXEN0)|(1 << RXCIE0); //�������� ��������, ���� � ���������� �� �����
    UCSR0C = (1 << UCSZ01)|(1 << UCSZ00)|(1 << USBS0); //8 ���, 1 ���� ���
}

void USART_TXD(uint8_t data) // �������� ����� �� UART
{
    while (!( UCSR0A & (1 << UDRE0))) {;} // ���� ���� �� ���������� ���������� ������
    UDR0 = data;	// ��������� ������� ������
}

void USART_CRLF(void) //�������� CR � LF
{
    USART_TXD('\r');
    USART_TXD('\n');
}

void arr_to_USART(uint8_t *s ) // �������� ������� �� UART
{
    while(*s)
    {
        USART_TXD(*s++);
    }
}

void string_to_USART(const uint8_t *s ) // �������� ����� �� ���� �� UART
{
    while(pgm_read_byte(s))
    {
        USART_TXD(pgm_read_byte(s++));
    }
}

ISR(USART_RX_vect) // ���������� ���������� �� ������� ������ � UDR0
{
    uint8_t temp;
    //uint8_t wr_err = 0; //��� ���������� ������ � ������
    temp = UDR0;
    //if(!(temp == 0x0A || temp == 0x0D))
    UDR_to_RX_Ring(temp);

/*
	if(wr_err == 1) //��� ������ ����� ������� ���������
	{
		send_string_to_LCD_XY(frame_err, 0, 0);
		string_to_USART (frame_err);
		_delay_ms(1500);
	}
	wr_err = UDR_to_RX_Ring(temp);// ����������� ������ � ������ � ��������� ��� ����������
	if(wr_err == 2); //��� ������ ������������ ������ ������� ���������
	{
		send_string_to_LCD_XY(rx_ring_ovf, 0, 0);
		string_to_USART (rx_ring_ovf);
		_delay_ms(1500);
	}
	*/
}

//������ ������� ���������
int main(void)
{
    unsigned char i, j = 0;// ���������� ��������
    unsigned char temperature[5];	// ������ ������ ����������� LB � HB
    uint16_t temp_int; //����� ����� �����������
    unsigned char temp_float; // ������� ����� �����������
    int8_t temp_int_signed; // ����� ����� �-�� �� ������ ��� ��������� � ���������� ����������
    unsigned int temp; // ��������� ���������� ��� �������� �� ��������������� ���� � ������ ��� "-" �����������
    unsigned char temp_sign; // ������� ����� �����������
    uint16_t Number = 0; //���� ������ �������� �����������
    uint8_t Dig_1, Dig_2, Dig_3, sign = 0;//���������� ��� ����� �������� ���� ����������� � ������� �����
    uint8_t n = 0; // ��� ���������� ���������� � ����� ���������
    uint8_t srch_done = 0; // ������� ���������� ��������� ����������
    uint8_t chng_done = 0; // ������� ��������� ��������� ����� UART
    uint8_t digits[N_DIGS]; //������ ��� �������� � utoa_fast_div ��� ���������� ��� ������ �������� ���� �����������
    uint8_t tmp[20]; //������ ��� �������� �� ���������� ������
    int8_t *last_t = (void*) calloc(n_max, sizeof(*last_t));

    // LCD_COM_PORT_DDR |= (1<<RS)|(1<<EN); //����� RS � EN ������, �������. ���� DAT � COM ������� �� ������ �����
    // LCD_COM_PORT = 0x00; // ������ 0 � RS � EN, ����������������� ���� DAT � COM ������� �� ������ �����
    LCD_DAT_PORT_DDR = 0xFF; // ���� ������ (� ������ � ������ ������) ��� - �� �����
    LCD_DAT_PORT = 0x00;// ������ 0 � ���� ������ � ������ ���
    _delay_ms(200); // �������� ���������� ���
    lcd_init(); // ������������� �������

    DDR_LED_PORT |= _BV(LED_PIN_NUM); //������ 1 - ��� led ����������� �� �����

    USART_Init(MYUBRR); //�������� UART
    ClearRing();	// �������� �����

    TIMSK0 |= (1<<TOIE0);  // ��������� ���������� �� ������������
    TCCR0B |= (1<<CS02) | (1<<CS00); // �������� ������� 1024
    TCCR0B &= ~(1<<CS01); // �������� ������� 1024
    TCCR0A &= ~(1<<WGM00) & ~(1<<WGM01); //����� ������ "normal"
    sei();

    DDR_SRC_PORT &= ~_BV(SRC_PIN_NUM);//������ 0 - ��� ������ ����������� �� ����
    SRC_PORT |= _BV(SRC_PIN_NUM);//������ 1 - �����. �������� � +
    if (!(SRC_PIN & src_msk)) //���� �� ���� 0, �.� ������ ������, ������ ������ ����� ��������� �� ���� � ���������� ���������� � �����
    {
        while (!press_time) {;} // ���� ������ �� ��������, ���
        press_time = 0; //�������� ��������� ������������� ����� �������, ����� �� ��������� ��������
        search_ID(); //��������� ���������� ���� ID-�����
        srch_done = 1; //������� ����������� ������ ����������
    }
    while (!(n = eeprom_read_byte((uint8_t*)dev_qty)))//���� ������ ���������� �� ����, ��������� � ��������� n, ���� 0 - ��� � �������������, ����� �� �����, ���� n==0;
    {
        lcd_clr(); // ������� �������
        send_string_to_LCD_XY (absence, 0, 0); //������� "��� ��������"
        send_string_to_LCD_XY (init_srch, 0, 1);// ������� "��������� �����?"
        while (!press_time) {;} // ���� ������ �� ��������, ���
        add_ID ();
        press_time = 0;
        srch_done = 1; //������� ������������ ���������� ���������
    }

    if (!srch_done) // ���� ���������� �� �����������, ��������� � ������ ����� ���������� � ����� � ����������
    {
        lcd_clr(); // ������� �������
        send_string_to_LCD_XY (ow_check, 0, 0);// ��������� "����� �����" � 1-� ������� ����� ������� 1 ������ ������
        for (uint8_t i = 0; i< n; i++) //�������� ���������� � ������� ������� (�� 0)
        {
            eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer)); // ��������� �������� ��������� �� �����
            uint8_t pad_res = scratchpad_rd(); // ��������� ������ ��������
            if (!pad_res)
            {
                buffer.flags.active = 1;// ������ ����� �����������
            }
            else if (pad_res == 1)//���� ������ ������ �� �������, ����.���� ����� == FF
            {
                buffer.flags.active = 0; //����� ����� �����������
                lcd_clr(); // ������� �������
                send_string_to_LCD_XY (no_answer_n, 0, 0); // ������� "��� ������ " � 1-� ������� ����� ������� 1 ������ ������
                send_arr_to_LCD_XY (buffer.name, 0, 1);
                _delay_ms(2000);
                lcd_clr(); // ������� �������
            }
            else if (pad_res == 2) //������ ������� i-�� ����., ���� CRC �� ��, ������� ��������� �� ������
            {
                buffer.flags.active = 0; //����� ����� �����������
                lcd_clr(); // ������� �������
                send_string_to_LCD_XY (scratch_err, 0, 0); // ������� "��.CRC-����. " � 1-� ������� ����� ������� 1 ������ ������
                send_arr_to_LCD_XY (buffer.name, 0, 1);
                _delay_ms(2000);
                lcd_clr(); // ������� �������
            }
            cli();
            eeprom_update_block (&buffer, &ee_arr[i], sizeof(buffer)); // ���������� � ����� �������� �������� 1W ���������� � ���������� ������ ������.
            sei();
        }
        lcd_clr(); // ������� �������
    }

    while(1)
    {
        n = eeprom_read_byte((uint8_t*)dev_qty); //������ ���������� ��������, ���������� � �����
        for (i = 0; i< n; i++) //�������� ����� � ������� ������� (�� 0)
        {
            eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer)); // ��������� �������� ��������� �� �����
            if (buffer.flags.active)//���� ���� ����������� ������, ����������� �����������
            {
                init_device();//������� ������ � �����������
                send_command(0x55);//������� ������������
                // ����� ��������� ��� ���������� � �������� ����������
                for (j = 0; j < 8 ; j++)
                {
                    send_command (buffer.code[j]);
                }

                send_command (0x44);//������� ��������������
                while (!read_data()) ;// ����������� ���� ���� �� ����� �� ����������� 1 - �������������� ���������
                init_device();//������� ������ � �����������
                send_command(0x55);//������� ������������
                for (j = 0; j < 8 ; j++)   // ����� �������� ������ ����������, � �������� ����� ����������
                {
                    send_command (buffer.code[j]);
                }
                send_command (0xBE);//������� ������ ������
                for (j = 0; j < 5; j++) //��������� ������ ��� ����� ����������� � ����.����
                {
                    unsigned char i;//��������� ���������� ��� ����������� �����
                    unsigned char data_temp = 0x00;
                    for (i = 0; i < 8; i++)
                    {
                        data_temp >>= 1;
                        if (read_data()) //���� 1, �� �������������  ������� ��� 1
                            data_temp |= 0x80;
                    }
                    temperature[j] = data_temp;
                }
                init_device();	// ����� �������� ��� ����������� �������� ������
                // ����������� ���������� � �������� �����������
                if (!(temperature[4] == 0xFF)) //���� ����.���� ������ ����� ==FF, �� ������ ��������.
                {
                    if (buffer.flags.line_alarm) //���������� ���� ����������, ���� �� ��� ������ � �����-�� ���������� �����? � ������ ���������
                    {
                        buffer.flags.line_alarm = 0;
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags)); //���������� ����� � �����
                        sei();
                    }
                    if ((temperature[1]&0b10000000) == 0) // �������� �� ��������������� �����������
                        temp_sign =0; //������ ���� �� ���
                    else //��������� �� ���.���� � ������
                    {
                        temp = ((unsigned int)temperature[1]<<8) | temperature[0];
                        temp = ~temp + 1;
                        temperature[0] = temp;
                        temperature[1] = temp>>8;
                        temp_sign = 1;  //������ ����� �� ���
                    }

                    temp_int = ((temperature[1]&0b00000111)<<4)|(temperature[0]>>4);	//�������  ����� ����. �����������
                    temp_float = (temperature[0]&0b00001111); //�������� � ������� ������� ����� ������� �����

                    temp_int_signed = (!temp_sign ? temp_int : ~(temp_sign - 1)); // ��������� ���� � temp_int ��� ��������� � tmin, ��� �����.����� ������� � ���.���
                    last_t[i] = temp_int_signed; //������� � ������ ��������� ���������� ��������
                    if ((temp_int_signed < buffer.tmin)&&(!buffer.flags.lt_alarm)) //���� � ���� �������, � ���� �� ����������
                    {
                        buffer.flags.lt_alarm = 1; // ������ ����
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                        sei();
                        if (buffer.flags.sms_T) //���� ���������� ���� �������� sms, ���������� �� lcd
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
                    else if ((temp_int_signed > buffer.tmax)&&(!buffer.flags.ht_alarm)) //���� � ���� �������, � ���� �� ����������
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
                    else if ((temp_int_signed > buffer.tmin)&&(buffer.flags.lt_alarm)) //���� � ���� �������, � ���� ����������
                    {
                        buffer.flags.lt_alarm = 0; // ������� ����
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                        sei();
                    }
                    else if ((temp_int_signed < buffer.tmax)&&(buffer.flags.ht_alarm)) //���� � ���� �������, � ���� ����������
                    {
                        buffer.flags.ht_alarm = 0; // ������� ����
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //���������� ����� � �����
                        sei();
                    }
                    //����������� � ����� ����� � *10 (����� ������ 1 ���������� ����), ���������� "." ����� ������� �������������
                    /*
                    ��������:
                    I.
                    1. �������� ������� ����� �� 0,0625 ��� �������� � ���������� �����, ����� ��� �� 10 ��� �������� � ����� (����� ���� ���� ����� �������)
                        temp_float = ((temp_float << 2) + temp_float) >> 3; - ����� �������� �� 5/8 (0,625) ��� ((_)*4 + (_))/8
                    2. temp_int = (temp_int << 3) + (temp_int << 1) - �������� ����� ����� �� 10, ��� (_)*8 + (_)*2
                    3. 	temp_float + temp_int;  �������� ����-��, ���������� �� 10
                    II. ������ temp_float �������� 0.0625, ����� (temp_float + temp_int) �������� �� 10 � ������������� �������� ��������� � ������
                    Number = ((uint16_t)((temp_float*0.0625 + temp_int)*10));//���� �������� � uint8_t, ����� ���� �� float;
                    */
                    temp_int = (temp_int << 3) + (temp_int << 1); //�������� ����� ����� �� 10, ����� 0 � �������� ��� ���������� �����
                    temp_float = ((temp_float << 2) + temp_float) >> 3; //�������� ������� ����� �� (0,0625 * 10), ����� �� 5/8, �������� ���������� ����� *10
                    Number = temp_float + temp_int; // �� ����� ������ ����� ������� �����, ����������� �� 10
                    //����������� ����� � �����, ����� �� � ���� �������� ����
                    for (j = 0; j < N_DIGS; j++) //�������� ������ �������� ������ '0', �.�. ������� ���� �����
                    {
                        digits[j] = '0';
                    }
                    j = 0;//�������� �������
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
                else //���� ������ ����� ��������, �� ������� ��� ---.-
                {
                    if (!buffer.flags.line_alarm) //��������� ���� ����������, ���� �� �� ��� ������ � �����-�� ���������� �����
                    {
                        buffer.flags.line_alarm = 1;
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags)); //���������� ����� � �����
                        sei();
                        lcd_clr();
                        send_arr_to_LCD_XY(buffer.name, 0, 0); //�������� ������������ �� ������
                        send_string_to_LCD (blank);
                        send_string_to_LCD (crash);
                        arr_to_USART(buffer.name);
                        USART_TXD(' ');
                        string_to_USART(crash);
                        USART_TXD('\r');
                        USART_TXD('\n');
                        _delay_ms(1500);
                    }
                    Dig_1 = '-';
                    Dig_2 = '-';
                    Dig_3 = '-';
                    sign = '-';
                }
            }
            else  //���� ���� ����������� �������, �� ������� ��� ???.?
            {
                Dig_1 = '?';
                Dig_2 = '?';
                Dig_3 = '?';
                sign = '?';
            }
            Frame (Dig_1, Dig_2, Dig_3, sign, i, n);  //������� ���� �������� ���� �  �����, � ����-�� ��� �������.� ������ �����., ���-�� �����

            //���������� ������������� �������� ����� ������ ������:
            //���������� ������ ������ �� �������: � ����-� 0,2,4... �������, 1,3,5... ������;
            //���� ������ ������ - ������ �������� ����� ������ ������.
            //���� ������ �������, �� ���������� ��������� - ���� �������� ��� ���������� ������
            if ((i & 0x01)||(i == (n-1)))
            {
                delay_cnt = 0; // ����� �������� �������� �� ���������, ���������� ������ 120 ���������� ~2�
                while (delay_cnt <= 120) // ���� ��� ��������� ����� �������� ������
                {
                    switch (press_time) // ���� �������������� ������� ������ � ��������� �� ������������ ����
                    {
                        case QUICK :
                            lcd_clr(); // ������� �������
                            send_arr_to_LCD_XY(utoa_fast_div(n_max, digits), 0, 0); //������� n_max - �������� �����
                            _delay_ms(1500);
                            press_time = 0;
                            break;
                        case SLOW :
                            add_ID ();
                            //��������� � ��������� n, ���� 0 - ������� � �������������(��� �������� ���� ����� �������� ���� �������� � ���� �������������)
                            while (!(n = eeprom_read_byte((uint8_t*)dev_qty)))
                            {
                                send_string_to_LCD_XY (absence, 0, 0); //������� "��� ��������"
                                _delay_ms(1500);
                                add_ID ();
                            }
                            i = 0; // ����� �� ����������� ��������� ����� �������
                            press_time = 0;
                            break;
                        case 0 :
                            break;
                    }
                    if(GetData()) //���� ��� ��������� ��������� ��������� �����
                    {
                        _delay_ms(100); //����� �������� ������� �������
                        lcd_clr(); // ������� �������
                        uint8_t num = RX_IndexNumber(); //������� ���� � ��������� ������
                        RX_Ring_to_Str(tmp, num); //��������� �� ������ � tmp
                        lcd_clr();
                        for(j=0; j < (num-1); j++) //������� �������� ������� �� lcd
                        {lcd_dat_XY(tmp[j], j, 0);}
                        _delay_ms(2000);
                        chng_done = 0; // �������� ������� ��������� ���������� �������
                        if ((tmp[0]=='R')&&(tmp[1]=='E')&&(tmp[2]=='N')&&(tmp[3]==' '))
                        {
                            for (i = 0; i < n; i++)
                            {
                                eeprom_read_block (&buffer.name, &ee_arr[i].name, sizeof(buffer.name));
                                if (!strncmp((void*)buffer.name, (void*)&tmp[4], sizeof(buffer.name)-1))
                                {
                                    location = &tmp[12];
                                    strncpy((void*)buffer.name, (void*)location, sizeof (buffer.name)-1); // ���������� ASCII ��� ����� � ���� name ������
                                    cli();
                                    eeprom_update_block (&buffer.name, &ee_arr[i].name, sizeof(buffer.name)-1);
                                    sei();
                                    chng_done = 1;
                                }
                            }
                            if (!chng_done)
                            {
                                lcd_clr();
                                send_string_to_LCD_XY (name_error_ren, 0, 0);
                                _delay_ms(2000);
                            }
                        }
                        else if ((tmp[0]=='T')&&((tmp[1]=='L')||(tmp[1]=='H'))&&(tmp[2]=='A')&&(tmp[3]=='L')&&(tmp[4]==' ')&&(tmp[4 + N_NAME]==' ')&&((tmp[5 + N_NAME]=='-')||(tmp[5 + N_NAME]=='+')))
                        {
                            int8_t t_lim = atoi_fast (&tmp[5 + N_NAME]);
                            if ((t_lim >= -127) && (t_lim <= 127))
                            {
                                for (i = 0; i < n; i++)
                                {
                                    eeprom_read_block (&buffer.name, &ee_arr[i].name, sizeof(buffer.name));
                                    if (!strncmp((void*)buffer.name, (void*)&tmp[5], sizeof(buffer.name)-1))
                                    {
                                        if (tmp[1]=='L')
                                        {
                                            buffer.tmin = t_lim;
                                            cli();
                                            eeprom_update_block (&buffer.tmin, &ee_arr[i].tmin, sizeof buffer.tmin);
                                            sei();
                                            lcd_clr();
                                            send_arr_to_LCD_XY(buffer.name, 0, 0);
                                            send_string_to_LCD (blank);
                                            send_string_to_LCD (t_min);
                                            send_string_to_LCD (blank);
                                            lcd_dat ((buffer.tmin & 0b10000000) ? '-' : '+');
                                            send_arr_to_LCD (utoa_fast_div (((buffer.tmin & 0b10000000) ? ((~buffer.tmin) + 1) : buffer.tmin), digits));
                                            chng_done = 1;
                                            _delay_ms(1500);
                                        }
                                        else if (tmp[1]=='H')
                                        {
                                            buffer.tmax = t_lim;
                                            cli();
                                            eeprom_update_block (&buffer.tmax, &ee_arr[i].tmax, sizeof buffer.tmax);
                                            sei();
                                            lcd_clr();
                                            send_arr_to_LCD_XY(buffer.name, 0, 0);
                                            send_string_to_LCD (blank);
                                            send_string_to_LCD (t_max);
                                            send_string_to_LCD (blank);
                                            lcd_dat ((buffer.tmax & 0b10000000) ? '-' : '+');
                                            send_arr_to_LCD (utoa_fast_div (((buffer.tmax & 0b10000000) ? ((~buffer.tmax) + 1) : buffer.tmax), digits));
                                            chng_done = 1;
                                            _delay_ms(1500);
                                        }
                                    }
                                }
                                if (!chng_done)
                                {
                                    lcd_clr();
                                    send_string_to_LCD_XY (name_error_al, 0, 0);
                                    _delay_ms(2000);
                                }
                            }
                            else send_string_to_LCD_XY (t_error, 0, 0);
                        }
                        else if ((tmp[0]=='S')&&(tmp[1]=='M')&&(tmp[2]=='S')&&(tmp[3]==' ')&&(tmp[4]=='T')&&((tmp[5]=='L')||(tmp[5]=='H'))&&((tmp[6]=='1')||(tmp[6]=='0'))&&(tmp[7]==' '))
                        {
                            for (i = 0; i < n; i++)
                            {
                                eeprom_read_block (&buffer.name, &ee_arr[i].name, sizeof(buffer.name));
                                eeprom_read_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags));
                                if (!strncmp((void*)buffer.name, (void*)&tmp[8], sizeof(buffer.name)-1))
                                {
                                    if ((tmp[5]=='L')&&(tmp[6]=='1'))
                                    {
                                        buffer.flags.sms_T = 1;
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
                                        send_string_to_LCD (on);
                                        chng_done = 1;
                                        _delay_ms(1500);
                                    }
                                    else if ((tmp[5]=='H')&&(tmp[6]=='1'))
                                    {
                                        buffer.flags.sms_T = 1;
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
                                        _delay_ms(1500);
                                    }
                                    else if ((tmp[5]=='L')&&(tmp[6]=='0'))
                                    {
                                        buffer.flags.sms_T = 0;
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
                                        _delay_ms(1500);
                                    }
                                    else if ((tmp[5]=='H')&&(tmp[6]=='0'))
                                    {
                                        buffer.flags.sms_T = 0;
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
                                        _delay_ms(1500);
                                    }
                                }
                            }
                            if (!chng_done)
                            {
                                lcd_clr();
                                send_string_to_LCD_XY (name_error_sms, 0, 0);
                                _delay_ms(2000);
                            }
                        }
                        else if ((tmp[0]=='A')&&(tmp[1]=='L')&&(tmp[2]=='L')&&(tmp[3]==' ')&&(tmp[4]=='T'))
                        {
                            for (i=0; i<n; i++)
                            {
                                eeprom_read_block (&buffer.name, &ee_arr[i].name, sizeof(buffer.name));
                                arr_to_USART (buffer.name);
                                string_to_USART(blank);
                                USART_TXD(((last_t[j] & 0b10000000) ? '-' : '+'));
                                arr_to_USART(utoa_fast_div (((last_t[i] & 0b10000000) ? ((~last_t[i]) + 1) : last_t[i]), digits));
                                USART_CRLF();
                            }
                        }
                        else
                        {
                            lcd_clr();
                            send_string_to_LCD_XY(com_error, 0, 0);
                            _delay_ms(2000);
                        }
                    } // ����������� ������ ������ � �������� �� ���������� ������
                } // ����������� ������ ����� while �������� ��� ���������
            }	// ����������� ������ �������� ��� ���������
        }	// ����������� ������ ����� ������ ��������� �� ������
    }	// ����������� ������ ������������ ����� main
}	// ����������� ������ �������� ���������

