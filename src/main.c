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
#define bit_msk 0x01 // ������� ����� ��� �������� ������� �� ����� 1-Wire �� ��������������� ����

//���� define ��� ������
#define SRC_PORT PORTB //����, � ������ �� ������� �������� ���������� ������ ������ ������ (����� ���� ���� � OW line)
#define DDR_SRC_PORT DDRB // ������� ����������� ������ �����, � ������ �� ������� �������� ���������� ������ ������ ������ (����� ���� ���� � OW line)
#define SRC_PIN PINB // ������� ����� ��������� ������
#define SRC_PIN_NUM 1 // ����� PIN, � �������� ���������� ������, ��� ������� _BV()
#define src_msk 0x02 // ������� ����� ��� �������� ������� �� ����� 1-Wire �� ��������������� ����

//unsigned char tempC[] PROGMEM = "����-�� ";
unsigned char absence[] PROGMEM = "��� ��������";
unsigned char error[] PROGMEM = "������ ����.";
unsigned char present_n[] PROGMEM = "�����.���. ";
unsigned char init_n[] PROGMEM = "����.���. ";
unsigned char no_answer[] PROGMEM = "��� ������ ����. ";
unsigned char temp_sign; // ������� ����� ����������� ��� �-��� ������ �� ���


typedef struct //��������� ��� ���������� ����������
{
    unsigned char name[8]; // ��� 1W ����������
    unsigned char code[8]; // ��� 1W ����������
    char tmax; // ������������ �
    char tmin; // ����������� �
} device; // ��������� ��� ���������� 1W ����������

device buffer; // ���������� ��� ������ ��� <-> �����
device ee_arr [2] EEMEM; // o�������� ������ �������� � �����, 2 - ���� ��������, ��� ������
unsigned char *location; //���������� ��� ����� 1W ��������� - ������� ������ ������ ����� �� ���������
unsigned char dev_num = 0x00; //���������� ����������� ������ ��� ���������� � ����� 1W ��������� �� ���������,
unsigned char dev_name[8] = {'D', 'x', 'x', 'x', 'x', 'x', 'x', 'x'}; //"D__" - ��� ���������� �� ���������, � ���� ��������� ASCII ��� ����������� ������
//unsigned char dev_qty EEMEM;
const uint16_t dev_qty = 0x3FF;//���������, ���������� �������� ������ ��������� ������ � ����� ��� ���������� ���������� ��������� ���������

typedef struct // ��������� ��� ������ �������
{
    uint8_t name[8]; //��� ����������
    short unsigned int dig_1;//������ ����� �����������
    short unsigned int dig_2;//������ ����� �����������
    short unsigned int dig_3;//���������� ����� �����������
    unsigned char sign; //���� �����������
} line;
line line_up; // ������� ������
line line_dn; // ������ ������

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
    //lcd_com(0x80); // ����� �������� ������ � 1-� ������� ����� ������� 1 ������ ������
}

// ������� ������� ����� �� LCD, ���������� �� Disp_prep
void Display (uint8_t line_qty)
{
    if (!line_qty) //���� ������ ������ ���� (�������) - ������ �������
    {
        lcd_com(0x01); // ������� �������
        _delay_us(1500);// ����� ���������� ������� �� ����� 1.5ms
    }
    //������� ������ ������ � ����� ������
    lcd_com(0x80); // ������� ��� � 1-� ������� ����� ������� 1 ������ ������
    send_arr_to_LCD (line_up.name); //������� ��� ����������
    lcd_com(0x88); // ������� ����������� � 8-� (�� 0) ������� 1 ������ ������
    if (!line_up.sign)
    {lcd_dat('+');}
    else
    {lcd_dat('-');}

    lcd_dat(line_up.dig_1 + '0');
    lcd_dat(line_up.dig_2 + '0');
    lcd_dat('.');
    lcd_dat(line_up.dig_3 + '0');

    //������� ������ ������, ���� ��� ����
    if (line_qty)
    {
        lcd_com(0xC0); // ������� ������ � 1-� ������� ����� ������� 2 ������ ������
        send_arr_to_LCD (line_dn.name); //������� ��� ����������
        lcd_com(0xC8); // ������� ����������� � 8-� (�� 0) ������� 2 ������ ������
        if (!line_up.sign)
        {lcd_dat('+');}
        else
        {lcd_dat('-');}

        lcd_dat(line_dn.dig_1 + '0');
        lcd_dat(line_dn.dig_2 + '0');
        lcd_dat('.');
        lcd_dat(line_dn.dig_3 + '0');
    }
}

// ������� �������� ����� �� ������������ ����� Number,
// ������������ ����� ������ � ����������� ��������� �� ��������,
// ��������� ������ ����� ��� ��������� �� LCD
// ���������� �� main
void Disp_prep (uint16_t Number, uint8_t i, uint8_t n)
{
    short unsigned int j=0;
    short unsigned int Num1, Num2, Num3;
    Num1=Num2=0;
    while (Number >= 100)
    {
        Number -= 100;
        Num1++;
    }
    while (Number >= 10)
    {
        Number -= 10;
        Num2++;
    }
    Num3 = Number;
    //���������� ������ LCD: i ��� � ����-�: 0,2,4... �������, 1,3,5... ������
    if (!(i & 0x01))
    {
        while (j < 8)
        {
            line_up.name[j] = buffer.name[j];
            j++;
        }
        line_up.dig_1 = Num1;
        line_up.dig_2 = Num2;
        line_up.dig_3 = Num3;
        line_up.sign = temp_sign;
        if (!(i & 0x01) && (i==(n-1))) //���� ������ �������, �� ���������, ����� ����� �� LCD
        {
            Display (0); // ������� � ������� ������ ���-�� ����� 0->1, 1->2
        }
    }
    else
    {
        while (j < 8)
        {
            line_dn.name[j] = buffer.name[j];
            j++;
        }
        line_dn.dig_1 = Num1;
        line_dn.dig_2 = Num2;
        line_dn.dig_3 = Num3;
        line_up.sign = temp_sign;
        Display (1);// ������� � ������� ������ ���-�� ����� 0->1, 1->2
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
// ������� ������ ���������, ������ � ������, �������� CRC ����� ��������� ���������
void search_ID(void)
{


    unsigned char i, j = 0;// ���������� ��������
    unsigned char n = 0;  //���������� ��������, ���������� � �����
    unsigned char data [8];// �����-������ ��� �������� ����� ��������
    unsigned char u;//���������� ��� �������� � ����� �������� CRC8
    unsigned char data_crc;// ����������, ������� ������������� ���� ������ � ����� �������� CRC8
    unsigned char crc8 = 0; //���������� ��� ���������� CRC8 � ����� �������� CRC8

    sei();// ���� SREG |= (1 << 7); ��������� ����� �����������
    if (n == 0) //����������, ������ ����� � ���� ������ ���� ��������
    {
        unsigned char p = 1;  //���������� ��� ����� ����������� ����
        unsigned char bit = 0x01;// ��������� ������� ����
        unsigned char New_conflict = 0; //���������� ��� ���� ������� ����
        unsigned char last_conflict = 0;  //���������� ��� ������ ������� ����

        for (j = 0; j < 8; j++) //������� �����-������
        {
            data[j] = 0x00;
        }
        j = 0;//�������� ������� �����-�������
        unsigned char bit1, bit2; //��� ��������� ����� ���� ���� ������

        do
        {
            New_conflict = 0;
            n++;
            if (!init_device())	// ����� ������� ������ � ��������� ����� ��������
            {
                // ���� �-��� ������ ������ 0
                lcd_com(0x80); // ������� ������ � 1-� ������� ����� ������� 1 ������ ������
                send_string_to_LCD (absence);//������� "��� ��������"
                abort ();
            }
            send_command(0xF0); // ������� ������
            // ������ ������ � ������� ������ ���������� �������� ��� ������� �������

            while (p <= 64) // ���� �� ����� ��������� ��� 64 ����
            {
                bit1 = read_data();//������ ����-����
                // _delay_us(2); // ��� ���� �������� ������ �� ��������
                bit2 = read_data();//������ ����-����
                if (bit1 && bit2) // ���������� ���������� ���� , ���� ��� �������
                {
                    lcd_com(0x80); // ������� ������ � 1-� ������� ����� ������� 1 ������ ������
                    send_string_to_LCD (no_answer);//������� "��� ������ ����."
                    abort ();
                }
                else if ((bit1) && (!bit2))// ��� = 1
                    data[j] |= bit;//���������� ���
                else if((!bit1) && (bit2))// ��� = 0
                    data[j] &= ~bit;
                else if((!bit1) && (!bit2))//�������� ��� 0
                {
                    //����� ����� ���������� ������� �����,  � ������� ������� ��������� ���������  � ���������� N_conflict
                    if (p == last_conflict)//���� ������� ������� ���� � ������ ��������� �������� ==  ������� � ���������� ���������� (������ ����� 0-�� �������), �� ������� � ������ 1
                        data[j] |= bit;
                    else if (p > last_conflict)// ���� ����� ������� ������, ������ ����������� ������, �� ������� 0 � ����� ������� ��������� �������
                    {
                        data[j] &= ~bit;
                        New_conflict = p;
                    }
                        //���� ����� ������� ����� ������� ������, ������ ������� ����������� ����������(���� ����� ������� ���, ��� ���������, ��� ��������� ���������� �� ����� ��� �� ������
                        //��������� ����������� ����������,  �������� 0 , �� ��� ����� ��������� ����� ����������)
                    else if (!(data[j] & bit))
                    {
                        New_conflict = p;
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
            last_conflict = New_conflict;
            p = 1;

            location = dev_name;//�������� ������� ���������� ������������� ASCII ��� ����� �� ���������
            utoa (dev_num, (void*)&dev_name[3], 10);// � ��� �� ��������� ����������� ASCII ��� ������ ���������� �� ������� ������
            //���������� ������������ ����� ���������� � void*, �.�. utoa � strncpy �� ���� ������, ����� char*, � � ��� unsigned char*
            strncpy((void*)buffer.name, (void*)location, sizeof buffer.name); // ���������� ASCII ��� ����� � ���� name ������

            // ������ ID-���� 1-�� 1W ���������� � ID_string � ����������� ��� � �����-���������
            for (j=0; j < 8; j++)
            {
                buffer.code[j] = data[j];
            }
            /* ��������� ���� tmax � tmin ������ ���������� */
            buffer.tmax = 30;
            buffer.tmin = 6;
            eeprom_update_block (&buffer, &ee_arr[dev_num], sizeof(buffer)); // ���������� � ����� �������� �������� 1W ����������.
            dev_num ++; //������������� ����� ���������� - ��� ����� ���������

            if (last_conflict != 0)
            {
                j = 0;
                bit = 0x01;
            }
        }

        while (last_conflict != 0); // ���� ����� ���� ��������� �� ����� 0, ���� ����� �� ��� ������� �������

        lcd_com(0x01); // ������� �������
        _delay_us(1500);// ����� ���������� ������� �� ����� 1.5ms
        lcd_com(0x80);
        //send_string_to_LCD ((uint8_t *)strcpy_P(buffer, (PGM_P) present_n)); // ������� "�����.���."
        send_string_to_LCD (present_n); // ������� "�����.���."
        lcd_dat(n +'0');// ������� ���������� �������� �������� (�������� ���������� n �� ����� ������)
        _delay_ms(3000);
    }

    //������� �������, ������ ��������, ���������� ��������� ������������ ���������� ����������
    i = 0;//�������, ������ ��������� � 0-� ��������
    j = 0;
    while(i != n)//������������ ���������� ���������(����� n �� ����� ������), �������� � ������� ����������
    {
        eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer));// ��������� �������� ��������� �� �����
        crc8 = 0;
        for(j = 0; j < 7; j++)
        {
            unsigned char bit_crc; //��������� ����������
            data_crc = buffer.code[j];
            for (u = 0 ; u < 8; u++)
            {
                bit_crc = ((crc8 ^ data_crc) & 0x01);
                if (bit_crc == 0)
                    crc8 >>= 1;
                else
                {
                    crc8 ^= 0x18;//  11000 , �� ������ �.�. ��� ���0 � 1 ����� 1
                    crc8 >>= 1; //������� �����
                    crc8 |= 0x80;//+ 1000 0000
                }
                data_crc >>=1;
            }
        }
        if (crc8 == buffer.code[j]) // ���� ��������� ���� �������� ����� �� ������
            i++;
        else
        {
            lcd_com(0x80); // ������� ������ � 1-� ������� ����� ������� 1 ������ ������
            send_string_to_LCD (error);//������� "������ ����."
            lcd_dat((i+1)+'0');// ������� � ������� � ������� � ID
            abort ();
        }
    }

    lcd_com(0x01); // ������� �������
    _delay_us(1500);// ����� ���������� ������� �� ����� 1.5ms
    lcd_com(0x80); // ������� ������ � 1-� ������� ����� ������� 1 ������ ������
    send_string_to_LCD (init_n); //������� "����.���."
    lcd_dat(i+'0');//������� ���-�� ����-�, ��������� �������� CRC8
    _delay_ms(3000);
    lcd_com(0x01); // ������� �������
    _delay_us(1500);// ����� ���������� ������� �� ����� 1.5ms
    //lcd_com(0x80); // ������� ������ � 1-� ������� ����� ������� 1 ������ ������
    //send_string_to_LCD (tempC); //������� "����-�� "
    //eeprom_update_byte (&dev_qty, n);0x0400
    eeprom_update_byte ((uint8_t*)dev_qty, n);
}


//������ ������� ���������
int main(void)
{
    unsigned char temperature[2];	// ������ ������ ����������� LB � HB
    unsigned char temp_int; //����� ����� �����������
    unsigned char temp_float; // ������� ����� �����������
    unsigned int temp; // ��������� ���������� ��� �������� �� ��������������� ���� � ������ ��� "-" �����������

    // LCD_COM_PORT_DDR |= (1<<RS)|(1<<EN); //����� RS � EN ������, �������. ���� DAT � COM ������� �� ������ �����
    // LCD_COM_PORT = 0x00; // ������ 0 � RS � EN, ����������������� ���� ������� �� ������ �����
    LCD_DAT_PORT_DDR = 0xFF; // ���� ������ (� ������ � ������ ������) ��� - �� �����
    LCD_DAT_PORT = 0x00;// ������ 0 � ���� ������ � ������ ���
    _delay_ms(200); // �������� ���������� ���
    lcd_init(); // ������������� �������

    DDR_SRC_PORT &= ~_BV(SRC_PIN_NUM);//������ 0 - ��� ������ ����������� �� ����
    SRC_PORT |= _BV(SRC_PIN_NUM);//������ 1 - �����. �������� � +
    if (!(SRC_PIN & src_msk)) //���� �� ���� 0, ������ ������
    {
        search_ID();
    }

    while(1)
    {
        unsigned char j = 0;// ���������� ��������
        unsigned char i = 0;
        unsigned char n = 0;
        //n = eeprom_read_byte(&dev_qty);
        n = eeprom_read_byte((uint8_t*)dev_qty);


        for (i = 0; i< n; i++) //�������� � ������� ������� (�� 0)
        {
            eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer)); // ��������� �������� ��������� �� �����
            init_device();//������� ������ � �����������
            send_command(0x55);//������� ������������
            // ����� ��������� ��� ���������� � �������� ����������
            for (j = 0; j < 8 ; j++)
            {
                unsigned char data_byte; // ���������� ��� �������� ����
                data_byte = buffer.code[j];
                send_command (data_byte); //�������� ��������� ��� ����������
            }

            send_command (0x44);//������� ��������������
            while (!read_data()) ;// ����������� ���� ���� �� ����� �� ����������� 1 - �������������� ���������
            init_device();//������� ������ � �����������
            send_command(0x55);//������� ������������
            for (j = 0; j < 8 ; j++)   // ����� �������� ������ ����������, � �������� ����� ����������
            {
                unsigned char data_byte; // ���������� ��� �������� ����
                data_byte = buffer.code[j];
                send_command (data_byte); //�������� ��������� ��� ����������
            }
            send_command (0xBE);//������� ������ ������
            for (j = 0; j < 2; j++) //��������� ������ ��� ����� �����������
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

            /*
            //��������� ����������� ���������� ���������� �����������
            crc8 = 0;
            for(j = 0; j < 7; j++)
            {
                unsigned char bit_crc; //��������� ����������
                data_crc = temperature[j];
                for (u = 0 ; u < 8; u++)
                {
                    bit_crc = ((crc8 ^ data_crc) & 0x01);
                    if (bit_crc == 0)
                        crc8 >>= 1;
                    else
                    {
                        crc8 ^= 0x18;//  11000 , �� ������ �.�. ��� ���0 � 1 ����� 1
                        crc8 >>= 1; //������� �����
                        crc8 |= 0x80;//+ 1000 0000
                    }
                    data_crc >>=1;
                }
            }
            if (crc8 == temperature[j]) // ���� ��������� ���� �������� ������� ������
            {


            }
            else
            {
                Display(000);
                _delay_ms(1000);
            }
            */
            //
            if ((temperature[1]&0b10000000) == 0)
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
            /*
            ����������� � ����� ����� � *10 (����� ������ 1 ���������� ����),
            ���������� "." ����� ������� �������������
            1. temp_float = temp_float * 0.0625 -  ������ �������� �� 0.0625 ���
                 (temp_float >> 4)  - ����� �� 16 ��������
            2. (temp_float + temp_int)*10 - ������ �������� �� 10 ���
                ((((temp_float + temp_int)<<1) + (temp_float + temp_int)<<3)) - �������� �� 10 ��������
            */

            Disp_prep ((uint16_t)((temp_float*0.0625 + temp_int)*10), i, n); //���� �������� � uint8_t, ����� ���� �� float; ������� � ����-�� ��� �������. ������ ����.
            if ((i & 0x01)||(i == (n-1))) //���������� ������ ������ �� �������: � ����-� 0,2,4... �������, 1,3,5... ������;
                //���� ������ ������ - ������ �������� ����� ������ ������.
                //���� ������ �������, �� ���������� ��������� - ���� ��������
            {
                _delay_ms(2000);
            }
            temp_int = 0;
            temp_float = 0;
        }
    }	// ����������� ������ ������������ �����
}      // ����������� ������ �������� ���������
