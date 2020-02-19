#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h> // ��� abort()
#include <avr/pgmspace.h> //�����������������, ���� ������� ������ � ������ �� ����
// #include <math.h> �����������������, ���� ����� �-� modf � �-�� �������������� ��� � ����� ��� �������� � ���

#define RS PORTD3 //  ����� ������ �����, �� �������� ��������� ������� RS � ���
#define EN PORTD2 //  ����� ������ �����, �� �������� ��������� ������� EN � ���
#define LCD_COM_PORT PORTD // ���� ��� ������ ������ � ��� (������� � ������ - ����� ���� �� �� ����� �����!!!)
//#define LCD_COM_PORT_DDR DDRD // ������� ����������� ������ � �����, ���� ���������� ����� ������ ���, ��.����
#define LCD_DAT_PORT PORTD // ���� �������� ������ (� ������ � ������ ������)� ���
#define LCD_DAT_PORT_DDR DDRD // ������� ����������� ������ �����, ���� ��������� ��� ������� ������ (� ������ � ������ ������)
/* ���-����� ��������� ������ � �������� �� � ���� LCD_DAT_PORT ������������ �������������� ������� �������, � �������
��������� ���. � ������ ������ ������������ ������ 4-7 �����. */
#define DDR_OW_PORT DDRB // ������� ����������� ������ �����, � ������ �� ������� �������� ���������� ����� 1-Wire
#define OW_PORT PORTB // ����, � ������ �� ������� �������� ���������� ����� 1-Wire
#define OW_PIN PINB // ������� ����� ������ ����� 1-Wire, � ������ �� ������� �������� ���������� ��� �����
#define OW_PIN_NUM 0 // ����� PIN, � �������� ���������� ����� 1-Wire, ��� ������� _BV()
#define bit_msk 0x01 // ������� ����� ��� �������� ������� �� ����� 1-Wire �� ��������������� ����
#define D 2 //������������ ���������� ��������, ������� ����� ���� ����������.

const unsigned char tempC[] PROGMEM = "����-�� ";
const unsigned char absence[] PROGMEM = "��� ��������";
const unsigned char error[] PROGMEM = "������ ����.";
const unsigned char present_n[] PROGMEM = "�����.���. ";
const unsigned char init_n[] PROGMEM = "����.���. ";
const unsigned char no_answer[] PROGMEM = "��� ������ ����. ";
//char buffer [20];
unsigned char temp_sign; // ������� ����� ����������� ��� �-��� ������ �� ���


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
static uint8_t lcd_rus(uint8_t c)
{
    if  (c > 191)
    {
        c -= 192;
        c = pgm_read_byte (&(convert_HD44780[c]));
    }
    return c;
}

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

// ������� ������ ������ �� ���
void send_string_to_LCD (const unsigned char *s)
{
    while(pgm_read_byte (s))
    {
        lcd_dat(lcd_rus(pgm_read_byte(s++)));
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
    lcd_com(0x80); // ����� �������� ������ � 1-� ������� ����� ������� 1 ������ ������
}

// ������� �������� ����� �� ������������ ����� Number, � ������������ ����� ������ ��� ���������� ���������
void Display (unsigned int Number)
{

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
    lcd_com(0x88); // ������� ������ � 1-� ������� ����� ������� 1 ������ ������
    if (!temp_sign)
    {lcd_dat('+');}
    else
    {lcd_dat('-');}
    lcd_dat(Num1 + '0');
    lcd_dat(Num2 + '0');
    lcd_dat('.');
    lcd_dat(Num3 + '0');

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

//������ ������� ���������
int main(void)
{
    // LCD_COM_PORT_DDR |= (1<<RS)|(1<<EN); //����� RS � EN ������, �������. ���� DAT � COM ������� �� ������ �����
    // LCD_COM_PORT = 0x00; // ������ 0 � RS � EN, ����������������� ���� ������� �� ������ �����
    LCD_DAT_PORT_DDR = 0xFF; // ���� ������ (� ������ � ������ ������) ��� - �� �����
    LCD_DAT_PORT = 0x00;// ������ 0 � ���� ������ � ������ ���
    _delay_ms(200); // �������� ���������� ���
    lcd_init(); // ������������� �������

    unsigned char j = 0;// ���������� ��������
    unsigned char i = 0;
    unsigned char n = 0;  //���������� �������, ��� ����� ������
    unsigned char data [D][8];// ������ ��� �������� ����� ��������
    unsigned char u;//���������� ��� �������� � ����������� ����
    unsigned char data_crc;// ����������, ������� ������������� ���� ������
    unsigned char crc8 = 0; //���������� ��� ����������� ����
    unsigned char temperature[2];	// ������ ������ ����������� LB � HB
    unsigned char temp_int; //����� ����� �����������
    unsigned char temp_float; // ������� ����� �����������
    unsigned int temp; // ��������� ���������� ��� �������� �� ��������������� ���� � ������ ��� "-" �����������

    sei();// ���� SREG |= (1 << 7); ��������� ����� �����������
    if (n == 0) //����������, ������ ����� � ���� ������ ���� ��������
    {
        unsigned char p = 1;  //���������� ��� ����� ����������� ����
        unsigned char bit = 0x01;// ��������� ������� ����
        unsigned char New_conflict = 0; //���������� ��� ���� ������� ����
        unsigned char last_conflict = 0;  //���������� ��� ������ ������� ����

        for (i = 0; i < (D); i++) //������� ������
        {
            for (j = 0; j < 8; j++)//8-� ����
            {
                data[i][j] = 0x00;
            }
        }
        j = 0;//��������
        i = 0;//��������
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
                    data[i][j] |= bit;//���������� ���
                else if((!bit1) && (bit2))// ��� = 0
                    data[i][j] &= ~bit;
                else if((!bit1) && (!bit2))//�������� ��� 0
                {
                    //����� ����� ���������� ������� �����,  � ������� ������� ��������� ���������  � ���������� N_conflict
                    if (p == last_conflict)//���� ������� ������� ���� � ������ ��������� �������� ==  ������� � ���������� ���������� (������ ����� 0-�� �������), �� ������� � ������ 1
                        data[i][j] |= bit;
                    else if (p > last_conflict)// ���� ����� ������� ������, ������ ����������� ������, �� ������� 0 � ����� ������� ��������� �������
                    {
                        data[i][j] &= ~bit;
                        New_conflict = p;
                    }
                        //���� ����� ������� ����� ������� ������, ������ ������� ����������� ����������(���� ����� ������� ���, ��� ���������, ��� ��������� ���������� �� ����� ��� �� ������
                        //��������� ����������� ����������,  �������� 0 , �� ��� ����� ��������� ����� ����������)
                    else if (!(data[i-1][j] & bit))
                    {
                        New_conflict = p;
                        data[i][j] &= ~bit;
                    }
                    else
                    {data[i][j] |= bit;}
                }

                // ����� ������� ��������������� ���, ������� ��� ��������� ���������� ������� ��������������� ����������
                if(data[i][j] & bit)
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
            if (last_conflict != 0)
            {
                i++;
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
        _delay_ms(4000);
    }

    //������� �������, ������ ��������, ���������� ��������� ������������ ���������� ����������
    i = 0;//�������, ������ ��������� � 0-� ��������
    j = 0;
    while(i != n)//������������ ���������� ���������(����� n �� ����� ������), �������� � ������� ����������
    {
        crc8 = 0;
        for(j = 0; j < 7; j++)
        {
            unsigned char bit_crc; //��������� ����������
            data_crc = data[i][j];
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
        if (crc8 == data[i][j]) // ���� ��������� ���� �������� ����� �� ������
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
    lcd_dat(i+'0');
    _delay_ms(4000);
    lcd_com(0x01); // ������� �������
    _delay_us(1500);// ����� ���������� ������� �� ����� 1.5ms
    lcd_com(0x80); // ������� ������ � 1-� ������� ����� ������� 1 ������ ������
    send_string_to_LCD (tempC); //������� "����-�� "

    while(1)
    {
        for (i = 0; i< n; i++) //�������� � ������� �������
        {
            init_device();//������� ������ � �����������
            send_command(0x55);//�������� ������������
            // ����� ��������� ��� ���������� � �������� ����������

            for (j = 0; j < 8 ; j++)
            {
                unsigned char data_byte; // ���������� ��� �������� ����
                data_byte = data[i][j];
                send_command (data_byte); //�������� ��������� ��� ����������
            }

            send_command (0x44);//�������� ��������������
            while (!read_data()) ;// ����������� ���� ���� �� ����� �� ����������� 1 - �������������� ���������
            init_device();//������� ������ � �����������
            send_command(0x55);//�������� ������������
            for (j = 0; j < 8 ; j++)   // ����� �������� ������ ����������� �������� ����� ����������
            {
                unsigned char data_byte; // ���������� ��� �������� ����
                data_byte = data[i][j];
                send_command (data_byte); //�������� ��������� ��� ����������
            }
            send_command (0xBE);//�������� ������ ������
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

            Display(((uint8_t)(temp_float*0.0625) + temp_int)*10);	// ���� �������� � uint8_t, ����� ���� �� float
            _delay_ms(2000);
            temp_int = 0;
            temp_float = 0;
        }
    }	// ����������� ������ ������������ �����
}      // ����������� ������ �������� ���������
