#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <string.h>
// #include <math.h> раскомментировать, если нужна ф-я modf в ф-ии преобразования ЧПТ в цифры для передачи в ЖКИ

//блок define для LCD
#define RS PORTD3						// Номер вывода порта, по которому передаётся команда RS в ЖКИ
#define EN PORTD2						// Номер вывода порта, по которому передаётся команда EN в ЖКИ
#define LCD_COM_PORT PORTD				// Порт для посыла команд в ЖКИ (команды и данные - могут быть не на одном порту!!!)
//#define LCD_COM_PORT_DDR DDRD 		// Регистр направления данных в порту, куда подцеплены линии команд ЖКИ, см.выше
#define LCD_DAT_PORT PORTD				// Порт отправки данных (и команд в данном случае)в ЖКИ
#define LCD_DAT_PORT_DDR DDRD			// Регистр направления данных порта, куда подцеплен ЖКИ линиями данных (и команд в данном случае)
/* бит-маски выделения ниблов и отправки их в порт LCD_DAT_PORT составляются соответственно номеров выводов, к которым
подключен ЖКИ. в данном случае используются выводы 4-7 порта. */

//блок define для OW
#define DDR_OW_PORT DDRB				// Регистр направления данных порта, к одному из выводов которого подключена линия 1-Wire
#define OW_PORT PORTB					// Порт, к одному из выводов которого подключена линия 1-Wire
#define OW_PIN PINB						// Регистр приёма ответа линии 1-Wire, к одному из выводов которого подключена эта линия
#define OW_PIN_NUM 0					// Номер PIN, к которому подключена линия 1-Wire, для макроса _BV()
#define bit_msk (1<<OW_PIN_NUM) 		// Битовая маска для проверки сигнала от линии 1-Wire на соответствующем пине

//блок разных define
#define N_DIGS 5						// размер массива цифр для преобразовании числа в строку для LCD, определяется размерностью максимального выводимого на LCD числа +1 для завершающего 0
#define N_NAME 8						//размер массива для имени устройства
#define EEP_MEM 1024					// объём епром для вычисления адреса последней и предпоследней ячейки, также для определения максимального количества устройств в епром
#define IND_PAUSE 120 					// пауза на индикацию очередного кадра, 2с
#define WAIT_LIM 900 					//пауза для ожидания действий пользователя, 15с

//блок define для инициализации кнопки
#define SRC_PORT PORTB					// Порт, к одному из выводов которого подключена кнопка нового поиска (здесь порт один с OW line)
#define DDR_SRC_PORT DDRB				// Регистр направления данных порта, к одному из выводов которого подключена кнопка нового поиска (здесь порт один с OW line)
#define SRC_PIN PINB					// Регистр приёма состояния кнопки
#define SRC_PIN_NUM 1					// Номер PIN, к которому подключена кнопка, для макроса _BV()
#define src_msk (1<<SRC_PIN_NUM)		// Битовая маска для проверки сигнала от кнопки на соответствующем пине

//блок define для опроса кнопки
#define CNT_QUICK 5						// количество сканирований на антидребезг, если больше - нажато точно
#define CNT_SLOW 15						// количество сканирований на длинное нажатие
#define QUICK 1							// длительность короткого нажатия
#define SLOW 2							// длительность длинного нажатия
#define RELEASED 0						// состояние кнопки - отпущена
#define PRESSED 1						// состояние кнопки - нажата

// блок define для светодиода индикации gsm уровня
#define LED_PORT PORTB					// Порт, к одному из выводов которого подключен светодиод (здесь порт один с OW line)
#define DDR_LED_PORT DDRB				// Регистр направления данных порта, к одному из выводов которого подключен светодиод (здесь порт один с OW line)
#define LED_PIN_NUM 5					// Номер PIN, к которому подключен светодиод, для макроса _BV()

// блок текстовых сообщений во флэш
unsigned char absence[] PROGMEM = "Нет датчиков";				// ошибка на этапе первой инициализации датчиков перед входом в алгоритм дешифрации адресов
unsigned char no_answer[] PROGMEM = "Нет ответа датч.";			// ошибка на старте дешифрации адресов после чтения первых двух тайм-слотов
unsigned char present_n[] PROGMEM = "Найдено датч. ";			// количество устройств, опознанных при первичной дешифрации адресов
unsigned char dev_excess[] PROGMEM = "Много датчиков";			// ошибка в процессе первичной дешифрации адресов, если количество датчиков превысит 50
unsigned char error[] PROGMEM = "Ошибка CRC-ID ";				// ошибка на этапе проверки CRC ID после первичной дешифрации адресов, выводится № датчика,
unsigned char init_n[] PROGMEM = "Исправн.датч. ";				// количество датчиков, прошедших проверку CRC после первичной дешифрации адресов
unsigned char init_srch[] PROGMEM = "Выполнить поиск?";			// ошибка при отсутствии в епром данных датчиков, если не проводилась первичная дешифрация адресов
unsigned char scratch_err[] PROGMEM = "Ош.CRC-блкн. ";			// ошибка при проверке CRC данных, прочитанных из блокнота, выводится № датчика
unsigned char no_answer_n[] PROGMEM = "Нет ответа ";			// ошибка на этапе чтения чтения блокнота при перекличке в main - если датчик не ответил, то конф.байт == FF, выводится имя датчика
unsigned char ow_check[] PROGMEM = "Опрос линии";				// сообщение в main при нормальном входе во время переклички
unsigned char correction[] PROGMEM = "Корректировка";			// сообщение в add_ID при входе
unsigned char subst_add[] PROGMEM = "Замена/добавл-е";			// сообщение в add_ID при входе в блок замены/удаления
unsigned char total_qty[] PROGMEM = "Всего устройств";			// сообщение в add_ID перед началом поиска
unsigned char plug_in[] PROGMEM = "Подключите устр.";			// сообщение в add_ID перед началом поиска
unsigned char press_btn[] PROGMEM = "и нажмите кнопку";			// сообщение в add_ID перед началом поиска
unsigned char mem_full[]PROGMEM = "Память заполнена";			// сообщение в add_ID, если количество датчиков уже 50, а неактивных нет
unsigned char new_dev_fnd[] PROGMEM = "Найдено новое";			// верхняя строка сообщения в add_ID, если найдено новое
unsigned char nev_dev_add[] PROGMEM = "Добавлено новое";		// верхняя строка сообщения в add_ID в конце блока добавления
unsigned char element[] PROGMEM = "устройство";					// нижняя строка сообщения в add_ID, если найдено или добавлено новое
unsigned char substitute[] PROGMEM = "Заменить?";				// сообщение в add_ID в блоке замены, если нашли неактивное устр-во, выводится имя датчика
unsigned char add_to_end[] PROGMEM = "Добавить?";				// сообщение в add_ID в начале блока добавления
unsigned char done[] PROGMEM = "Замена выполнена";				// сообщение в add_ID после вставки взамен неактивного
unsigned char no_new[] PROGMEM = "Новых устр-в нет";			// сообщение в add_ID если прошли весь цикл поиска ноовых ID
unsigned char delete[] PROGMEM = "Удалить?";					// сообщение - первый пункт меню Корректировка
unsigned char del_done[] PROGMEM = "Удалено";					// сообщение по факту удаления устройства
unsigned char t_error[] PROGMEM = "Неверная темп-ра";			// сообщение при попытке записать некорректную температуру в датчик
unsigned char com_error[] PROGMEM = "Неверная команда";			// сообщение при ошибке в тексте команды
unsigned char name_error_ren[] PROGMEM = "Неверное имя REN";	// сообщение при ошибкочном имени в тексте команды
unsigned char name_error_al[] PROGMEM = "Неверное имя AL";		// сообщение при ошибкочном имени в тексте команды
unsigned char name_error_sms[] PROGMEM = "Неверное имя SMS";	// сообщение при ошибкочном имени в тексте команды
unsigned char t_min[] PROGMEM = "Tmin";							// часть строки при подтверждении изменения минимального порогоа температуры
unsigned char t_max[] PROGMEM = "Tmax";							// часть строки при подтверждении изменения максимального порога температуры
unsigned char t_low[] PROGMEM = "T<";							// часть строки при сообщениии о снижении температуры ниже минимального порога
unsigned char t_high[] PROGMEM = "T>";							// часть строки при сообщениии о повышении температуры выше максимального порога
unsigned char sms_send []PROGMEM = "SMS";						// часть строки при подтверждении включения sms о выходе температуры за пределы
unsigned char on []PROGMEM = "ВКЛ.";
unsigned char off []PROGMEM = "ОТК.";
unsigned char blank []PROGMEM = " ";
unsigned char crash []PROGMEM = "АВАРИЯ!";

// блок глобальных переменных и структур
typedef struct //битовое поле для флагов
{
    uint8_t active : 1;		// признак активности на линии
    uint8_t lt_alarm : 1;	// нижний порог температуры пройден вниз
    uint8_t ht_alarm : 1;	// верхний порог температуры превышен
    uint8_t line_alarm : 1;	// признак пропадания в процессе измерений
    uint8_t sms_T : 1;		// отправить sms при Т < tmin или Т > tmax
    uint8_t relay_1 : 1;	// реле 1
    uint8_t relay_2 : 1;	// реле 2
    uint8_t reserved : 1;	// свободный
} bit_set;

typedef struct //структура для параметров устройства
{
    unsigned char name[N_NAME];		// имя 1W устройства
    unsigned char code[8];			// код 1W устройства
    char tmax;						// максимальная Т
    char tmin;						// минимальная Т
    bit_set flags;					// флаги состояния
} device;							// структура для параметров 1W устройства

/*typedef union //объединение для параметров устройства
{
	unsigned char name[N_NAME]; // имя 1W устройства
	unsigned char code[8]; // код 1W устройства
	char tmax; // максимальная Т
	char tmin; // минимальная Т
	bit_set flags;// флаги состояния
} dev_param;*/

device buffer;																// переменная для обмена озу <-> епром
device ee_arr [2] EEMEM;													// oбъявляем массив структур в епром, 2 - пока временно, для начала
unsigned char *location;													// указатель на имя 1W устройства, задаваемого юзером вместо имени по умолчанию
unsigned char dev_name[N_NAME] = {'S', 'e', 'n', 's', '.', '_', '_', '_'};	// "Dxxxxxx" - имя устройства по умолчанию, к нему добавится ASCII код порядкового номера
const uint16_t dev_qty = (EEP_MEM - 1);										// константа, содержащая значение адреса последней ячейки в епром для переменной количества дешифрованных устройств
const uint16_t dev_last_n = (EEP_MEM - 2);									// константа, содержащая значение адреса предпоследней ячейки в епром для переменной последнего номера устройства, добавленного в список в еепром
/*нужна для корректного продолжения автоматической нумерации в именах при добавлении в интерактивном режиме после удаления устройств через меню интерактивного удаления */
const uint8_t n_max = (uint8_t)((EEP_MEM - 2) / sizeof(device));			// максимально допустимое количество усройств в епром
uint8_t scratchpad [9];														// массив байтов, прочитанных из блокнота DS18B20



volatile uint8_t flash_cnt;		// счётчик прерываний для индикации уровня gsm вспышками диода
volatile uint8_t flash_num;		// число прошедших вспышек
volatile uint8_t pause_num;		// число прошедших отрезков пауз между сериями вспышек
const uint8_t gsm_lvl = 6;		// уровень gsm сигнала, пока const


typedef struct // структура для строки дисплея, из них строится кадр (экран)
{
    uint8_t name[N_NAME];	// имя устройства
    unsigned char dig_1;	// первая цифра температуры
    unsigned char dig_2;	// вторая цифра температуры
    unsigned char dig_3;	// десятичная цифра температуры
    unsigned char sign;		// знак температуры
} line;
line line_up; // верхняя строка кадра
line line_dn; // нижняя строка кадра

volatile uint16_t btn_cnt = 0;				// счётчик считываний состояния кнопки
volatile uint8_t btn_state = 0;				// состояние кнопки - нажата/отпущена
volatile uint8_t btn_time = 0;				// время нажатия кнопки, действует только в обработчике прерывания на сканирование, но можно использовать снаружи для перехода по длинному жиму до отпускания
volatile uint8_t press_time = 0;			// это время нажатия кнопки отдаётся наружу
volatile uint16_t int_cnt = 0;				// счётчик переполнения таймера TIMER0, чтобы делать опрос кнопки на произвольной частоте
volatile uint8_t delay_cnt = IND_PAUSE;		// таймер задержки на индикацию после отправки кадра в lcd
volatile uint16_t wait_timer = WAIT_LIM + 1;// таймер ожидания действий пользователя остановлен, +1 чтобы при первом входе в меню не сработала проверка if(wait_timer==WAIT_LIM)
//volatile uint16_t time_gsm = 0;			// счётчик для запросов gsm-модуля


// Функция записи команды в ЖКИ
void lcd_com(unsigned char p)
{
    LCD_COM_PORT &= ~(1 << RS);							// RS = 0 (запись команд)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p & 0xF0); 	// Выделяем старший нибл
    LCD_COM_PORT |= (1 << EN);  						// EN = 1 (начало записи команды в LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN); 						// EN = 0 (конец записи команды в LCD)

    LCD_COM_PORT &= ~(1 << RS); 						// RS = 0 (запись команд)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p << 4); 	// Выделяем младший нибл
    LCD_COM_PORT |= (1 << EN); 							// EN = 1 (начало записи команды в LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN); 						// EN = 0 (конец записи команды в LCD)
    _delay_us(50);
}

// Функция записи данных в ЖКИ, выводит символы на lcd
void lcd_dat(unsigned char p)
{
    LCD_COM_PORT |= (1 << RS);							// RS = 1 (запись данных)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p & 0xF0);	// Выделяем старший нибл
    LCD_COM_PORT |= (1 << EN);							// EN = 1 (начало записи команды в LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN);							// EN = 0 (конец записи команды в LCD)

    LCD_COM_PORT |= (1 << RS);							// RS = 1 (запись данных)
    LCD_DAT_PORT &= 0x0F; LCD_DAT_PORT |= (p << 4);		// Выделяем младший нибл
    LCD_COM_PORT |= (1 << EN);							// EN = 1 (начало записи команды в LCD)
    _delay_us(2);
    LCD_COM_PORT &= ~(1 << EN);							// EN = 0 (конец записи команды в LCD)
    _delay_us(50);
}

//Таблица перекодировки в русские символы.
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

// функция перекодировки символов
static unsigned char lcd_rus(uint8_t c)
{
    if  (c > 191)
    {
        c -= 192;
        c = pgm_read_byte (&(convert_HD44780[c]));
    }
    return c;
}

// функция вывода строки на ЖКИ
void send_string_to_LCD (const unsigned char *s)
{
    while(pgm_read_byte (s))
    {
        lcd_dat(lcd_rus(pgm_read_byte(s++)));
        _delay_ms (1);
    }
}

// функция вывода массива символов на ЖКИ (в массиве должен быть символ "конца строки" 0x00)
void send_arr_to_LCD (unsigned char *s)
{
    while(*s)
    {
        lcd_dat(lcd_rus (*s++));
        _delay_ms (1);
    }
}

// Функция инициализации ЖКИ
void lcd_init(void)
{
    // Активизация четырехразрядного режима
    lcd_com(0x28);		// 4 бит режим 2 строки, 5х8
    lcd_com(0x08);		// выключение экрана, выключение отображения курсора, курсор не мигает 0000 1000(bin)
    lcd_com(0x06);		// инкрементирование, сдвиг всего экрана отключен 0000 0110(bin)
    lcd_com(0x01);		// очистка дисплея
    _delay_us(3000);	// время выполнения очистки не менее 1.5ms
    lcd_com(0x0C);		// включение экрана 0000 1100(bin)
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

void str_clr(uint8_t place)	//очистка строки через ввод пробелов
{
    switch(place)
    {
        case 0: lcd_com(0x80);	//первая строка
            break;
        case 1: lcd_com(0xC0);	//вторая строка
            break;
        default: lcd_com(0x80);
    }
    for (uint8_t i=0; i<16; i++)
    {lcd_dat(0x20);}			//печатаем пробелы
}

// функция выводит знаки на LCD, вызывается из Frame
void Display (uint8_t line_qty)
{
    send_arr_to_LCD_XY (line_up.name, 0, 0);		// выводим имя устройства в 0-ю позицию 1 строки экрана
    lcd_dat_XY(line_up.sign, 11, 0);				// выводим температуру в 11-ю (от 0) позицию 1 строки экрана
    if (line_up.dig_1 != '0')						// ведущие нули, кроме одного перед десятичной точкой не нужны
        lcd_dat(line_up.dig_1);
    lcd_dat(line_up.dig_2);
    lcd_dat('.');
    lcd_dat(line_up.dig_3);

    if (line_qty)									// выводим вторую строку, если она есть
    {
        send_arr_to_LCD_XY (line_dn.name, 0, 1);	// выводим имя устройства в 0-ю позицию 2 строки экрана
        lcd_dat_XY(line_dn.sign, 11, 1);			// выводим температуру в 11-ю (от 0) позицию 2 строки экрана
        if (line_dn.dig_1 != '0')					// ведущие нули, кроме одного перед десятичной точкой не нужны
            lcd_dat(line_dn.dig_1);
        lcd_dat(line_dn.dig_2);
        lcd_dat('.');
        lcd_dat(line_dn.dig_3);
    }
}

// Функция подготовки кадра (вызывается из main), заполняет массив строк для индикации на LCD
void Frame (uint8_t Dig1, uint8_t Dig2, uint8_t Dig3, uint8_t sign, uint8_t i, uint8_t n)
//определяем строку LCD: i это № устр-в: 0,2,4... верхняя, 1,3,5... нижняя
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

// Функция инициализации датчиков
unsigned char init_device(void)
{
    unsigned char OK_Flag = 0;			// переменная результата опроса присутствия датчиков
    // unsigned char Flag = 0;
    cli();
    OW_PORT &= ~_BV(OW_PIN_NUM);		// в порту ставим 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);		// 1 - порт на выход
    _delay_us(480);						// задержка 480 мкс
    //«отпускает линию»
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);	// 0 - порт на вход
    _delay_us(70);						// задержка 70 мкс ,берем чуть больше после перепада
    if (!(OW_PIN & bit_msk))			// проверяем факт ответа датчиков, если PINB==0 - ответ есть
        OK_Flag = 1;					// если ответ есть
    else
        OK_Flag = 0;					// если ответа нет
    _delay_us(410);					// остальная задержка 410 для окончания импульса присутствия
    sei();								// разрешаем прерывания
    return OK_Flag;
}
// функция отправки 1 в линию
void send_1 (void)
{
    cli();								// Запретим общие прерывания
    OW_PORT &= ~_BV(OW_PIN_NUM);		// в порту ставим 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);		// 1 - порт на выход
    _delay_us(6);						// задержка 15 мкс
    //«отпускает»
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);	// 0 - порт на вход
    _delay_us(64);						// задержка 45 мкс ,берем чуть больше
    sei();								// разрешаем прерывания
}
// функция отправки 0 в линию
void send_0(void)
{
    cli(); 								// Запретим общие прерывания
    OW_PORT &= ~_BV(OW_PIN_NUM);		// в порту ставим 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);		// 1 - порт на выход
    _delay_us(60);						// задержка 120 мкс
    //«отпускает»
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);	// 0 - порт на вход
    _delay_us(10);						// задержка 1 мкс ,перед записью следующего бита
    sei();								// разрешаем прерывания
}

//Функция выдачи команд на датчики
void send_command (unsigned char command)
{
    unsigned char i;
    for (i=0; i < 8; i++)
    {
        if (command & 0x01)	// если позиция бита 1, то передаем 1
        {
            send_1 ();
        }
        else				//передаем 0
        {

            send_0 ();
        }
        command >>= 1;		//сдвигаем вправо для обработки следующего бита
    }
}

//Функция чтения из датчиков
unsigned char read_data(void)
{
    unsigned char bit;
    cli();							// Запретим общие прерывания
    OW_PORT &= ~_BV(OW_PIN_NUM);	// в порту ставим 0
    DDR_OW_PORT |= _BV(OW_PIN_NUM);	// 1 - порт на выход
    _delay_us(6);					// задержка 2 мкс
    //«отпускает», управление передается датчику
    DDR_OW_PORT &= ~_BV(OW_PIN_NUM);// 0 - порт на вход
    _delay_us(9);					// задержка 9 мкс ,перед считыванием
    bit = (OW_PIN & bit_msk);
    _delay_us(55);					// задержка 50 мкс ,перед чтением следующего бита
    sei();							// разрешаем прерывания
    return bit;
}

// Проверка CRC8
int8_t CRC_check(uint8_t *data, uint8_t crcbitN)
{
    uint8_t j;						// счётчик байтов в массиве data, котоый проверяем
    uint8_t crc8 = 0;				// переменная для сравнивания результата расчёта CRC8 по младшим (crcbitN-1) байтам с CRC8 из crcbitN байта
    uint8_t data_crc;				// сюда в начале цикла сдвигов пишем каждый очередной байт из полученного из main массива
    uint8_t u;						// счётчик битов при сдвигах в алгоритме расчёта CRC8
    for(j = 0; j < crcbitN; j++)
    {
        unsigned char bit_crc;		// локальная переменная
        data_crc = data[j];
        for (u = 0 ; u < 8; u++)
        {
            bit_crc = ((crc8 ^ data_crc) & 0x01);
            if (bit_crc == 0)
                crc8 >>= 1;
            else
            {
                crc8 ^= 0x18;		//  11000 , по модулю т.е. там где 0 и 1 будут 1
                crc8 >>= 1;			// сдвгаем влево
                crc8 |= 0x80;		// + 1000 0000
            }
            data_crc >>=1;
        }
    }
    if (crc8 == data[j])			// если последний байт и CRC равны - хорошо
        return 0;
    else
        return 1;
}

// Функция дешифрации очередного ID-кода при опросе массива датчиков
void find_ID (uint8_t* data, uint8_t* New_conflict, uint8_t* last_conflict)
{
    uint8_t p = 1; 									// переменная для цикла считыввания кода
    uint8_t bit = 0x01;								// начальная позиция бита в маске
    uint8_t bit1, bit2;								// для сравнения битов двух тайм слотов
    uint8_t j = 0;									// счетчик байтов буфер-строки

    if (!init_device())								// выдаём импульс сброса и проверяем ответ датчиков
    {
        // если ф-ция сброса вернёт 0
        send_string_to_LCD_XY (absence, 0, 0);		// выводим "нет датчиков" в 1-ю верхнюю левую позицию 1 строки экрана
        abort ();
    }
    send_command(0xF0);								// команда поиска
    while (p <= 64)									// пока не будут прочитаны все 64 бита
    {
        bit1 = read_data();							// первый тайм-слот
        // _delay_us(2);							// без этой задержки иногда не работает
        bit2 = read_data();							// второй тайм-слот
        if (bit1 && bit2)							// сравниваем полученные биты , если обе единицы
        {
            send_string_to_LCD_XY (no_answer, 0 ,0);// выводим "нет ответа датч." в 1-ю верхнюю левую позицию 1 строки экрана
            abort ();
        }
        else if ((bit1) && (!bit2))					// бит = 1
            data[j] |= bit;							// записываем бит
        else if((!bit1) && (bit2))					// бит = 0
            data[j] &= ~bit;
        else if((!bit1) && (!bit2))					// Конфликт оба 0
        {
            //здесь будем сравнивать позиции битов,  в номерах которых произошли конфликты  в переменной N_conflict
            if (p == *last_conflict)				// если текущая позиция бита в котром произошел конфликт ==  позиции в предыдущем исчеслении (скорей всего 0-ая позиция), то запишем в адресс 1
                data[j] |= bit;
            else if (p > *last_conflict)			// если номер позиции больше, номера предыдущего опроса, то запишем 0 и номер позиции конфликта обновим
            {
                data[j] &= ~bit;
                *New_conflict = p;
            }
                //если вдруг текущий номер позиции меньше, номера позиции предыдущего исчесления(Если вдруг текущий бит, при конфликте, при очередном исчеслении не дошел еще до номера
                //конфликта предыдущего исчесления,  содержит 0 , то это будет считаться новым конфликтом)
            else if (!(data[j] & bit))
            {
                *New_conflict = p;
                data[j] &= ~bit;
            }
            else
            {data[j] |= bit;}
        }

        // Далее запишем соответствующий бит, который при следующем исчеслении включит соответствующие устройства
        if(data[j] & bit)
        {
            send_1 ();
        }
        else			// передаем 0
        {
            send_0 ();
        }

        p++;			// увеличиваем на 1
        bit <<= 1;		// сдвигаем влево

        if (!bit)		// сдвиг проходит все 8 бит и значение равно 0
        {
            j++;
            _delay_ms(100);
            bit = 0x01;
        }

    }//выходит из цикла после обработки 64-х битов
    *last_conflict = *New_conflict;
}
// функция переводит целое число в коды символов его цифр путём быстрого деления сдвигами и сложениями
// принимает число и указатель на массив для кодов символов цифр этого числа
// в последний элемент массива ставит завершающий 0, дальше коды символов цифр от последней к первой
// возвращает указатель на первую значащую цифру преобразуемого цисла
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
        // умножаем на 0.8
        res.quot = value >> 1;
        res.quot += res.quot >> 1;
        res.quot += res.quot >> 4;
        res.quot += res.quot >> 8;
        res.quot += res.quot >> 16;
        uint32_t qq = res.quot;
        res.quot >>= 3;													// делим на 8
        res.rem = (uint8_t) (value - ((res.quot << 1) + (qq & ~7ul)));	// вычисляем остаток
        if(res.rem > 9)													// корректируем остаток и частное, если остаток >=10
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

// функция переводит символы числа и знака в число с учётом знака
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
// Функция поиска устройств, запись в еепром
void search_ID(void)
{
    unsigned char i, j = 0;				// переменные счетчики
    unsigned char n = 0;				// количество датчиков, записанных в епром
    unsigned char m = 0;				// количество датчиков, прошедших проверку CRC8
    unsigned char data [8];				// буфер-массив для хранения кодов датчиков
    unsigned char New_conflict = 0;		// переменная для новой позиции бита конфликта
    unsigned char last_conflict = 0;	// переменная для старой позиции бита конфликта
    uint8_t digits[N_DIGS];				// массив для передачи в utoa_fast_div для заполнения его кодами символов цифр

    for (j = 0; j < 8; j++)				// обнулим буфер-массив
    {
        data[j] = 0x00;
    }
    j = 0;								// обнуляем счётчик буфер-массива

    do
    {
        New_conflict = 0;
        n++;
        if (n > n_max)									// если число устройств превысило 50, не влезет в епром
        {
            send_string_to_LCD_XY (dev_excess, 0, 0);	// выводим "много датчиков" в 1-ю верхнюю левую позицию 1 строки экрана
            abort ();
        }

        find_ID (data, &New_conflict, &last_conflict);	// запрос на очередной проход для дешифрации следующего устройства

        location = dev_name;							// временно локации устройства присваивается ASCII код имени по умолчанию
        // utoa_fast_div((n-1), &dev_name[(sizeof dev_name)- N_DIGS]);// в имя по умолчанию добавляется ASCII код номера устройства по порядку поиска, от 0.
        for (j = 0; j < N_DIGS; j++)					// заполним массив символов кодами '0'
        {
            digits[j] = '0';
        }
        utoa_fast_div((n-1), digits);					// переводим номер устройства в символы
        for (j = 1; j < 4; j++)							// записываем 3 последние элемента digits в последние 3 элемента массива имени - это номер устройства
        {
            dev_name[N_NAME-j] = digits[N_DIGS-j];
        }
        strncpy((void*)buffer.name, (void*)location, sizeof buffer.name); // записываем ASCII код имени в поле name буфера
        /*приходится использовать явное приведение к void*, strncpy не жрёт ничего, кроме char*, а у нас unsigned char* */
        for (j=0; j < 8; j++)							// запись ID-кода 1-го 1W устройства в ID_string и копирование его в буфер-структуру
        {
            buffer.code[j] = data[j];
        }
        /* заполняем поля tmax, tmin и flags буфера значениями */
        buffer.tmax = 30;
        buffer.tmin = 6;
        buffer.flags.active = 1;	// датчик активен
        buffer.flags.lt_alarm = 0;
        buffer.flags.ht_alarm = 0;
        buffer.flags.line_alarm =0;
        buffer.flags.sms_T = 0;
        buffer.flags.relay_1 = 0;
        buffer.flags.relay_2 = 0;
        buffer.flags.reserved =0;
        cli();
        eeprom_update_block (&buffer, &ee_arr[n-1], sizeof(buffer)); // записываем в епром описание текущего 1W устройства.
        sei();
    }
    while (last_conflict != 0);								// пока номер бита конфликта не равен 0, если равен то все датчики найдены

    lcd_clr(); 												// очистка дисплея
    send_string_to_LCD_XY (present_n, 0, 0);				// Выводим "Найдено датч. "
    send_arr_to_LCD_XY (utoa_fast_div (n, digits), 14, 0);	// выводим количество устр-в, utoa_fast_div отдаёт указатель на первый ненулевой символ массива digits
    _delay_ms(2500);

    //Датчики найдены, адреса записаны, необходимо проверить правильность переданной информации
    i = 0;			// обнулим, начнем проверку с 0-х индексов
    j = 0;
    m = n;			// начальное значение колич. датчиков для счётчика прошедших проверку CRC
    while(i != n)	// обрабатываем количество устройств(равно n из цикла поиска), начинаем с первого устройства
    {
        eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer));// считываем описание усройства из епром
        if (CRC_check (buffer.code, 0x07))						// передаём указатель на массив с ID, номер байта CRC8; если вернётся 1, CRC не ОК
        {
            m--;
            buffer.flags.active = 0;							// сброс флага присутствия
            send_string_to_LCD_XY (error, 0, 0);				// выводим "Ошибка CRC-ID" в 1-ю верхнюю левую позицию 1 строки экрана
            send_arr_to_LCD_XY (buffer.name, 0, 1);
            _delay_ms(2000);
        }
        i++;
    }

    lcd_clr();												// очистка дисплея
    send_string_to_LCD_XY (init_n, 0 ,0);					// выводим "Исправн.датч. " в 1-ю верхнюю левую позицию 1 строки экрана
    send_arr_to_LCD_XY (utoa_fast_div (m, digits), 14, 0);	// выводим количество устр-в, utoa_fast_div отдаёт указатель на первый ненулевой символ массива digits
    _delay_ms(2500);
    lcd_clr();												// очистка дисплея
    cli();
    eeprom_update_byte ((uint8_t*)dev_qty, n);
    eeprom_update_byte ((uint8_t*)dev_last_n, (n-1));
    sei();
}

// функция чтения блокнота датчика
uint8_t scratchpad_rd (void)
{
    uint8_t j = 0;							// счетчик байт блокнота
    uint8_t p;								// счётчик 72 битов для цикла считывания  блокнота
    uint8_t bit = 0x01;						// начальная позиция бита для формирования байта блокнота
    uint8_t bit_reg;						// сюда вносим ресультат чтения очередного бита

    for (j = 0; j < 9; j++) 				// обнулим массив
    {
        scratchpad[j] = 0x00;
    }
    j = 0;									// обнуляем счетчик
    init_device();							// выдаём импульс сброса и проверяем ответ датчиков
    send_command(0x55);						// команда соответствия

    for (j = 0; j < 8 ; j++)				// передаём код устройства к которому обращаемся
    {
        send_command (buffer.code[j]);
    }
    j = 0;									// обнуляем счетчик
    send_command(0xBE);						// читаем блокнот
    for (p = 1; p <= 72; p++)				// пока не будут прочитаны все 72 бита
    {
        bit_reg = read_data(); 				// читаем бит
        if (bit_reg)
            scratchpad[j] |= bit;			// записываем бит =1
        else
            scratchpad[j] &= ~bit;			// записываем бит = 0
        bit <<= 1;							// сдвигаем влево
        if (!bit)							// сдвиг проходит все 8 бит и значение равно 0
        {
            j++;
            _delay_ms(100);
            bit = 0x01;
        }
    }										// выходит из цикла после обработки 72-х битов
    init_device();
    if (scratchpad[4] == 0xFF)				// датчик не ответил, все биты конф.байта == 1
        return 1;
    else if (CRC_check(scratchpad, 0x08))	// если CRC не OK
        return 2;
    else
        return 0;
}

//сканирование кнопки, определение длительности нажатия, вызывается из ISR(TIMER0_OVF_vect)
void BTN_SCAN(void)
{
    if (!(SRC_PIN & src_msk))										// если нажато
    {
        if(btn_cnt < CNT_QUICK)										// если нажато меньше порога признания нажатия
        {
            btn_cnt++;												// просто растим счётчик, защита от дребезга
        }
        else if ((btn_cnt >= CNT_QUICK) && (btn_cnt < CNT_SLOW))	// если нажато дольше порога признания, но не дотягивает до длинного жима
        {
            if (!btn_state)											// если попадаем сюда в первый раз,т.е. ещё не было зафиксировано нажатие
            {
                btn_state = PRESSED;								// фиксируем факт нажатия
                btn_time = QUICK;									// фиксируем факт как минимум короткого нажатия
                btn_cnt++;											// растим счётчик дальше
            }
            else													// если уже было зафиксировано нажатие
            {
                btn_cnt++;											// просто растим счётчик дальше
            }
        }
        else if (btn_cnt == CNT_SLOW)								// если нажато дольше порога длинного жима
        {
            btn_cnt = CNT_SLOW + 10;								// чтобы в следующий раз, если кнопку так и не отпустили, сюда не зашёл
            btn_time = SLOW;										// фиксируем факт длинного нажатия
        }
    }
    else if ((SRC_PIN & src_msk)&&(btn_state == PRESSED))			// если кнопка не нажата, а факт фиксации есть, значит кнопку отпустили
    {
        press_time = btn_time;										// передаём наружу длительность нажатия и обнуляем всё для следующего круга
        btn_time = 0;
        btn_state = RELEASED;
        btn_cnt=0;
    }
}

void menu(uint8_t* qty, uint8_t* active) // КА меню для удаления/добавления/вставки устройств
{
    uint8_t  i, j = 0;									// счетчики, для массивов i - строки, j - столбцы
    static uint8_t  k = 0;								// счётчик для стадий 3 и 8, static нужно из-за выходов с очередными №№ в ожидании выбора пользователя
    static uint8_t  n = 0;								// количество датчиков, записанных в епром, static для сохранения между переходами
    static uint8_t  data [8];							// буфер-массив для хранения кодов датчиков, static для сохранения между переходами
    uint8_t  New_conflict = 0;							// переменная для новой позиции бита конфликта
    uint8_t  last_conflict = 0;							// переменная для старой позиции бита конфликта
    uint8_t digits[N_DIGS];								// массив для передачи в utoa_fast_div для заполнения его кодами символов цифр
    static uint8_t  stage, _stage = 0;					//номер состояния КА, текущий и предыдущий

    if (!(*active)) return;								// если неактивен, не входим
    if (delay_cnt < IND_PAUSE) return;					// если пауза индикации не истекла, не входим
    if (wait_timer == WAIT_LIM)							// если не дождались действий пользователя
    {
        wait_timer = WAIT_LIM + 1;						// +1 чтобы при следующем входе не сработала проверка if(wait_timer==WAIT_LIM)
        if (!(n = eeprom_read_byte((uint8_t*)dev_qty))) //если n==0, возврат в меню
        {
            stage = 10;
            return;
        }
        else											//если n!=0, просто деактивируем КА меню и выходим в main
        {
            k = n = stage = _stage = *active = 0;		// т.к. юзер протупил, то n не изменилось и не нужно обновлять его в main
            return;
        }

    }
    switch (stage)
    {
        case 0:
            str_clr(0);									// очистка 1-й строки
            str_clr(1);									// очистка 2-й строки
            send_string_to_LCD_XY (correction, 0, 0);	// выводим "Корректировка"
            n = eeprom_read_byte((uint8_t*)dev_qty);	// читаем количество датчиков, записанных в епром
            for (j = 0; j < 8; j++)   {data[j] = 0x00;} // обнулим буфер-массив
            delay_cnt = 0;								// запускаем таймер паузы на индикацию
            stage = 1;
            break;

        case 1:
            str_clr(0);									// очистка 1-й строки
            str_clr(1);									// очистка 2-й строки
            if (!n)
            {
                send_string_to_LCD_XY (subst_add, 0 ,0);// выводим "Замена/добавление"
                delay_cnt = 0;							// запускаем таймер паузы на индикацию
                stage = 5;
            }
            else
            {
                send_string_to_LCD_XY (delete, 0, 0);	// выводим "Удалить?"
                wait_timer = 0;
                stage = 2;
            }
            break;

        case 2:
            if (press_time == SLOW)
            {
                wait_timer = WAIT_LIM + 1;				//+1 чтобы проверка (wait_timer==WAIT_LIM) на входе не выбросила в main
                press_time = 0;
                stage = 3;
            }
            else if (press_time == QUICK)
            {
                wait_timer = WAIT_LIM + 1;				//+1 чтобы проверка (wait_timer==WAIT_LIM) на входе не выбросила в main
                press_time = 0;
                str_clr(0);								// очистка 1-й строки
                send_string_to_LCD_XY (subst_add, 0 ,0);// выводим "Замена/добавление"
                delay_cnt = 0;							// запускаем таймер паузы на индикацию
                stage = 4;
            }
            break;

        case 3:
            if (_stage != stage)	//если первый вход
            {
                eeprom_read_block (&buffer, &ee_arr[k], sizeof(buffer));	// считываем описание 1-го усройства из епром
                send_arr_to_LCD_XY (buffer.name, 0, 1);						// выводим в строке 2 lcd имя датчика
                _stage = stage;
                wait_timer = 0;
                return;
            }
            if (press_time == SLOW)
            {
                wait_timer = WAIT_LIM + 1;									//+1 чтобы проверка (wait_timer==WAIT_LIM) на входе не выбросила в main
                press_time = 0;
                for (; k < n; k++)
                {
                    eeprom_read_block (&buffer, &ee_arr[k+1], sizeof(buffer));	// считываем описание следующего усройства из епром
                    cli();
                    eeprom_update_block (&buffer, &ee_arr[k], sizeof(buffer));	// перемещаем в епром описание следующего усройства на текущую строку
                    sei();
                }
                cli();
                eeprom_update_byte ((uint8_t*)dev_qty, (n-1));
                sei();
                str_clr(0);								// очистка 1-й строки
                str_clr(1);								// очистка 2-й строки
                send_string_to_LCD_XY (del_done, 0, 0); // выводим "Удалено"
                delay_cnt = 0;							// запускаем таймер паузы на индикацию
                stage = 10;
            }
            else if (press_time == QUICK)
            {
                wait_timer = WAIT_LIM + 1;				//+1 чтобы проверка (wait_timer==WAIT_LIM) на входе не выбросила в main
                press_time = 0;
                if ((k + 1) < n)
                {
                    k += 1;
                    eeprom_read_block (&buffer, &ee_arr[k], sizeof(buffer));// считываем описание усройства из епром
                    str_clr(1);												// очистка 2-й строки
                    send_arr_to_LCD_XY (buffer.name, 0, 1);					// выводим в строке 2 lcd имя датчика
                    wait_timer = 0;
                }
                else
                {
                    str_clr(0);										// очистка 1-й строки
                    str_clr(1);										// очистка 2-й строки
                    send_string_to_LCD_XY (subst_add, 0 ,0);		// выводим "Замена/добавление"
                    delay_cnt = 0;									// запускаем таймер паузы на индикацию
                    stage = 4;
                }
            }
            break;

        case 4:
            str_clr(0);												// очистка 1-й строки
            str_clr(1);												// очистка 2-й строки
            send_string_to_LCD_XY (total_qty, 0, 0);				// выводим "Всего устройств"
            send_arr_to_LCD_XY (utoa_fast_div (n, digits), 7, 1);	// выводим количество устр-в, utoa_fast_div отдаёт указатель на первый ненулевой символ массива digits
            delay_cnt = 0;											// запускаем таймер паузы на индикацию
            stage = 5;
            /*
            for (not_act = 0; not_act < n; not_act++)	// как вариант убрать в 5 и 8 поиск первого неактивного
            {
                eeprom_read_block (&buffer.flags, &ee_arr[k].flags, sizeof(buffer.flags));//считываем флаг 1W устройства из епром
                if (!buffer.flags.active)
                    {break;}						// если в епром есть неактивное устр-во, выход с к == № неактивного
                else if (not_act == (n - 1))		// если проверили последний
                    {not_act = n_max;}				// если неактивных нет, номеру неактивного устр-ва присваивается условное значение
            }
            */
            break;

        case 5:
            str_clr(0);		// очистка 1-й строки
            str_clr(1);		// очистка 2-й строки
            if (n == n_max) // если епром заполнена, проверим, нет ли неактивных
            {
                /* вместо от for(){;} до else ниже, используется результат поиска неактивного из 4
                if (not_act < n_max)
                {
                    send_string_to_LCD_XY (plug_in, 0, 0);		// выводим "Подключите устр-во"
                    send_string_to_LCD_XY (press_btn, 0, 1);	// выводим "и нажмите кнопку"
                    wait_timer = 0;
                    stage = 6;
                    return;
                }
                else
                {
                    send_string_to_LCD_XY (mem_full, 0, 0); 	// выводим "Память заполнена"
                    delay_cnt = 0;								// запускаем таймер паузы на индикацию
                    stage = 10;
                    return;
                }
                */
                for (i = 0; i < n; i++)
                {
                    eeprom_read_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags));//считываем флаг 1W устройства из епром
                    if (!buffer.flags.active)					// если в епром есть неактивное, переходим к поиску
                    {
                        send_string_to_LCD_XY (plug_in, 0, 0);	// выводим "Подключите устр-во"
                        send_string_to_LCD_XY (press_btn, 0, 1);// выводим "и нажмите кнопку"
                        wait_timer = 0;
                        stage = 6;
                        return;
                    }
                }
                send_string_to_LCD_XY (mem_full, 0, 0); 		// выводим "Память заполнена"
                delay_cnt = 0;									// запускаем таймер паузы на индикацию
                stage = 10;
            }
            else
            {
                send_string_to_LCD_XY (plug_in, 0, 0);			// выводим "Подключите устр-во"
                send_string_to_LCD_XY (press_btn, 0, 1);		// выводим "и нажмите кнопку"
                wait_timer = 0;
                stage = 6;
            }
            break;

        case 6:
            if (press_time == 0) return;
            else
            {
                wait_timer = WAIT_LIM + 1;						//+1 чтобы проверка (wait_timer==WAIT_LIM) на входе не выбросила в main
                press_time = 0;
                do
                {
                    New_conflict = 0;
                    find_ID (data, &New_conflict, &last_conflict); 		// запрос на очередной проход для дешифрации следующего устройства
                    if (CRC_check (data, 0x07)) 						// передаём указатель на массив с ID, номер байта CRC8; если вернётся 1, CRC не ОК
                    {
                        send_string_to_LCD_XY (error, 0, 0);			// выводим "ошибка иниц." в 1-ю верхнюю левую позицию 1 строки экрана
                    }
                    //начинаем сравнение найденного ID с имеющимися в епром
                    i = 0;
                    do //цикл for(i=0;i<n;i++) заменён на do-while(i<n), иначе при n==0 в него не попасть, т.к. i==n==0 сразу.
                    {
                        eeprom_read_block (&buffer.code, &ee_arr[i].code, sizeof(buffer.code));		// считываем ID усройства из епром
                        if (!strncmp((void*)data, (void*)buffer.code, sizeof data))					// если найденный ID совпал с записанным в епром - запрос нового ID
                            //приходится использовать явное приведение к void*, т.к. strncmp не жрёт ничего, кроме char*, а у нас unsigned char*
                            break;	//выход в верхний do_while
                        else if (strncmp((void*)data, (void*)buffer.code, sizeof data) && ((i == (n-1))||(i == n))) // если найденный ID не совпал ни с одним записанным в епром -> найдено новое устр-во
                            //при несовпадении найденного ни с одним записанным для случая n==0 к условию i==(n-1) добавлено условие ||(i==n) , иначе эта строка при n==0 не работает
                        {
                            str_clr(0);	//очистка 1-й строки
                            str_clr(1);	//очистка 2-й строки
                            send_string_to_LCD_XY (new_dev_fnd, 0, 0);	//выводим "найдено новое" в 1-ю верхнюю левую позицию 1 строки экрана
                            send_string_to_LCD_XY (element, 0, 1);		//выводим "устройство" в 1-ю верхнюю левую позицию 2 строки экрана
                            delay_cnt = 0;
                            stage = 7;
                            return;
                        }
                        i++;
                    }
                    while (i < n);
                }
                while (last_conflict != 0); 			// пока номер бита конфликта не равен 0, если равен то все датчики проверены
                str_clr(0);								// очистка 1-й строки
                str_clr(1);								// очистка 2-й строки
                send_string_to_LCD_XY (no_new, 0, 0);	// выводим "Новых нет"
                delay_cnt = 0;							// запускаем таймер паузы на индикацию
                stage = 10;
            }
            break;

        case 7:
            str_clr(0);	// очистка 1-й строки
            str_clr(1);	// очистка 2-й строки
            if (n)		// сразу идём в раздел замены, если n!=0
            {
                stage = 8;
            }
            else	// пропускаем раздел замены, если n==0
            {
                send_string_to_LCD_XY (add_to_end, 0, 0);// выводим "Добавить?"
                wait_timer = 0;
                stage = 9;
            }
            break;

        case 8:
            if (stage != _stage)	// если первый вход
            {
                /* вместо от for(){;} до else ниже, используется результат поиска неактивного из 4
                if (not_act < n_max)
                {
                    k = not_act;
                    eeprom_read_block (&buffer.name, &ee_arr[k].name, sizeof(buffer.name)); // считываем имя неактивного устр-ва
                    send_string_to_LCD_XY (substitute, 0, 0);								// выводим "Заменить?"
                    send_arr_to_LCD_XY (buffer.name, 0, 1); 								// выводим имя неактивного устр-ва
                    wait_timer = 0;
                    press_time = 0;
                    _stage = stage;
                    return;
                }
                else
                {
                    send_string_to_LCD_XY (add_to_end, 0, 0);//выводим "Добавить?"
                    wait_timer = 0;
                    stage = 9;
                    return;
                }
                */
                for (k = 0; k < n; k++)
                {
                    eeprom_read_block (&buffer.flags, &ee_arr[k].flags, sizeof(buffer.flags));	// считываем флаг 1W устройства из епром
                    if (!buffer.flags.active) 													// если в епром есть неактивное
                    {
                        eeprom_read_block (&buffer.name, &ee_arr[k].name, sizeof(buffer.name));	// считываем имя неактивного устр-ва
                        send_string_to_LCD_XY (substitute, 0, 0);								// выводим "Заменить?"
                        send_arr_to_LCD_XY (buffer.name, 0, 1); 								// выводим имя неактивного устр-ва
                        wait_timer = 0;
                        press_time = 0;
                        _stage = stage;
                        return;
                    }
                }
                send_string_to_LCD_XY (add_to_end, 0, 0);//выводим "Добавить?"
                wait_timer = 0;
                stage = 9;
            }
            else
            {
                if (press_time == SLOW)
                {
                    wait_timer = WAIT_LIM + 1;							//+1 чтобы проверка (wait_timer==WAIT_LIM) на входе не выбросила в main
                    for (j=0; j < 8; j++)   {buffer.code[j] = data[j];}	// запись ID-кода 1-го 1W устройства в ID_string и копирование его в буфер-структуру
                    buffer.flags.active = 1; // датчик активен
                    cli();
                    eeprom_update_block (&buffer.code, &ee_arr[k].code, sizeof(buffer.code));		// записываем в епром ID-код нового 1W устройства.
                    eeprom_update_block (&buffer.flags, &ee_arr[k].flags, sizeof(buffer.flags));	//записываем в епром флаг нового 1W устройства.
                    sei();
                    str_clr(0);							// очистка 1-й строки
                    str_clr(1);							// очистка 2-й строки
                    send_string_to_LCD_XY (done, 0, 0); // выводим "Выполнено"
                    press_time = 0;
                    delay_cnt = 0;						// запускаем таймер паузы на индикацию
                    stage = 10;
                }
                else if (press_time == QUICK)			//если жим короткий, ищем другое неактивное устройство
                {
                    wait_timer = WAIT_LIM + 1;			//+1 чтобы проверка (wait_timer==WAIT_LIM) на входе не выбросила в main
                    for (; k < n; k++)
                    {
                        eeprom_read_block (&buffer.flags, &ee_arr[k].flags, sizeof(buffer.flags));// считываем флаг 1W устройства из епром
                        if (!buffer.flags.active) //если в епром есть неактивное
                        {
                            eeprom_read_block (&buffer.name, &ee_arr[k].name, sizeof(buffer.name));
                            str_clr(1);								// очистка 2-й строки
                            send_arr_to_LCD_XY (buffer.name, 0, 1); // выводим имя неактивного датчика
                            wait_timer = 0;
                            press_time = 0;
                            _stage = stage;
                            return;
                        }
                    }
                    str_clr(0);	// очистка 1-й строки
                    str_clr(1);	// очистка 2-й строки
                    send_string_to_LCD_XY (add_to_end, 0, 0);	// выводим "Добавить?"
                    wait_timer = 0;
                    stage = 9;
                }
            }
            break;

        case 9:
            if (press_time == SLOW)
            {
                wait_timer = WAIT_LIM + 1;					//+1 чтобы проверка (wait_timer==WAIT_LIM) на входе не выбросила в main
                if (n < n_max)
                {
                    for (j=0; j < 8; j++)  {buffer.code[j] = data[j];}
                    location = dev_name;					// временно локации устройства присваивается ASCII код имени по умолчанию
                    cli();
                    eeprom_update_byte((uint8_t*)dev_last_n, (eeprom_read_byte((uint8_t*)dev_last_n)+1)); //инкрементируем номер, присвоенный ранее последнему добавленному устр-ву
                    sei();
                    for (j = 0; j < N_DIGS; j++)			//заполним массив символов кодами '0'
                    {
                        digits[j] = '0';
                    }
                    utoa_fast_div (eeprom_read_byte((uint8_t*)dev_last_n), digits); // переводим номер в символы цифр
                    for (j = 1; j < 4; j++)// записываем 3 последние элемента digits в последние 3 элемента массива имени - это номер устройства
                    {
                        dev_name[N_NAME-j] = digits[N_DIGS-j];
                    }
                    //приходится использовать явное приведение к void*, т.к. utoa и strncpy не жрут ничего, кроме char*, а у нас unsigned char*
                    strncpy((void*)buffer.name, (void*)location, sizeof buffer.name); // записываем ASCII код имени в поле name буфера
                    buffer.tmax = 30;
                    buffer.tmin = 6;
                    buffer.flags.active = 1; // датчик активен
                    buffer.flags.lt_alarm = 0;
                    buffer.flags.ht_alarm = 0;
                    buffer.flags.line_alarm =0;
                    buffer.flags.sms_T = 0;
                    buffer.flags.relay_1 = 0;
                    buffer.flags.relay_2 = 0;
                    buffer.flags.reserved =0;
                    cli();
                    eeprom_update_block (&buffer, &ee_arr[n], sizeof(buffer)); // записываем в конец списка в епром описание текущего 1W устройства.
                    sei();
                    n++;
                    cli();
                    eeprom_update_byte ((uint8_t*)dev_qty, n);
                    sei();
                    str_clr(0);	// очистка 1-й строки
                    str_clr(1);	// очистка 2-й строки
                    send_string_to_LCD_XY (nev_dev_add, 0, 0);	// выводим "Добавлено новое" в 1-ю  позицию 1 строки экрана
                    send_string_to_LCD_XY (element, 0, 1); 		// выводим "устройство" в 1-ю  позицию 2 строки экрана
                    delay_cnt = 0;								// запускаем таймер паузы на индикацию
                    press_time = 0;
                    stage = 10;
                }
                else
                {
                    send_string_to_LCD_XY (mem_full, 0, 0); 	// выводим "Память заполнена"
                    press_time = 0;
                    delay_cnt = 0;								// запускаем таймер паузы на индикацию
                    stage = 10;
                }
            }
            else if (press_time == QUICK)
            {
                wait_timer = WAIT_LIM + 1;						//+1 чтобы проверка (wait_timer==WAIT_LIM) на входе не выбросила в main
                stage = 10;
                press_time = 0;
            }
            break;

        case 10:
            str_clr(0);										// очистка 1-й строки
            str_clr(1);										// очистка 2-й строки
            n = eeprom_read_byte((uint8_t*)dev_qty);
            if (!n)											// если в процессе удалены все датчики, возврат к началу меню
            {
                send_string_to_LCD_XY (absence, 0, 0);		// выводим "нет датчиков"
                delay_cnt = 0;								// запускаем таймер паузы на индикацию
                stage = 0;
            }
            else
            {
                *qty = n;									// если дошли до конца меню, обновляем значение n для цикла опроса
                k = n = stage = _stage = *active = 0;		// полный выход из меню и деактивация
                wait_timer = WAIT_LIM + 1;					// +1 чтобы при следующем входе не сработала проверка if(wait_timer==WAIT_LIM)
            }
            break;
    }
    return;
}

ISR(TIMER0_OVF_vect) //обработчик прерывания таймера 0
{
    int_cnt++; //счётчик прерываний
    if (int_cnt == 2) //при максимальном делителе таймера 1024 нужно доп.делитель на 2, чтобы иметь 30Гц
    {
        BTN_SCAN (); //проверяем кнопку
        int_cnt = 0; //начинаем новый отсчёт
    }

    if (delay_cnt < IND_PAUSE)
    {delay_cnt++;}//счётчик задержки 2c для индикации

    if (wait_timer < WAIT_LIM)
    {wait_timer++;}//счётчик паузы 15c на действия пользователя

    flash_cnt++; //счётчик для вспышек индикации уровня сигнала
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
    //time_gsm++; //счётчик интервалов 30c запросов GSM, сбрасываается в main
}

//начало основой программы
int main (void)
{
    uint8_t i = 0;								// счетчик "по вертикали", в т.ч. порядковый номер устройства в списке
    unsigned char j = 0;						// счетчик "по горизонтали"
    unsigned char temperature[5];				// массив байтов температуры LB и HB
    uint16_t temp_int = 0; 							// целая часть температуры
    unsigned char temp_float = 0; 					// дробная часть температуры
    int8_t temp_int_signed; 					// целая часть т-ры со знаком для сравнения с пороговыми значениями
    unsigned int temp; 							// временная переменная для перевода из дополнительного кода в прямой при "-" температуре
    unsigned char temp_sign = 0; 					// признак знака температуры
    uint16_t Number = 0; 						// сюда попадёт значение температуры
    uint8_t Dig_1, Dig_2, Dig_3, sign = 0;		// переменные для кодов символов цифр температуры и символа знака
    uint8_t n = 0;								// для количества записанных в епром устройств
    uint8_t srch_done = 0;						// признак проведённой первичной дешифрации
    uint8_t digits[N_DIGS];						// массив для передачи в utoa_fast_div для заполнения его кодами символов цифр температуры
    int8_t last_t[n_max];						// массив целых частей последних измеренных температур со знаком
    uint8_t next_i = 0;							// номер следующего устройства, которое выводится на lcd
    uint8_t stage = 0;							// состояние автомата индикации
    uint8_t lines_n = 0;							// количество строк кадра, передаётся в функцию Display
    uint8_t menu_act = 0;						// флаг фхода в меню

    // LCD_COM_PORT_DDR |= (1<<RS)|(1<<EN); 	// линии RS и EN выходы, раскомм. если DAT и COM цеплять на разные порты
    // LCD_COM_PORT = 0x00;						// ставим 0 в RS и EN, раскомментировать если DAT и COM цеплять на разные порты
    LCD_DAT_PORT_DDR = 0xFF;					// порт данных (и команд в данном случае) ЖКИ - на выход
    LCD_DAT_PORT = 0x00;						// ставим 0 в порт данных и команд ЖКИ
    _delay_ms(200);								// Ожидание готовности ЖКИ
    lcd_init();									// Инициализация дисплея

    DDR_LED_PORT |= _BV(LED_PIN_NUM); 			// ставим 1 - пин led активирован на выход

    TIMSK0 |= (1<<TOIE0);						// разрешаем прерывание по переполнению
    TCCR0B |= (1<<CS02) | (1<<CS00);			// делитель частоты 1024
    TCCR0B &= ~(1<<CS01);						// делитель частоты 1024
    TCCR0A &= ~(1<<WGM00) & ~(1<<WGM01);		// режим работы "normal"
    sei();

    DDR_SRC_PORT &= ~_BV(SRC_PIN_NUM);			// ставим 0 - пин кнопки активирован на вход
    SRC_PORT |= _BV(SRC_PIN_NUM);				// ставим 1 - внутр. подтяжка к +

    if (!(SRC_PIN & src_msk))					// если на пине 0, т.е кнопка нажата, делаем полный поиск устройств на шине и перезапись найденного в епром
    {
        while (!press_time) {;}					// пока кнопка не отпущена, ждём
        press_time = 0;							// обнуляем результат идентификации длины нажатия, чтобы не отработал повторно
        search_ID();							// призводим дешифрацию всех ID-кодов
        srch_done = 1;							// признак выполненной полной дешифрации
    }

    if (!(n = eeprom_read_byte((uint8_t*)dev_qty)))//если полной дешифрации не было, обновляем и проверяем n, если 0 - идём в корректировку, ходим по кругу, пока n==0;
    {
        lcd_clr();								// очистка дисплея
        send_string_to_LCD_XY (absence, 0, 0);	// выводим "Нет датчиков"
        send_string_to_LCD_XY (init_srch, 0, 1);// выводим "Выполнить поиск?"
        while (press_time != SLOW)				// пока не будет длинного нажатия, ждём
        {press_time = 0;}					//сбрасываем короткие нажатия
        menu_act = 1;							//активируем КА меню, проваливаемся в главный цикл, КА опроса будет блокирован
        srch_done = 1;							//признак выполненного добавления устройств
    }

    if (!srch_done)	// если дешифрация не выполнялась, переходим к опросу ранее записанных и далее к измерениям
    {
        lcd_clr();								// очистка дисплея
        send_string_to_LCD_XY (ow_check, 0, 0);	// сообщение "опрос линии" в 1-ю верхнюю левую позицию 1 строки экрана
        for (uint8_t i = 0; i< n; i++)			//начинаем перекличку с первого датчика (от 0)
        {
            eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer)); // считываем описание усройства из епром
            uint8_t pad_res = scratchpad_rd(); // результат чтения блокнота
            if (!pad_res)
            {
                buffer.flags.active = 1;		// подъём флага присутствия
            }
            else if (pad_res == 1)				//если датчик просто не ответил, конф.байт будет == FF
            {
                buffer.flags.active = 0;		//сброс флага присутствия
                lcd_clr();						// очистка дисплея
                send_string_to_LCD_XY (no_answer_n, 0, 0); // выводим "Нет ответа " в 1-ю верхнюю левую позицию 1 строки экрана
                send_arr_to_LCD_XY (buffer.name, 0, 1);
                _delay_ms(2000);
                lcd_clr();						// очистка дисплея
            }
            else if (pad_res == 2)				//читаем блокнот i-го датч., если CRC не ОК, выводим сообщение об ошибке
            {
                buffer.flags.active = 0;		//сброс флага присутствия
                lcd_clr();						// очистка дисплея
                send_string_to_LCD_XY (scratch_err, 0, 0); // выводим "Ош.CRC-блкн. " в 1-ю верхнюю левую позицию 1 строки экрана
                send_arr_to_LCD_XY (buffer.name, 0, 1);
                _delay_ms(2000);
                lcd_clr();						// очистка дисплея
            }
            cli();
            eeprom_update_block (&buffer, &ee_arr[i], sizeof(buffer)); // возвращаем в епром описание текущего 1W устройства с обновлённым байтом флагов.
            sei();
        }
        lcd_clr();								// очистка дисплея
    }

    while(1)
    {
        if (!menu_act)	//если пользователь не вошёл в меню, работает опрос и нидикация.
        {
            //n = eeprom_read_byte((uint8_t*)dev_qty;
            eeprom_read_block (&buffer, &ee_arr[i], sizeof(buffer));	// считываем описание усройства из епром
            if (buffer.flags.active)									// если флаг присутствия поднят, запрашиваем температуру (если сброшен, на дисплей в индикации пошлём ???.?)
            {
                init_device();		//импульс сброса и присутствие
                send_command(0x55);	//команда соответствия
                // после передадим код устройства к которому обращаемся
                for (j = 0; j < 8 ; j++)
                {
                    send_command (buffer.code[j]);
                }

                send_command (0x44);			// команда преобразования
                while (!read_data()) ;			// выполняется цикл пока на линии не установится 1 - преобразование закончено
                init_device();					// импульс сброса и присутствие
                send_command(0x55);				// команда соответствия
                for (j = 0; j < 8 ; j++)   		// опять передаем адрес устройства, к которому будем обращаться
                {
                    send_command (buffer.code[j]);
                }
                send_command (0xBE);			// команда чтение памяти
                for (j = 0; j < 5; j++) 		// считываем первые два байта температуры и конф.байт
                {
                    unsigned char i;			// локальная переменная для внутреннего цикла
                    unsigned char data_temp = 0x00;
                    for (i = 0; i < 8; i++)
                    {
                        data_temp >>= 1;
                        if (read_data()) 		// если 1, то устанавливаем  старший бит 1
                            data_temp |= 0x80;
                    }
                    temperature[j] = data_temp;
                }
                init_device();						// сброс датчиков для прекращения передачи данных
                // преобразуем полученное в значение температуры
                if (!(temperature[4] == 0xFF))		// если конф.байт станет вдруг ==FF, то датчик замолчал.
                {
                    if (buffer.flags.line_alarm) 	// сбрасываем флаг пропадания, если он был поднят в каком-то предыдущем цикле, а датчик проявился
                    {
                        buffer.flags.line_alarm = 0;
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags)); //записываем флаги в епром
                        sei();
                    }
                    if ((temperature[1]&0b10000000) == 0)	// проверка на отрицательность температуры
                        temp_sign = 0; 						// рисуем плюс на ЖКИ
                    else 									// переводим из доп.кода в прямой
                    {
                        temp = ((unsigned int)temperature[1]<<8) | temperature[0];
                        temp = ~temp + 1;
                        temperature[0] = temp;
                        temperature[1] = temp >> 8;
                        temp_sign = 1;						//рисуем минус на ЖКИ
                    }

                    temp_int = ((temperature[1]&0b00000111)<<4)|(temperature[0]>>4);	// выводим  целое знач. температуры
                    temp_float = (temperature[0]&0b00001111); 							// выделяем с помощью битовой маски дробную часть

                    temp_int_signed = (!temp_sign ? temp_int : ~(temp_sign - 1));		// добавляем знак к temp_int для сравнения с tmin, для отриц.числа перевод в доп.код
                    last_t[i] = temp_int_signed;										// заносим в масиив последних измеренных значений
                    if ((temp_int_signed < buffer.tmin)&&(!buffer.flags.lt_alarm))		// если Т ниже предела, а флаг не установлен
                    {
                        buffer.flags.lt_alarm = 1; // ставим флаг
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); // записываем флаги в епром
                        sei();
                        if (buffer.flags.sms_T) // если установлен флаг отправки sms, отправляем на lcd
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
                    else if ((temp_int_signed > buffer.tmax)&&(!buffer.flags.ht_alarm))				//если Т выше предела, а флаг не установлен
                    {
                        buffer.flags.ht_alarm = 1; // ставим флаг
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //записываем флаги в епром
                        sei();
                        if (buffer.flags.sms_T) //если установлен флаг отправки sms, отправляем на lcd
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
                    else if ((temp_int_signed > buffer.tmin)&&(buffer.flags.lt_alarm))				//если Т выше (T min), а флаг установлен
                    {
                        buffer.flags.lt_alarm = 0; // снимаем флаг
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //записываем флаги в епром
                        sei();
                    }
                    else if ((temp_int_signed < buffer.tmax)&&(buffer.flags.ht_alarm))				//если Т ниже (T max) предела, а флаг установлен
                    {
                        buffer.flags.ht_alarm = 0; // снимаем флаг
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof buffer.flags); //записываем флаги в епром
                        sei();
                    }
                }
                else 											//если датчик вдруг замолчал (на дисплей в индикации пошлём ---.-)
                {
                    if (!buffer.flags.line_alarm)				//поднимаем флаг пропадания, если он не был поднят в каком-то предыдущем цикле
                    {
                        buffer.flags.line_alarm = 1;
                        cli();
                        eeprom_update_block (&buffer.flags, &ee_arr[i].flags, sizeof(buffer.flags)); //записываем флаги в епром
                        sei();
                        lcd_clr();
                        send_arr_to_LCD_XY(buffer.name, 0, 0); 	//извещаем пользователя об аварии
                        send_string_to_LCD (blank);
                        send_string_to_LCD (crash);
                        _delay_ms(1500);
                    }
                }
            }

            switch (stage)	//автомат вывода на lcd
            {
                case 0:
                    if (i == next_i)							// если текущий равен ожидаемому
                    {
                        if (!buffer.flags.active)				//если поднят флаг пропадания
                        {
                            Dig_1 = '-';
                            Dig_2 = '-';
                            Dig_3 = '-';
                            sign = '-';
                        }
                        else if (buffer.flags.line_alarm)		//если флаг присутствия сброшен
                        {
                            Dig_1 = '?';
                            Dig_2 = '?';
                            Dig_3 = '?';
                            sign = '?';
                        }
                        else									//если датчик активен
                        {
                            temp_int = (temp_int << 3) + (temp_int << 1); 		//умножаем целую часть на 10, нужен 0 в единицах для десятичной цифры
                            temp_float = ((temp_float << 2) + temp_float) >> 3; //умножаем дробную часть на (0,0625 * 10), итого на 5/8, получаем десятичную цифру *10
                            Number = temp_float + temp_int;						// на место единиц встаёт дробная часть, домноженная на 10
                            /*		преобразуем число в цифры, потом их в коды символов цифр		*/
                            for (j = 0; j < N_DIGS; j++) 						//заполним массив символов кодами '0', т.к. ведущие нули нужны
                            {
                                digits[j] = '0';
                            }
                            j = 0;							//обнуляем счётчик
                            utoa_fast_div (Number, digits); //преобразуем число в символы цифр и записываем в массив digits
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

                        Frame (Dig_1, Dig_2, Dig_3, sign, i, n);//заносим строку в структуру кадра

                        if (i == (n-1))							//если строка последняя
                        {
                            next_i = 0;							//начинаем новый круг индикации
                            stage = 1;							//переходим в стадию индикации
                            if (!(i & 0x01)	)					//если строка верхняя
                            {
                                lines_n = 0;					//вывод одной верхней строки
                            }
                            else								//если строка нижняя
                            {
                                lines_n = 1;					//вывод двух строк
                            }
                        }
                        else									//если строка не последняя
                        {
                            next_i += 1;						//индексируем номер следующего выводимого устр-ва
                            if (i & 0x01)						//если строка нижняя
                            {
                                lines_n = 1;					//вывод двух строк
                                stage = 1;						//переходим в стадию индикации
                            }
                        }
                    }
                    break;
                case 1:
                    if (delay_cnt >= IND_PAUSE)	// если пауза на индикацию истекла
                    {
                        str_clr(0);				// очистка 1-й строки
                        str_clr(1);				// очистка 2-й строки
                        Display (lines_n);		// выводим кадр на lcd
                        delay_cnt = 0;			// перезапускаем таймер индикации
                        stage = 0;				// возврат в стадию формирования кадра
                    }
                    break;
            }	//конец автомата индикации

            i += 1;								//индексируем номер устройства
            if (i > (n - 1)){i = 0;}			//если номер вышел за пределы максимального, обнуляем
        }	//конец опроса очередного устройства

        if (!menu_act)						//если КА меню неактивен,...
        {
            if (press_time == SLOW)			//... и произошло длинное нажатие
            {
                menu_act = 1;				// ативируем КА меню (блок опроса будет блокирован)
                press_time = 0;				// обнуляем факт нажатия
                delay_cnt = IND_PAUSE;		// чтобы вход в меню сразу после нажатия, иначе будет ждать окончания паузы индикации
                i = next_i = stage = 0;		// чтобы после выхода из меню опрос и индикация начались с первого устройства (от 0)
            }
            else {press_time = 0;}			//если нажатие короткое, пока ничего не делаем? обнуляем факт нажатия
        }

        menu (&n, &menu_act);				//вызываем КА меню, внутри проверка menu_act, если 0 - выход.
    }	//конец бесконечного цикла
}	//конец функции
