#include <avr/io.h>
#include <util/delay.h>		// задержки
#include <avr/interrupt.h>	// прерывания
#include <stdlib.h>
#include <avr/pgmspace.h>	// строки во флэш
#include <avr/eeprom.h>		// работа с епром
#include <string.h>			// функции работы со строками
#include <ctype.h>			// для isdigit()

//#define DEBUG
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
#define N_NAME 8						// размер массива для имени устройства
#define EEP_MEM 1024					// объём епром для вычисления адреса последней и предпоследней ячейки, также для определения максимального количества устройств в епром
#define IND_PAUSE 120 					// пауза на индикацию очередного кадра, 2с
#define WAIT_LIM 900 					// пауза для ожидания действий пользователя, 15с

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
#define LED_PORT		PORTB	// Порт, к одному из выводов которого подключен светодиод (здесь порт один с OW line)
#define DDR_LED_PORT	DDRB	// Регистр направления данных порта, к одному из выводов которого подключен светодиод (здесь порт один с OW line)
#define LED_PIN_NUM		5		// Номер PIN, к которому подключен светодиод, для макроса _BV()
#define GSM_LVL_TIME	3600	// интервал запроса уровня сигнала 60c

//блок define для USART-GSM
#define MYUBRR			103					// скорость usart 9600
#define RX_RING_SIZE	128					// размер буфера берём 128, ответ модема с текстом вх.смс больше 64 байт, а нужно обеспечить размер равный степени 2
#define RX_IND_MSK		(RX_RING_SIZE - 1)	// маска индексов кольцевого буфера приёмника для обнуления индекса при переходе  RX_Index через 0
#define TX_RING_SIZE	32					// размер буфера берём 32, максимальная длина посылки в модем 25 байт, но нужно обеспечить размер равный степени 2
#define TX_IND_MSK		(TX_RING_SIZE - 1)	// маска индексов кольцевого буфера передатчика для обнуления индекса при переходе TX_Index через 0

//блок define для работы с модемом
//#define SMS_SIZE			25					// ограничение длины исходящих смс
#define QUEUE_SIZE			16					// размер кольца очереди обработчиков
#define INC_TASK_SIZE		8 					// размер кольца задач на чтение смс
#define OUT_TASK_SIZE		4					// размер кольца задач на отправку смс
#define CMD_TASK_SIZE		4					// размер кольца задач на отправку команд
#define QUEUE_IND_MSK		(QUEUE_SIZE - 1)	// маска кольца очереди обработчиков
#define INC_TASK_IND_MSK	(INC_TASK_SIZE - 1)	// маска кольца задач на чтение смс
#define OUT_TASK_IND_MSK	(OUT_TASK_SIZE - 1)	// маска кольца задач на отправку смс
#define CMD_TASK_IND_MSK	(CMD_TASK_SIZE - 1)	// маска кольца задач на отправку команд
#define PWR_UP_ANS_TIMER	900					// таймер получения ответа модема при включении
#define PAUSE_CNT_MAX		60					// максимальная пауза после приёма очередного байта по RX перед XOFF (PAUSE_CNT_MAX/60)c
#define MSG_SIZE			(RX_RING_SIZE + 1)	// размер массива для выгрузки из кольца приёмника; +1б к Rx-кольцу т.к. в конце всегда нужен 0 даже при выгрузке 128б
#define TODO_MAX			20					// размер массива для текста команды контроллеру
#define RST_PORT			PORTB				// Порт, к одному из выводов которого подключен вывод сброса модема(здесь порт один с OW line)
#define DDR_RST_PORT		DDRB				// Регистр направления данных порта, к одному из выводов которого подключен вывод сброса модема(здесь порт один с OW line)
#define RST_PIN_NUM			2 					// Номер вывода, к которому подключен вывод сброса модема, для макроса _BV()

// блок текстовых сообщений во флэш
unsigned char absence[] 		PROGMEM = "Нет датчиков";		// ошибка на этапе первой инициализации датчиков перед входом в алгоритм дешифрации адресов
unsigned char no_answer[] 		PROGMEM = "Нет ответа датч.";	// ошибка на старте дешифрации адресов после чтения первых двух тайм-слотов
unsigned char present_n[] 		PROGMEM = "Найдено датч. ";		// количество устройств, опознанных при первичной дешифрации адресов
unsigned char dev_excess[] 		PROGMEM = "Много датчиков";		// ошибка в процессе первичной дешифрации адресов, если количество датчиков превысит 50
unsigned char error[] 			PROGMEM = "Ошибка CRC-ID ";		// ошибка на этапе проверки CRC ID после первичной дешифрации адресов, выводится № датчика,
unsigned char init_n[]			PROGMEM = "Исправн.датч. ";		// количество датчиков, прошедших проверку CRC после первичной дешифрации адресов
unsigned char init_srch[] 		PROGMEM = "Выполнить поиск?";	// ошибка при отсутствии в епром данных датчиков, если не проводилась первичная дешифрация адресов
unsigned char scratch_err[] 	PROGMEM = "Ош.CRC-блкн. ";		// ошибка при проверке CRC данных, прочитанных из блокнота, выводится № датчика
unsigned char no_answer_n[] 	PROGMEM = "Нет ответа ";		// ошибка на этапе чтения чтения блокнота при перекличке в main - если датчик не ответил, то конф.байт == FF, выводится имя датчика
unsigned char ow_check[] 		PROGMEM = "Опрос линии";		// сообщение в main при нормальном входе во время переклички
unsigned char correction[] 		PROGMEM = "Корректировка";		// сообщение в add_ID при входе
unsigned char subst_add[] 		PROGMEM = "Замена/добавл-е";	// сообщение в add_ID при входе в блок замены/удаления
unsigned char total_qty[] 		PROGMEM = "Всего устройств";	// сообщение в add_ID перед началом поиска
unsigned char plug_in[] 		PROGMEM = "Подключите устр.";	// сообщение в add_ID перед началом поиска
unsigned char press_btn[] 		PROGMEM = "и нажмите кнопку";	// сообщение в add_ID перед началом поиска
unsigned char mem_full[]		PROGMEM = "Память заполнена";	// сообщение в add_ID, если количество датчиков уже 50, а неактивных нет
unsigned char new_dev_fnd[] 	PROGMEM = "Найдено новое";		// верхняя строка сообщения в add_ID, если найдено новое
unsigned char nev_dev_add[] 	PROGMEM = "Добавлено новое";	// верхняя строка сообщения в add_ID в конце блока добавления
unsigned char element[] 		PROGMEM = "устройство";			// нижняя строка сообщения в add_ID, если найдено или добавлено новое
unsigned char substitute[] 		PROGMEM = "Заменить?";			// сообщение в add_ID в блоке замены, если нашли неактивное устр-во, выводится имя датчика
unsigned char add_to_end[] 		PROGMEM = "Добавить?";			// сообщение в add_ID в начале блока добавления
unsigned char done[] 			PROGMEM = "Замена выполнена";	// сообщение в add_ID после вставки взамен неактивного
unsigned char no_new[] 			PROGMEM = "Новых устр-в нет";	// сообщение в add_ID если прошли весь цикл поиска ноовых ID
unsigned char delete[] 			PROGMEM = "Удалить?";			// сообщение - первый пункт меню Корректировка
unsigned char del_done[] 		PROGMEM = "Удалено";			// сообщение по факту удаления устройства
unsigned char t_error[] 		PROGMEM = "Неверная темп-ра";	// сообщение при попытке записать некорректную температуру в датчик
unsigned char com_error[] 		PROGMEM = "Неверная команда";	// сообщение при ошибке в тексте команды
unsigned char frame_err[]		PROGMEM = "Ошибка приёма!"; 	// сообщение при ошибке приёма байта по Rx
unsigned char name_error_ren[] 	PROGMEM = "Неверное имя REN";	// сообщение при ошибкочном имени в тексте команды
unsigned char name_error_al[] 	PROGMEM = "Неверное имя AL";	// сообщение при ошибкочном имени в тексте команды
unsigned char name_error_sms[] 	PROGMEM = "Неверное имя SMS";	// сообщение при ошибкочном имени в тексте команды
unsigned char t_min[] 			PROGMEM = "Tmin";				// часть строки при подтверждении изменения минимального порогоа температуры
unsigned char t_max[] 			PROGMEM = "Tmax";				// часть строки при подтверждении изменения максимального порога температуры
unsigned char t_low[] 			PROGMEM = "T<";					// часть строки при сообщениии о снижении температуры ниже минимального порога
unsigned char t_high[] 			PROGMEM = "T>";					// часть строки при сообщениии о повышении температуры выше максимального порога
unsigned char sms_send []		PROGMEM = "SMS";				// часть строки при подтверждении включения sms о выходе температуры за пределы
unsigned char on []				PROGMEM = "ВКЛ.";
unsigned char off []			PROGMEM = "ОТК.";
unsigned char blank []			PROGMEM = " ";
unsigned char crash []			PROGMEM = "FAIL!";
unsigned char tx_ring_ovf[]		PROGMEM = "TX-буф. полный!";
unsigned char rx_ring_ovf[]		PROGMEM = "RX-буф. полный!";
unsigned char quick[]			PROGMEM = "QUICK";
unsigned char slow[]			PROGMEM = "SLOW";
unsigned char sim900[]			PROGMEM = "SIM900 ";
unsigned char not_rdy[]			PROGMEM = "не готов";
unsigned char out_ring_ovf[]	PROGMEM = "Исх.буф.полный!";
unsigned char inc_ring_ovf[]	PROGMEM = "Вх.буф.полный!";
unsigned char cmd_ring_ovf[]	PROGMEM = "Ком.буф.полный!";
unsigned char q_ring_ovf[]		PROGMEM = "Конв.полный!";

//блок текстовых строк во флэш для работы с модемом
unsigned char ANS_OK[]			PROGMEM = "\r\nOK\r\n";		// ответный ОК
unsigned char AT_CMGD[]			PROGMEM = "AT+CMGD=";		// удалить смс ("=1,4" - удалить все)
unsigned char AT_CMGS[]			PROGMEM = "AT+CMGS=\"";		// послать смс
unsigned char ANS_ENT[]			PROGMEM = "\r\n> ";			// затем вводим текст
unsigned char ANS_CMGS[]		PROGMEM = "\r\n+CMGS: ";	// смс отправлено
unsigned char CTRL_Z[]			PROGMEM = {0x1A, 0};		// Ctrl-Z - символ завершения смс
unsigned char AT_CMGR[]			PROGMEM = "AT+CMGR=";		// прочитать смс
unsigned char ANS_CMGR[]		PROGMEM = "\r\n+CMGR: ";	// ответ
unsigned char AT_BUSY[]			PROGMEM = "AT+GSMBUSY=1";	// запрет всех входящих звонок (1-запрещены, 0 - разрешены)
unsigned char ANS_CMTI[]		PROGMEM = "\r\n+CMTI: ";	// входящее смс
unsigned char AT_CSQ[]			PROGMEM = "AT+CSQ";			// запрос уровня сигнала
unsigned char ANS_CSQ[]			PROGMEM = "\r\n+CSQ: ";		// ответ на запрос уровня сигнала
unsigned char CRLF[]			PROGMEM = "\r\n";			// символы CRLF
unsigned char TEXT_1_4[]		PROGMEM = "1,4";			// "1,4" - удалить все смс
unsigned char PHONE[]			PROGMEM = "+79052135678";	// номер телефона, пока здесь
unsigned char QUOTES[]			PROGMEM = "\"";				// закрывающие кавычки для добавления в конце телефона при отправке смс
unsigned char CALL_RDY[]		PROGMEM = "Ready\r\n";		// Call Ready - последний URC модема после включения или сброса

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
int8_t *t_all = NULL;														// глобальный указатель на локальный (в main) массив целых частей последних измеренных температур со знаком (last_t[n_max])
/* из-за вычислямости размера массива last_t[n_max] его не сделать глобальным сразу, можно только засунуть в функцию, а глобально обращаться через этот указатель */

//блок переменных-счётчиков для индикации уровня gsm
volatile uint8_t flash_cnt; 	// счётчик прерываний для индикации уровня gsm вспышками диода
volatile uint8_t flash_num; 	// число прошедших вспышек
volatile uint8_t pause_num; 	// число прошедших отрезков пауз между сериями вспышек
volatile uint8_t gsm_lvl; 		// уровень gsm сигнала
volatile uint16_t time_gsm; 	// счётчик для запросов уровня сигнала gsm-модуля
volatile uint8_t gsm_lvl_req;	// флаг запроса уровня gsm сигнала

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

//блок переменных, структур и массивов для работы с модемом
typedef struct	//структура трекинга работы обработчиков
{
    uint8_t flag_1 : 1;	// шаг 1
    uint8_t flag_2 : 1;	// шаг 2
    uint8_t flag_3 : 1;	// резерв
    uint8_t flag_4 : 1;	// резерв
    uint8_t flag_5 : 1;	// резерв
    uint8_t flag_6 : 1;	// резерв
    uint8_t flag_7 : 1;	// резерв
    uint8_t flag_8 : 1;	// резерв
}tracker;

typedef struct	//структура содержимого смс
{
    uint8_t sms_type;					// вариант шаблона смс (обязательный параметр)
    uint8_t dev_num;					// номер устройства (заполняется при необходимости)
    int8_t param;						// параметр, пока только температура из массива результатов измерений (заполняется при необходимости)
}sms_mask;

typedef struct 	//структура задачи приёма смс, 4 байта
{
    tracker step;						// Флаги процесса
    uint8_t	sms_num[3];					// массив символов порядкового номера смс для чтения
}inc_task;

typedef struct //структура задачи отправки смс, 21 байт, снизить до 8 через передачу сюда № устройства, указателей (части строк смс) и массива символов (текущее значение параметра).
{
    tracker step;						// Флаги процесса отправки
    sms_mask sms_txt; 					// структура для записи параметров исходящего смс, сборка текста налету при отправке по параметрам
}out_task;

typedef struct //структура задачи отправки команд в модем, 5 байт
{
    tracker step;						// Флаги процесса
    uint8_t* cmd;						// указатель на AT-команду
    uint8_t* par;						// указатель на параметр AT-команды
}cmd_task;

sms_mask sms_buff;						// буфер для текста смс
volatile unsigned char pause_cnt;		// счётчик паузы после приёма последнего байта в кольцо приёмника
volatile unsigned char msg_upld; 		// флаг окончания новой посылки от модема и разрешения выгрузки в из кольца приемника в msg
volatile unsigned char modem_rdy;		// флаг готовности модема
unsigned char cmd_task_H, cmd_task_T, inc_task_H, inc_task_T, out_task_H, out_task_T, queue_H, queue_T = 0;//индексы очереди и списков
volatile uint16_t ans_cnt;				// счётчик интервала на получение ответа OK от модема на команду
uint16_t ans_lim;						// предел интервала на получение ответа модема на команду
uint8_t handl_res;						// результат работы обработчика задачи
uint8_t msg[MSG_SIZE]; 					// массив для выгрузки из кольцевого буфера     	#############
uint8_t todo_txt [TODO_MAX];			// массив для выгрузки текста команды контроллеру	#############
uint8_t mod_ans;						// парсер присваивает значение в зависимости от ответа модема на запросы ( "ОК", ">" и пр.)
enum {OK = 1, INVITE};					// варианты значений для mod_ans, см.выше
enum {FAIL, ALARM, DONE, ALL, TEST1, TEST2};			// варианты значений для типов смс
tracker RESET;							// создаём битовое поле для флагов инициализаци модема

//	блок переменных и массивов для работы с USART
/*	алгоритм работы колец Rx и Tx - сначала сдвигаем индекс, потом пишем/читаем
	КОЛЬЦА ОЧЕРЕДИ ОБРАБОТЧИКОВ И СПИСКОВ ЗАДАЧ РАБОТАЮТ ПО-ДРУГОМУ!!! */
volatile uint8_t RX_IndexIN;	// входящий индекс кольцевого буфера приёмника, здесь обязательно volatile, меняется только в прерывании
uint8_t RX_IndexOUT;			// выходящий индекс кольцевого буфера приёмника
uint8_t RX_ring[RX_RING_SIZE];	// массив для кольцевого буфера приёмника
uint8_t TX_IndexIN;				// входящий индекс кольцевого буфера передатчика,
volatile uint8_t TX_IndexOUT;	// выходящий индекс кольцевого буфера передатчика; здесь обязательно volatile, меняется только в прерывании
uint8_t TX_ring[TX_RING_SIZE];	// массив для кольцевого буфера передатчика

//блок функций и массивов для работы с очередями
/*	алгоритм работы колец очереди и задач: сначала читаем/пишем, потом сдвигаем индекс
	алгоритм отличается от алгоритма колец Rx и TX!!! */
inc_task RD_SMS[INC_TASK_SIZE];				//кольцевой список задач чтения смс
out_task WR_SMS[OUT_TASK_SIZE];				//кольцевой список задач отправки смс
cmd_task WR_CMD[CMD_TASK_SIZE];				//кольцевой список задач отправки команд
uint8_t (*HANDLERS[QUEUE_SIZE])(void);		//кольцевая очередь указателей на функции-обработчики заданий
uint8_t read_sms (void);					//объявляем HANDLER чтения смс
uint8_t send_sms (void);					//объявляем HANDLER отправки смс
uint8_t parser(void);						//объявляем HANDLER разбора текста
uint8_t send_cmd (void);					//объявляем HANDLER отправки команды в модем
void inc_to_queue(uint8_t*);				//объявляем функцию постановки в очередь задачи чтения смс
void out_to_queue(sms_mask*);				//объявляем функцию постановки в очередь задачи отправки смс
void cmd_to_queue(uint8_t*, uint8_t*);		//объявляем функцию постановки в очередь задачи отправки команды в модем
void to_do(void);							//объявляем функцию разбора и выполнения команд из текста смс

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

/*					#######################		начало блока функций работы с приёмником 		###########################*/

// Функция очистки кольцевого буфера приёмника
/*void Clear_RX_Ring(void)
{
	cli();
	RX_IndexIN = 0;
	RX_IndexOUT = 0;
	sei();
}*/

// Функция возвращает количество непрочитанных байт в кольцевом буфере приёмника
uint8_t RX_IndexNumber(void)
{
    return (RX_IndexIN - RX_IndexOUT) & RX_IND_MSK;
}

// Функция загрузки байта данных в кольцевой буфер приёмника
uint8_t UDR_to_RX_Ring(char value)
{
    if (((RX_IndexIN + 1) & RX_IND_MSK) == RX_IndexOUT)
        return 1; // Переполнение буфера, не пишем новые данные
    else
    {
        RX_IndexIN++;
        RX_IndexIN &= RX_IND_MSK;
        RX_ring[RX_IndexIN] = value;
        return 0;
    }
}

// Функция выгрузки строки данных из кольцевого буфера в массив для разбора команд
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

/*// наличие не прочитанных данных в кольцевом буфере приёмника
uint8_t Get_RX_Data(void)
{
	if(RX_IndexIN != RX_IndexOUT) return 1;
	return 0;
}*/

/*					#########################		конец блока функций работы с приёмником 	################*/

/*					#########################		начало блока функций работы с передатчиком 	################*/

// Функция очистки кольцевого буфера передатчика
/*void Clear_TX_Ring(void)
{
	cli();
	TX_IndexIN = 0;
	TX_IndexOUT = 0;
	sei();
}*/

// наличие не прочитанных данных в кольцевом буфере передатчика
uint8_t Get_TX_Data(void)
{
    if(TX_IndexIN != TX_IndexOUT) return 1;
    return 0;
}

void USART_TXD(uint8_t data) // Передача байта по UART
{
    while (!( UCSR0A & (1 << UDRE0))) {;} 	// Ждем пока не отправятся предыдущие данные
    UDR0 = data;							// Отпраляем текущие данные
}

/* void USART_CRLF(void) //передача CR и LF
{
	USART_TXD('\r');
	USART_TXD('\n');
}*/

/*void arr_to_USART(uint8_t *s ) // Передача массива по UART
{
	while(*s)
	{
		USART_TXD(*s++);
	}
}*/

/*void string_to_USART(const uint8_t *s ) // Передача сроки из флэш по UART
{
	while(pgm_read_byte(s))
	{
		USART_TXD(pgm_read_byte(s++));
	}
}*/

//функция отправки байта в кольцевой буфер передатчика
void byte_to_TX_Ring(uint8_t byte)
{
    if (((TX_IndexIN + 1) & TX_IND_MSK) == TX_IndexOUT)
    {
        send_string_to_LCD_XY(tx_ring_ovf, 0, 0);		//###############
        //string_to_USART (tx_ring_ovf);				//###############
    }
    while (((TX_IndexIN + 1) & TX_IND_MSK) == TX_IndexOUT) {;} //если голова догнала хвост, ждём
    TX_IndexIN++;
    TX_IndexIN &= TX_IND_MSK;
    TX_ring[TX_IndexIN] = byte;

}

//функция отправки массива в кольцо передатчика
void arr_to_TX_Ring(uint8_t *s)
{
    while(*s)
        byte_to_TX_Ring(*s++);
}

//функция отправки строки из флэш в кольцо передатчика
void string_to_TX_Ring(const uint8_t *s)
{
    while(pgm_read_byte(s))
        byte_to_TX_Ring(pgm_read_byte(s++));
}

/*//функция отправки символов CR и LF в кольцо передатчика
void CRLF_to_TX_Ring(void)
{
	byte_to_TX_Ring('\r');
	byte_to_TX_Ring('\n');
}*/

/*			################################			конец блока функций работы с передатчиком 			##################*/


void USART_Init( unsigned int ubrr)
{
    UBRR0H = (uint8_t)(ubrr >> 8);						//скорость 9600
    UBRR0L = (uint8_t)ubrr;								//скорость 9600
    UCSR0A = 0;											//обнуляем флаги
    UCSR0B = (1 << RXEN0)|(1 << TXEN0)|(1 << RXCIE0);	//включаем передачу, приём и прерывания по приёму
    UCSR0C = (1 << UCSZ01)|(1 << UCSZ00)|(1 << USBS0);	//8 бит, 1 стоп бит
}

void msg_clr(void)	//очистка msg
{
    for (uint8_t j=0; j < MSG_SIZE; j++) {msg[j] = 0;}
}

void NRESET(void) //сброс модема
{
    cli();
    TX_IndexIN = TX_IndexOUT = RX_IndexIN = RX_IndexOUT = 0;//очищаем кольца приёма и передачи
    modem_rdy = 0;						//модем не готов
    gsm_lvl = 0;						//моргание уровня GSM останавливаем для индикации нерабочего модема
    RESET.flag_1 = 0;					//войти в первый этап инициализации
    ans_lim = PWR_UP_ANS_TIMER;			//таймер ожидания ответа 15с
    msg_clr();							//обнуляем msg
    mod_ans = 0;						//обнуляем ответ, чтобы не считать его повторно при фактическом отсутствии

    /*если NRESET тянуть на 0 через внешний ключ, то так: (и раскомментировать строку в main) */
    //RST_PORT |= _BV(RST_PIN_NUM);		//замыкаем ключ сброса
    //_delay_us(50);
    //RST_PORT &= ~_BV(RST_PIN_NUM);	//размыкаем ключ сброса

    /*если NRESET тянуть на 0 сразу портом меги, то так:  (и закомментировать строку в main) */
    RST_PORT &= ~_BV(RST_PIN_NUM);		//ставим 0 на выводе для сброса модема
    DDR_RST_PORT |= _BV(RST_PIN_NUM);	//вывод для сброса модема активирован на выход - замыкаем NRESET
    _delay_us(50);
    DDR_RST_PORT &= ~_BV(RST_PIN_NUM);	//вывод для сброса модема в 3-е состояние - размыкаем NRESET

    ans_cnt = 0;						//запускаем таймер ожидания ответа
    sei();
}

uint8_t send_cmd (void)	//HANDLER отправки команды
{
    if (!WR_CMD[cmd_task_T].step.flag_1)			//если флаги процесса нули
    {
        uint8_t cmd_txt[26];
        strcpy_P ((char*)cmd_txt, (PGM_P) WR_CMD[cmd_task_T].cmd);		//собираем текст команды
        if (WR_CMD[cmd_task_T].par != NULL)								//если параметр команды не пустой
        {
            strcat_P ((char*)cmd_txt, (PGM_P) WR_CMD[cmd_task_T].par);	//добавляем текст команды
        }
        strcat_P ((char*)cmd_txt, (PGM_P) CRLF);	//собираем текст команды и записываем в задачу
        arr_to_TX_Ring (cmd_txt);					//отправка текста команды в кольцо передатчика
        ans_lim = 180;								//установка времени ожидания ответа
        UCSR0B |= (1<<UDRIE0);						//разрешение прерывания по опустошению UDR передатчика
        ans_cnt = 0;								//запускаем таймер ответа
        WR_CMD[cmd_task_T].step.flag_1 = 1;			//текст команды отправлен в модем
        return 'S';
    }
    else if (WR_CMD[cmd_task_T].step.flag_1)
    {
        if (mod_ans == OK)	//если в msg есть ОК
        {
            cmd_task_T = (cmd_task_T + 1) & CMD_TASK_IND_MSK;	//удаляем из списка команд
            queue_T = (queue_T + 1) & QUEUE_IND_MSK;			//удаляем выполненную задачу из очереди
            mod_ans = 0;										//обнуляем ответ, чтобы не считать его повторно при фактическом отсутствии
            return 'F';
        }
        else if (ans_cnt >= ans_lim)
        {
            WR_CMD[cmd_task_T].step.flag_1 = 0;	//сброс флага "текст команды отправлен в модем"
            NRESET();							//сброс модема
            return 'G';
        }
        else return 'H';
    }
    else return 'Z';
}

uint8_t send_sms (void)	//HANDLER отправки смс
{
    uint8_t cmd[26];		// временный массив текста команды
    uint8_t name[N_NAME];	// временный массив имени устройства
    uint8_t k = 0;			// переменная номера устройства
    uint8_t n = 0;			// переменная количества устройств
    uint8_t sms_type = 0;	// переменная для типа смс
    int8_t t = 0;			// переменная температуры из массива измерений
    uint8_t digits[N_DIGS];	// временный массив цифр температур

    if (!WR_SMS[out_task_T].step.flag_1 && !WR_SMS[out_task_T].step.flag_2)	//если первый вход в задачу
    {
        strcpy_P ((char*)cmd, (PGM_P) AT_CMGS);
        strcat_P ((char*)cmd, (PGM_P) PHONE);
        strcat_P ((char*)cmd, (PGM_P) QUOTES);
        strcat_P ((char*)cmd, (PGM_P) CRLF);
        arr_to_TX_Ring (cmd);
        ans_lim = 180;							//таймер ожидания ответа 3с
        UCSR0B |= (1<<UDRIE0);					//разрешение прерывания по опустошению UDR передатчика
        ans_cnt = 0;							//запускаем таймер ожидания ответа ">"
        WR_SMS[out_task_T].step.flag_1 = 1;		//запрос на передачу смс отправлен
        return 'A';
    }
    else if (WR_SMS[out_task_T].step.flag_1)	//если запрос на передачу смс был отправлен в модем
    {
        if (mod_ans == INVITE)	//если в msg есть ">"
        {
            sms_type = WR_SMS[out_task_T].sms_txt.sms_type;
            switch (sms_type)
            {
                case FAIL:
                    k = WR_SMS[out_task_T].sms_txt.dev_num;					// копируем номер устройства
                    eeprom_read_block (name, &ee_arr[k].name, sizeof name);	// читаем имя устройства во временный массив
                    arr_to_TX_Ring (name);			// шлём в кольцо имя
                    string_to_TX_Ring (blank);		// шлём в кольцо пробел
                    string_to_TX_Ring (crash);		// шлём в кольцо АВАРИЯ!
                    break;
                case ALL:
                    n = eeprom_read_byte((uint8_t*)dev_qty);
                    for (k = 0; k < n; k++)
                    {
                        eeprom_read_block (name, &ee_arr[k].name, sizeof name);	// читаем имя устройства во временный массив
                        arr_to_TX_Ring (name);									// шлём в кольцо имя
                        string_to_TX_Ring (blank);								// шлём в кольцо пробел
                        t = t_all[k];											// читаем температуру из массива измерений
                        byte_to_TX_Ring(((t & 0b10000000) ? '-' : '+'));		// шлём в кольцо знак температуры
                        arr_to_TX_Ring(utoa_fast_div (((t & 0b10000000) ? ((~t) + 1) : t), digits)); // шлём в кольцо цифры темп-ры
                        string_to_TX_Ring (CRLF);								// шлём в кольцо символы CRLF
                        UCSR0B |= (1<<UDRIE0);									// разрешение прерывания по опустошению UDR передатчика
                        while (UCSR0B & (1<<UDRIE0)){;}							// ждём пока не сбросится флаг
                    }
                    break;
                case TEST1:
                    string_to_TX_Ring (quick);
                    break;
                case TEST2:
                    string_to_TX_Ring (slow);
                default:							// если ни один случай не отработал
                    string_to_TX_Ring (sms_send);	// пока шлём текст SMS АВАРИЯ!
                    string_to_TX_Ring (crash);
                    break;
            }
            string_to_TX_Ring (CTRL_Z);				// обеспечить \0 в конце текста смс в .sms_txt!!!
            ans_lim = 600;							// время ожидания ответа 10с
            UCSR0B |= (1<<UDRIE0);					// разрешение прерывания по опустошению UDR передатчика
            ans_cnt = 0;							// запускаем таймер ответа
            WR_SMS[out_task_T].step.flag_1 = 0;		// сброс флага "запрос на передачу смс отправлен в модем"
            WR_SMS[out_task_T].step.flag_2 = 1;		// подъём флага "тескт смс отправлен в модем"
            mod_ans = 0;							// обнуляем ответ, чтобы не считать его повторно при фактическом отсутствии
            return 'B';
        }
#ifdef DEBUG
            else if (ans_cnt < ans_lim)
			{return 'C';}
#endif
        else if (ans_cnt >= ans_lim)
        {
            WR_SMS[out_task_T].step.flag_1 = 0;		//сброс флага "запрос на передачу смс отправлен в модем"
            NRESET();								//сброс модема
            return 'D';
        }
        else return 'X';
    }
    else if (WR_SMS[out_task_T].step.flag_2)					//если текст смс был отправлен в модем
    {
        if (mod_ans == OK)										//если в msg есть ".....OK"
        {
            out_task_T = (out_task_T + 1) & OUT_TASK_IND_MSK;	//удаляем задачу из списка задач на отправку смс
            queue_T = (queue_T + 1) & QUEUE_IND_MSK;			//удаляем выполненную задачу из очереди
            mod_ans = 0;										//обнуляем ответ, чтобы не считать его повторно при фактическом отсутствии
            return 'E';
        }
#ifdef DEBUG
            else if (ans_cnt < ans_lim)
			{return 'F';}
#endif
        else if (ans_cnt >= ans_lim)
        {
            WR_SMS[out_task_T].step.flag_2 = 0;				//сброс флагов задачи
            NRESET();										//сброс модема
            return 'G';
        }
        else return 'Y';
    }
    else
        return 'H';
}

uint8_t read_sms (void)	//HANDLER чтения смс
{
    uint8_t cmd[16];														//массив текста команды
    if (!RD_SMS[inc_task_T].step.flag_1 && !RD_SMS[inc_task_T].step.flag_2) //если первый вход в задачу
    {
        strcpy_P ((char*)cmd, (PGM_P) AT_CMGR);					//собираем команду из флэш
        strcat ((char*)cmd, (char*)RD_SMS[inc_task_T].sms_num);	//собираем команду из флэш
        strcat_P ((char*)cmd, (PGM_P) CRLF);					//собираем команду из флэш
        arr_to_TX_Ring (cmd);
        ans_lim = 120;						//таймер ответа 2с
        UCSR0B |= (1<<UDRIE0);				//разрешение прерывания по опустошению UDR передатчика
        ans_cnt = 0;						//запускаем таймер ожидания ответа "/r/nOK/r/n"
        RD_SMS[inc_task_T].step.flag_1 = 1;	//подъём флага "запрос на чтение смс отправлен в модем"
        return 'A';
    }
    else if (RD_SMS[inc_task_T].step.flag_1) //если запрос на чтение смс отправлен в модем
    {
        if (mod_ans == OK)	//если в msg есть "/r/nOK/r/n"
        {
            strcpy_P ((char*)cmd, (PGM_P) AT_CMGD);					//собираем команду из флэш
            strcat ((char*)cmd, (char*)RD_SMS[inc_task_T].sms_num);	//собираем команду из флэш
            strcat_P ((char*)cmd, (PGM_P) CRLF);					//собираем команду из флэш
            arr_to_TX_Ring (cmd);
            ans_lim = 120;							//таймер ожидания ответа 2с
            UCSR0B |= (1<<UDRIE0);					//разрешение прерывания по опустошению UDR передатчика
            ans_cnt = 0;							//запускаем таймер ответа
            RD_SMS[inc_task_T].step.flag_1 = 0;		//кладём флаг "запрос на чтение смс отправлен в модем"
            RD_SMS[inc_task_T].step.flag_2 = 1;		//подъём флага "запрос на удаление прочитанного смс отправлен в модем"
            mod_ans = 0;							//обнуляем ответ, чтобы не считать его повторно при фактическом отсутствии
            return 'B';
        }
#ifdef DEBUG
            else if (ans_cnt < ans_lim)
			{return 'C';}
#endif
        else if (ans_cnt >= ans_lim)
        {
            RD_SMS[inc_task_T].step.flag_1 = 0;	//сброс флага "запрос на чтение смс отправлен в модем"
            NRESET();							//сброс модема
            return 'D';
        }
        else return 'X';
    }
    else if (RD_SMS[inc_task_T].step.flag_2) //если запрос на удаление прочитанного смс отправлен в модем
    {
        if (mod_ans == OK) //если в msg есть "/r/nOK/r/n"
        {
            inc_task_T = (inc_task_T + 1) & INC_TASK_IND_MSK;	//удаляем задачу из списка задач на чтение смс
            queue_T = (queue_T + 1) & QUEUE_IND_MSK;			//удаляем выполненную задачу из очереди
            mod_ans = 0;										//обнуляем ответ, чтобы не считать его повторно при фактическом отсутствии
            return 'E';
        }
#ifdef DEBUG
            else if (ans_cnt < ans_lim)
			{return 'F';}
#endif
        else if (ans_cnt >= ans_lim)
        {
            RD_SMS[inc_task_T].step.flag_2 = 0;	//сброс флагов задачи
            NRESET();							//сброс модема
            return 'G';
        }
        else return 'Y';
    }
    else
        return 'H';
}

uint8_t parser(void) //разбор текста msg
{
    char* txt_ptr;			//указатель на начало искомого текста в тексте смс
    uint8_t n = 0;			//число найденных +cmti:, их может быть несколько
    uint8_t  number[3];		//временный массив для порядкового номера смс
    uint8_t pars_res = 'Z';	//результат работы парсера

    if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_OK))!=NULL) 				//если в msg есть r/n/OKr/n:
    {
        mod_ans = OK;
        pars_res = 'O';
    }
    else if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_ENT))!=NULL)			//если в msg есть r/n>:
    {
        mod_ans = INVITE;
        pars_res = 'I';
    }

    if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_CMGR))!=NULL)	//если в msg есть r/n/+cmgr:
    {
        if (strstr_P((const char*)msg, (char*)(PGM_P)PHONE)!=NULL)				//если телефон правильный
        {
            for (uint8_t j = 0; j < TODO_MAX; j++) {todo_txt [j] = 0;}		 	//очистка массива задания контроллеру
            uint8_t j = 0;
            while (((*(txt_ptr + 64 + j)) != '\r') && (j < TODO_MAX))			//копируем текст команды контроллеру из смс
            {
                todo_txt[j] = *(txt_ptr + 64 + j);
                j++;
            }
            to_do(); 		//вызываем функцию распознавания команды
            pars_res = 'R';
        }
    }

    if ((txt_ptr = strstr_P((const char*)msg, (PGM_P) ANS_CSQ))!=NULL) //если в строке есть r/n/+csq:_
    {
        uint8_t tmp[3];						//временный массив приёма символов цифр уровня сигнала
        uint8_t j = 0;
        while (isdigit(*(txt_ptr + 8 + j)))	//переносим цифровые символы после r/n/+csq:_ в массив
        {
            tmp[j] = *(txt_ptr + 8 + j);
            j++;
        }
        uint8_t lev = atoi_fast (tmp);					// переводим символы в число == уровню сигнала
        if (lev > 0 && lev < 6)        	{gsm_lvl = 2;}	//0-25% - одна вспышка
        else if (lev >= 6 && lev < 9)   {gsm_lvl = 4;}	//25-50% - две вспышки
        else if (lev >= 9 && lev < 16)  {gsm_lvl = 6;}	//50-75% - три вспышки
        else if (lev >= 16 && lev <= 31){gsm_lvl = 8;}	//75-100% - четыре вспышки
        else {gsm_lvl = 0;}								//нет сигнала, не моргает
        pars_res = 'Q';
    }

    do	//ищем все r/n/+cmti:_
        if ((txt_ptr = strstr_P((const char*)(msg + n), (PGM_P) ANS_CMTI))!=NULL)	//если в msg есть r/n/+cmti:
        {
            uint8_t j = 0;
            while (isdigit(*(txt_ptr + 14 + j)))	//если символы после r/n/+cmti: "SM", - цифры, заносим № входящего смс в задачу чтения смс
            {
                number[j] = *(txt_ptr + 14 + j);
                j++;
            }
            number[j] = '\0';		//завершающий 0
            inc_to_queue (number);	//ставим в очередь задание на чтение смс
            n += 15;				//смещение для поиска +cmti: в оставшейся части строки
            pars_res = 'T';
        }
    while (txt_ptr != NULL);
    msg_clr();
    return pars_res;
}

void inc_to_queue (uint8_t* arr)	//постановка в очередь задачи чтения смс
{
    if ((((queue_H + 1) & QUEUE_IND_MSK) != queue_T) && (((inc_task_H + 1) & INC_TASK_IND_MSK) != inc_task_T)) 	//если голова не догнала хвост
    {
        for (uint8_t j=0; j<3; j++)							//копируем номер смс в структуру задачи
        {
            RD_SMS[inc_task_H].sms_num[j] = arr[j];
        }
        //обнуляем флаги процесса
        RD_SMS[inc_task_H].step.flag_1 = RD_SMS[inc_task_H].step.flag_2 = RD_SMS[inc_task_H].step.flag_3 = RD_SMS[inc_task_H].step.flag_4
                = RD_SMS[inc_task_H].step.flag_5 = RD_SMS[inc_task_H].step.flag_6 = RD_SMS[inc_task_H].step.flag_7 = RD_SMS[inc_task_H].step.flag_8 = 0;
        inc_task_H = (inc_task_H + 1) & INC_TASK_IND_MSK;	//сдвигаем индекс головы списка заданий на чтение смс
        HANDLERS[queue_H] = read_sms;						//ставим в очередь задание прочитать смс
        queue_H = (queue_H + 1) & QUEUE_IND_MSK;			//сдвигаем индекс головы очереди заданий
    }
    else
    {
        if  (((inc_task_H + 1) & INC_TASK_IND_MSK) == inc_task_T)
        {
            send_string_to_LCD_XY (inc_ring_ovf, 0, 0);	//выводим "Вх.буф.полный!"
            _delay_ms(1000);
        }
        if  (((queue_H + 1) & QUEUE_IND_MSK) == queue_T)
        {
            send_string_to_LCD_XY (q_ring_ovf, 0, 1);	//выводим "Конв.полный!"
            _delay_ms(1000);
        }
    }
}

void out_to_queue (sms_mask *str)	//постановка в очередь задачи отправки смс
{
    if ((((queue_H + 1) & QUEUE_IND_MSK) != queue_T) && (((out_task_H + 1) & OUT_TASK_IND_MSK) != out_task_T)) 	//если голова не догнала хвост
    {
        WR_SMS[out_task_H].sms_txt.sms_type = str->sms_type;		//копируем информацию о содержании смс в структуру задачи
        WR_SMS[out_task_H].sms_txt.dev_num = str->dev_num;
        WR_SMS[out_task_H].sms_txt.param = str->param;
        //обнуляем флаги процесса
        WR_SMS[out_task_H].step.flag_1 = WR_SMS[out_task_H].step.flag_2 = WR_SMS[out_task_H].step.flag_3 = WR_SMS[out_task_H].step.flag_4
                = WR_SMS[out_task_H].step.flag_5 = WR_SMS[out_task_H].step.flag_6 = WR_SMS[out_task_H].step.flag_7 = WR_SMS[out_task_H].step.flag_8 = 0;
        out_task_H = (out_task_H + 1) & OUT_TASK_IND_MSK;			//сдвигаем индекс головы списка заданий на отправку смс
        HANDLERS[queue_H] = send_sms;								//ставим в очередь задание отправить смс
        queue_H = (queue_H + 1) & QUEUE_IND_MSK;					//сдвигаем индекс головы очереди заданий
    }
    else
    {
        if  (((out_task_H + 1) & OUT_TASK_IND_MSK) == out_task_T)
        {
            send_string_to_LCD_XY (out_ring_ovf, 0, 0);	//выводим "Исх.буф.полный!"
            _delay_ms(1000);
        }
        if  (((queue_H + 1) & QUEUE_IND_MSK) == queue_T)
        {
            send_string_to_LCD_XY (q_ring_ovf, 0, 1);	//выводим "Конв.полный!"
            _delay_ms(1000);
        }
    }
}

void cmd_to_queue (uint8_t *cmd, uint8_t *par)	//постановка в очередь задачи отправки команды
{
    if ((((queue_H + 1) & QUEUE_IND_MSK) != queue_T) && (((cmd_task_H + 1) & CMD_TASK_IND_MSK) != cmd_task_T)) 	//если голова не догнала хвост
    {
        WR_CMD[cmd_task_H].cmd = cmd;		//записываем указатель на текст команды в задачу
        WR_CMD[cmd_task_H].par = par;		//записываем указатель на параметр команды в задачу
        //обнуляем флаги процесса
        WR_CMD[cmd_task_H].step.flag_1 = WR_CMD[cmd_task_H].step.flag_2 = WR_CMD[cmd_task_H].step.flag_3 = WR_CMD[cmd_task_H].step.flag_4
                = WR_CMD[cmd_task_H].step.flag_5 = WR_CMD[cmd_task_H].step.flag_6 = WR_CMD[cmd_task_H].step.flag_7 = WR_CMD[cmd_task_H].step.flag_8 = 0;
        cmd_task_H = (cmd_task_H + 1) & CMD_TASK_IND_MSK;				//сдвигаем индекс головы списка заданий на отправку команды
        HANDLERS[queue_H] = send_cmd;									//ставим в очередь задание отправить команду
        queue_H = (queue_H + 1) & QUEUE_IND_MSK;						//сдвигаем индекс головы очереди заданий
    }
    else
    {
        if  (((cmd_task_H + 1) & CMD_TASK_IND_MSK) == cmd_task_T)
        {
            send_string_to_LCD_XY (cmd_ring_ovf, 0, 0);	//выводим "Ком.буф.полный!"
            _delay_ms(1000);
        }
        if  (((queue_H + 1) & QUEUE_IND_MSK) == queue_T)
        {
            send_string_to_LCD_XY (q_ring_ovf, 0, 0);	//выводим "Конв.полный!"
            _delay_ms(1000);
        }
    }
}

void to_do (void) //тестовый модуль разбора и выполнения команды
{
    if (todo_txt[0]=='r' && todo_txt[1]=='e' && todo_txt[2]=='q' && todo_txt[3]=='u' && todo_txt[4]=='e' && todo_txt[5]=='s' && todo_txt [6] == 't')
    {
        // out_to_queue ((uint8_t*)(PSTR("answer")));
        sms_buff.sms_type = TEST2;
        out_to_queue (&sms_buff);
    }
    else if (todo_txt[0]=='T' && todo_txt[1]==' ' && todo_txt[2]=='A' && todo_txt[3]=='L' && todo_txt[4]=='L')
    {
        sms_buff.sms_type = ALL;
        out_to_queue (&sms_buff);
    }
    else
    {
        lcd_clr();
        send_string_to_LCD_XY (com_error, 0, 0);
        _delay_ms (1500);
    }
}

/*			################################			начало блока обработчиков прерываний				##################*/

ISR(USART_RX_vect)	// Обработчик прерывания для приёмника по приходу данных в UDR0
{
    uint8_t temp;				//временная переменная для приёма из UDR0	#################
    uint8_t wr_err;				//код успешности записи в кольцо			#################
    pause_cnt = 0;				//сброс счётчика паузы в ISR(TIMER0_OVF_vect) после приёма очередного байта #############
    if(UCSR0A & (1 << FE0))		// Ошибка кадрирования, не пишем новые данные, сообщаем
    {
        send_string_to_LCD_XY(frame_err, 0, 0);
        //string_to_USART (frame_err);#############
        _delay_ms(1500);
    }
    else
    {
        temp = UDR0;
        wr_err = UDR_to_RX_Ring(temp);		// запрашиваем запись в кольцо и принимаем код успешности
        if(wr_err == 1)						//при ошибке переполнения выводим сообщение, данные в UDR_to_RX_Ring не записаны
        {
            send_string_to_LCD_XY(rx_ring_ovf, 0, 0);
            //string_to_USART (rx_ring_ovf);	#############
            _delay_ms(1500);
        }
    }
}

ISR (USART_UDRE_vect)  // Обработчик прерывания по опустошению UDR передатчика
{
    TX_IndexOUT++;
    TX_IndexOUT &= TX_IND_MSK;		//проверка маски кольцевого буфера
    UDR0 = TX_ring[TX_IndexOUT];	//запись из кольцевого буфера в UDR
    if (!Get_TX_Data())				//если буфер уже пуст
    {
        UCSR0B &= ~(1<<UDRIE0); 	//запрет прерывания по опустошению UDR передатчика
    }
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

    flash_cnt++;		//счётчик для генерации вспышек индикации уровня сигнала
    if (flash_cnt == 10)
    {
        flash_cnt = 0;
        if (!pause_num)
        {
            if (flash_num < gsm_lvl)	//################### серия вспышек равна gsm_lvl/2
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
    {time_gsm++;}			//счётчик интервалов запросов GSM #############
    else if (time_gsm == GSM_LVL_TIME)
    {
        time_gsm +=1;			//чтобы при следующем прерывании сюда не заходил, пока не сбросится в main
        gsm_lvl_req = 1;		//ставим флаг - отправить запрос уровня GSM
    }

    if (pause_cnt < PAUSE_CNT_MAX)
    {pause_cnt++;} 			//счётчик паузы после приёма последнего байта в кольцо приёмника #############
    else if (pause_cnt == PAUSE_CNT_MAX)
    {
        //byte_to_TX_Ring(0x13);	//XOFF
        //UCSR0B |= (1<<UDRIE0);	//разрешение прерывания по опустошению UDR передатчика
        USART_TXD (0x13);		//XOFF прямая отправка
        msg_upld = 1;			//ставим флаг прихода новой посылки
        pause_cnt += 1;			//чтобы при следующем прерывании сюда не заходил, пока не сбросится в ISR(USART_RX_vect)
    }

    if (ans_cnt < ans_lim) {ans_cnt++;}		//счётчик интервала для ответа модема на команду или при сбросе ##############
}
/*			################################			конец блока обработчиков прерываний				##################*/

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



//начало основой программы
int main (void)
{
    uint8_t i = 0;								// счетчик "по вертикали", в т.ч. порядковый номер устройства в списке
    unsigned char j = 0;						// счетчик "по горизонтали"
    unsigned char temperature[5];				// массив байтов температуры LB и HB
    uint16_t temp_int = 0; 						// целая часть температуры
    unsigned char temp_float = 0; 				// дробная часть температуры
    int8_t temp_int_signed; 					// целая часть т-ры со знаком для сравнения с пороговыми значениями
    unsigned int temp; 							// временная переменная для перевода из дополнительного кода в прямой при "-" температуре
    unsigned char temp_sign = 0; 				// признак знака температуры
    uint16_t Number = 0; 						// сюда попадёт значение температуры
    uint8_t Dig_1, Dig_2, Dig_3, sign = 0;		// переменные для кодов символов цифр температуры и символа знака
    uint8_t n = 0;								// для количества записанных в епром устройств
    uint8_t srch_done = 0;						// признак проведённой первичной дешифрации
    uint8_t digits[N_DIGS];						// массив для передачи в utoa_fast_div для заполнения его кодами символов цифр температуры
    int8_t last_t[n_max];						// массив целых частей последних измеренных температур со знаком
    uint8_t next_i = 0;							// номер следующего устройства, которое выводится на lcd
    uint8_t stage = 0;							// состояние автомата индикации
    uint8_t lines_n = 0;						// количество строк кадра, передаётся в функцию Display
    uint8_t menu_act = 0;						// флаг фхода в меню
    t_all = last_t;								// определяем глобальный указатель на локальный массив

    cli();
    modem_rdy = 0;								//при старте по умолчанию модем не готов
    RESET.flag_1 = 0;							//при старте делаем инициализацию модема
    msg_upld = 0;								//запрет выгрузки в msg
    pause_cnt = PAUSE_CNT_MAX + 1;				//таймер паузы приёма байтов в кольцо приёмника остановлен
    ans_lim = PWR_UP_ANS_TIMER;					//таймер ответа при инициализации модема
    time_gsm = GSM_LVL_TIME + 1;				//счётчик интервалов запросов уровня gsm остановлен
    gsm_lvl_req = 1;							//выполнить запрос уровня gsm после инициализации модема

    // LCD_COM_PORT_DDR |= (1<<RS)|(1<<EN); 	// линии RS и EN выходы, раскомм. если DAT и COM цеплять на разные порты
    // LCD_COM_PORT = 0x00;						// ставим 0 в RS и EN, раскомментировать если DAT и COM цеплять на разные порты
    LCD_DAT_PORT_DDR = 0xFF;					// порт данных (и команд в данном случае) ЖКИ - на выход
    LCD_DAT_PORT = 0x00;						// ставим 0 в порт данных и команд ЖКИ
    _delay_ms(200);								// Ожидание готовности ЖКИ
    lcd_init();									// Инициализация дисплея

    DDR_LED_PORT |= _BV(LED_PIN_NUM); 			// ставим 1 - пин led активирован на выход
    DDR_SRC_PORT &= ~_BV(SRC_PIN_NUM);			// ставим 0 - пин кнопки активирован на вход
    SRC_PORT |= _BV(SRC_PIN_NUM);				// ставим 1 - внутр. подтяжка к +
    DDR_RST_PORT &= ~_BV(RST_PIN_NUM);			// вывод для сброса модема в 3-е состояние - размыкаем NRESET
    /*раскомментировать, если NRESET на 0 через внешний ключ, см. ф-цию. NRESET*/
    //DDR_RST_PORT |= _BV(RST_PIN_NUM);			//вывод для сброса модема активирован на выход

    TIMSK0 |= (1<<TOIE0);						// разрешаем прерывание по переполнению
    TCCR0B |= (1<<CS02) | (1<<CS00);			// делитель частоты 1024
    TCCR0B &= ~(1<<CS01);						// делитель частоты 1024
    TCCR0A &= ~(1<<WGM00) & ~(1<<WGM01);		// режим работы "normal"

    USART_Init(MYUBRR);							//включаем UART
    TX_IndexIN = TX_IndexOUT = RX_IndexIN = RX_IndexOUT = 0;// Сбросить к/буферы приёмника и передатчика
    msg_clr();									//очистка msg
    ans_cnt = 0;								// запуск таймера ожидания ответа модема после включения/сброса
    cmd_to_queue (AT_CMGD, TEXT_1_4);			// добавляем в список команд "удалить все смс"
    sei();										// глобально разрешаем прерывания



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
                        sms_buff.sms_type = FAIL;				//записываем в буферную структуру тип смс
                        sms_buff.dev_num = i;					//записываем в буферную структуру номер аварийного устройства
                        sms_buff.param = 0;						//обнуляем параметр, здесь не нужен
                        out_to_queue (&sms_buff);				//ставим задачу на отправку смс
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
            // else {press_time = 0;}			//если нажатие короткое, пока ничего не делаем? обнуляем факт нажатия
        }

        menu (&n, &menu_act);				//вызываем КА меню, внутри проверка menu_act, если 0 - выход.

        if (msg_upld)						//если поднят флаг прихода новой посылки
        {
            uint8_t pars_res = 0;			//результат работы парсера
            uint8_t num = RX_IndexNumber(); //сколько байт в кольцевом буфере
            RX_Ring_to_Str(msg, num);		//выгружаем из кольца в msg
            msg_upld = 0;					//обнуляем флаг прихода новой посылки
            //byte_to_TX_Ring(0x11);			//XON
            //UCSR0B |= (1<<UDRIE0);			//разрешение прерывания по опустошению UDR передатчика
            USART_TXD (0x11);					//XON прямая отправка
            if (modem_rdy)					//парсер вызывется только при активном модеме
            {
                pars_res = parser();		//разбираем текст в msg
#ifdef DEBUG
                lcd_dat_XY(pars_res, 7, 1); //выводим признак результатата работы парсинга
#endif
            }
        }

        if (!modem_rdy)		//если модем не готов(где-то инициирован сброс и снят флаг)
        {
            if (!RESET.flag_1)	//первый заход после сброса
            {
                // send_string_to_LCD_XY (sim900, 0, 0);
                // send_string_to_LCD_XY (not_rdy, 7, 0);
                if (strstr_P((const char*)msg, (PGM_P) CALL_RDY) != NULL)//если в msg есть Ready\r\n
                {
                    msg_clr();
                    string_to_TX_Ring (AT_BUSY);
                    string_to_TX_Ring (CRLF);	//отправляем AT+GSMBUSY=1
                    ans_lim = 120;				//таймер ответа 1с
                    UCSR0B |= (1<<UDRIE0);		//разрешение прерывания по опустошению UDR передатчика
                    ans_cnt = 0;				//запускаем таймер ожидания ответа от модема
                    RESET.flag_1 = 1;			//поднимаем флаг "запрет входящих звонков отправлен в модем"
#ifdef DEBUG
                    lcd_dat_XY((0x35), 4, 1);	//вывод символа 5 на lcd
					_delay_ms(1500);
#endif
                }
#ifdef DEBUG
                    else if (ans_cnt < ans_lim)
				{
					lcd_dat_XY('i', 5, 1);		//вывод символа i на lcd
					_delay_us(50);
				}
#endif
                else if (ans_cnt >= ans_lim)
                {
#ifdef DEBUG
                    lcd_dat_XY((0x36), 6, 1);	//вывод символа 6 на lcd
					_delay_us(50);
#endif
                    NRESET();	//сброс модема
                }
            }
            else if(RESET.flag_1)	//ждём ОК в ответ на AT+GSMBUSY=1
            {
                if (strstr_P((const char*)msg, (PGM_P) ANS_OK) != NULL)	//если в msg есть "...OK"
                {
                    msg_clr();
                    modem_rdy = 1;	//модем готов
                    // lcd_clr();
                    // _delay_us(50);
                    // send_string_to_LCD_XY (not_rdy, 6, 0);
                    // send_string_to_LCD_XY (sim900, 2, 0);//смещаем чтобы убрать "не" из "не готов"
#ifdef DEBUG
                    lcd_dat_XY((0x39), 9, 1);	//вывод символа 9 на lcd
#endif
                }
#ifdef DEBUG
                    else if (ans_cnt < ans_lim)
				{
					lcd_dat_XY('h', 5, 1);		//вывод символа h на lcd
					//_delay_us(50);
				}
#endif
                else if (ans_cnt >= ans_lim)
                {
#ifdef DEBUG
                    lcd_dat_XY((0x37), 7, 1);	//вывод символа 7 на lcd
#endif
                    NRESET();	//сброс модема
                }
            }
        }

        else	//если модем готов, идём в тело главного цикла
        {
            if (queue_T != 	queue_H)
            {
                handl_res = HANDLERS[queue_T]();
#ifdef DEBUG
                lcd_dat_XY(handl_res, 8, 1); //вывод на lcd результатов из обработчиков
#endif
            }

            if (gsm_lvl_req)				//если в прерывании установился флаг отправки запроса уровня
            {
                cmd_to_queue (AT_CSQ, NULL);//ставим задачу
                gsm_lvl_req = 0;			//обнуляем флаг
                time_gsm = 0;				//запускаем таймер запросов уровня gsm в прерывании
            }

            switch (press_time) 			// блок детектирования нажатия кнопки и задач по длительности жима
            {
                case QUICK :
                {
                    sms_buff.sms_type = ALL;
                    out_to_queue (&sms_buff);			//отправляем перечень температур в смс
                    lcd_clr(); 							//очистка дисплея
                    send_string_to_LCD_XY(quick, 0, 0); //отправляем текст QUICK на lcd
                    press_time = 0;
                    break;
                }
                case SLOW : // сейчас не работает
                {
                    //out_to_queue (slow);	//отправляем текст SLOW
                    lcd_clr();				// очистка дисплея
                    send_string_to_LCD_XY (slow, 0, 0);
                    press_time = 0;
                    break;
                }
                case 0 :
                    break;
            }
        }
    }	//конец бесконечного цикла
}	//конец функции main
