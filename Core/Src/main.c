/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include <sensor.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	FrameStart,
	Source,
	Destination,
	CommandLength,// if '00' go to data
	Command,
	Data,
	Argument,
	FrameEnd
} FrameDetection;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_BUF_LEN 1500// 20 ramek
#define RX_BUF_LEN 75
#define CRC8_POLYNOMIAL 0x07
#define CRC8_INITIAL_VALUE 0x00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#pragma GCC diagnostic ignored "-Wint-conversion"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static FrameDetection detection = FrameStart;
uint8_t frame_index = 0;
char src[2];
char dst[2];
char cmd_len[2];
char data[64];// podwójna wielkość dla obsługi kodowania
char cmd[63];// podwójna wielkość dla obsługi kodowania
char cmd_arg[60];// podwójna wielkość dla obsługi kodowania
char checksum[3];
char frame[75];//max length + '<' + '>' + '\0'
uint8_t cmd_length = 64; // zmienna pomocnicza do sprawdzania rozmiarów komend
bool data_check = true;
bool decoding = false;

volatile uint32_t read_interval = 1000;
//zmienne buforowe
volatile uint8_t BUF_RX[RX_BUF_LEN];
volatile uint8_t BUF_TX[TX_BUF_LEN];
volatile uint16_t empty_RX = 0;
volatile uint16_t busy_RX = 0;
volatile uint16_t empty_TX = 0;
volatile uint16_t busy_TX = 0;

volatile SensorMeasurment_t BUF_SENSOR[SENSOR_BUF_LEN];
volatile uint16_t empty_SENSOR = 0;
volatile uint16_t busy_SENSOR = 0;

//command errors
char* new_line = "\r\n\r";
char* e_cmd_too_long = "Command is too long";
char* e_cmd = "Unknown command";
char* e_arg = "Argument Error";
char* e_arg_too_long = "Argument Too Long";
char* e_index = "Invalid Index";
char* e_crc = "Validation Failed";
char* e_frame = "Frame error";
char* e_data_too_long = "Data too long";
char* e_len = "Command length error";
char* e_address = "Address Error";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int8_t USART_get_char() {//pobieranie z bufora odbiorczego
    // Sprawdź, czy bufor odbiorczy jest pełny
    if (busy_RX == empty_RX) {
        return -1;
    }
    // Pobierz znak z bufora
    int8_t sign = BUF_RX[busy_RX];
    busy_RX++;
    if (busy_RX >= RX_BUF_LEN) {
        busy_RX = 0; // zapętlenie bufora
    }
    return sign; // Zwróć odebrany znak
}

void USART_send(char* message){ //wysyłanie do bufora TX
    uint16_t i,idx = empty_TX;

    for(i=0; message[i] != '\0'; i++){ // przenosimy tekst z wywołania funkcji USART_send do tablicy BUF_TX[]

    	BUF_TX[idx] = message[i];
        idx++;
        if(idx >= TX_BUF_LEN) idx = 0;//zapętlenie idx
    } // cały tekst message[] znajduje się już teraz w BUF_TX[]
    __disable_irq(); //wyłączamy przerwania, bo poniżej kilka linii do zrobienia
    if ((busy_TX == empty_TX)&&(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)==SET))
      	//sprawdza czy bufor jest pusty
       	//i czy flaga jest ustawiona(czy jest gotowy do ponownej komunikacji)
    {
        empty_TX = idx;
        uint8_t tmp = BUF_TX[busy_TX];//znak do wysłania
        busy_TX++;
        if(busy_TX >= TX_BUF_LEN) busy_TX = 0;//zapętla bufor
        HAL_UART_Transmit_IT(&huart2, &tmp, 1);
    }
    else
    {
        empty_TX = idx;
    }
    __enable_irq(); //ponownie aktywujemy przerwania
}

void temp_log(void) {
    uint16_t lastIndex = (empty_SENSOR + SENSOR_BUF_LEN - 1) % SENSOR_BUF_LEN;
    char msg[32];
    sprintf(msg, "temperature: %.1fC", BUF_SENSOR[lastIndex].temperature);
    USART_send(msg);
}

void moist_log(void) {
    uint16_t lastIndex = (empty_SENSOR + SENSOR_BUF_LEN - 1) % SENSOR_BUF_LEN;
    char msg[32];
    sprintf(msg, "moisture: %u%%", BUF_SENSOR[lastIndex].moisture);
    USART_send(msg);
}

void sensor_log(void) {
    uint16_t lastIndex = (empty_SENSOR + SENSOR_BUF_LEN - 1) % SENSOR_BUF_LEN;
    char msg[32];
    sprintf(msg, "temp: %.1fC, moist: %u%%", BUF_SENSOR[lastIndex].temperature, BUF_SENSOR[lastIndex].moisture);
    USART_send(msg);
}

//Resetuje wszystkie tablice ramki i nasłuchiwanie
void Frame_reset() {
	//zapełnia tablice pustymi znakami, lekko redundantne rozwiązanie
	//ale zapewnia że dane z poprzedniej ramki nie zostaną przypadkowo wykorzystane ponownie
	detection = FrameStart;
	decoding = false;
	frame_index = 0;
	cmd_length = 64;
	data_check = true;
	memset(cmd,		'\0', 63);
	memset(cmd_arg, '\0', 60);
	memset(data,	'\0', 64);

}

//sprawdza poprawność rozmiaru tabli, true - error
bool Size_check(char* tbl, size_t max_size, char* error) {
    if (frame_index >= max_size) {
		USART_send(new_line);
		USART_send(error);
		USART_send(new_line);
		Frame_reset();
		return true;
	}
	return false;
}

// Funkcja do odwracania bitów w bajcie do crc
uint8_t Reverse_bits(uint8_t byte) {
    uint8_t reversed = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            reversed |= (1 << (7 - i));
        }
    }
    return reversed;
}

// Funkcja obliczająca CRC8 dla ciągu znaków
uint8_t Calculate_CRC8(char* data) {
    uint8_t crc = CRC8_INITIAL_VALUE;
    uint8_t length = strlen(data);  // Długość ciągu znaków

    for (uint8_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        byte = Reverse_bits(byte);  // Odwrócenie bitów w bajcie
        crc ^= byte;  // XOR z bieżącym bajtem

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {//sprawdza MSB
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;  // Przesunięcie i XOR z wielomianem
            } else {
                crc <<= 1;  // Tylko przesunięcie
            }
        }
    }

    return Reverse_bits(crc);  // Odwrócenie bitów CRC na koniec
}

//dokleja tablice x do gotowej ramki
void Frame_merge(char* tbl) {
	for (uint8_t i = 0;i < strlen(tbl);i++) {
		frame[frame_index] = tbl[i];
		frame_index++;
	}
}

//łączy wszystkie tablice ramki danych i dodaje sume kontrolną
void Merge_data_frame() {
	frame_index = 0;
	frame[frame_index++] = '<';
	Frame_merge(src);
	Frame_merge(dst);
	Frame_merge(cmd_len);
	Frame_merge(data);
	uint8_t crc = Calculate_CRC8(frame);
	char crc_str[3]; // 2 cyfry plus '\0'
	snprintf(crc_str, sizeof(crc_str), "%02X", crc); // konwersja hex do char
	Frame_merge(crc_str);
	frame[frame_index++] = '>';
	frame[frame_index] = '\0';
}

//łączy wszystkie tablice ramki komendy i dodaje sume kontrolną
void Merge_Command_frame() {
	frame_index = 0;
	frame[frame_index++] = '<';
	Frame_merge(src);
	Frame_merge(dst);
	Frame_merge(cmd_len);
	Frame_merge(cmd);
	frame[frame_index++] = '[';
	Frame_merge(cmd_arg);
	frame[frame_index++] = ']';
	uint8_t crc = Calculate_CRC8(frame);
	char crc_str[3]; // 2 cyfry plus '\0'
	snprintf(crc_str, sizeof(crc_str), "%02X", crc); // konwersja hex do char
	Frame_merge(crc_str);
	frame[frame_index++] = '>';
	frame[frame_index] = '\0';
}

void Frame_compare() {
    USART_send(new_line);
    if (data_check) { // DEBUG data frame
        Merge_data_frame();
        USART_send(frame);
        return;
    } else { // DEBUG command frame
        Merge_Command_frame();
        USART_send(frame);
    }

    if (strcmp(cmd, "cur") == 0) { // Sprawdzenie komendy cur
        if (cmd_arg[0] == '\0') {
            sensor_log();
        } else if (strcmp(cmd_arg, "t") == 0) {
            temp_log();
        } else if (strcmp(cmd_arg, "m") == 0) {
            moist_log();
        } else {
            USART_send(e_arg);
        }

    } else if (strcmp(cmd, "int") == 0) { // Ustawienie lub wyświetlenie interwału
        uint32_t tmp_int_from_frame = atoi(cmd_arg);

        if (cmd_arg[0] == '\0') { // Wyświetlanie interwału w sekundach
            char tmp_current_int[11]; // max 4 miliony + \0
            sprintf(tmp_current_int, "%u", read_interval / 1000); // Konwersja na sekundy
            USART_send("Current Interval: ");
            USART_send(tmp_current_int);
            USART_send("s"); // Dodanie jednostki sekund
        } else { // Ustawienie nowego interwału
            read_interval = tmp_int_from_frame * 1000; // Konwersja na milisekundy

            if (tmp_int_from_frame == 0) { // Jeśli podano 0, wyłącz interwał
                USART_send("Interval disabled");
            } else { // W przeciwnym razie ustaw interwał na podaną wartość
                char tmp_current_int[11];
                sprintf(tmp_current_int, "%u", tmp_int_from_frame); // Wypisujemy wartość w sekundach
                USART_send("Interval set to: ");
                USART_send(tmp_current_int);
                USART_send("s"); // Dodanie jednostki sekund
            }
        }

    } else if (strcmp(cmd, "data") == 0) { // Obsługa archiwum pomiarów
        if (cmd_arg[0] == '\0') { // Wyświetlenie całego archiwum
            if (busy_SENSOR == empty_SENSOR) { // Sprawdzenie czy bufor nie jest pusty
                USART_send("No data available");
            } else {
                uint16_t idx = busy_SENSOR;
                while (idx != empty_SENSOR) {
                    char msg[50];
                    sprintf(msg, "id: %.1fC, %u%%", BUF_SENSOR[idx].temperature, BUF_SENSOR[idx].moisture);
                    USART_send(msg);
                    USART_send(new_line);
                    idx = (idx + 1) % SENSOR_BUF_LEN;
                }
            }
        } else { // indeks -1 lub konkretna wartość
            int requested_index = atoi(cmd_arg);
            if (requested_index == -1) { // Ostatni pomiar
                if (busy_SENSOR == empty_SENSOR) {
                    USART_send("No data available");
                } else {
                    uint16_t lastIndex = (empty_SENSOR + SENSOR_BUF_LEN - 1) % SENSOR_BUF_LEN;
                    char msg[50];
                    sprintf(msg, "id: %.1fC, %u%%", BUF_SENSOR[lastIndex].temperature, BUF_SENSOR[lastIndex].moisture);
                    USART_send(msg);
                }
            } else if (requested_index < 0 || requested_index >= SENSOR_BUF_LEN) { // Indeks poza zakresem
                USART_send(e_index);
            } else { // Pobranie konkretnego indeksu
                uint16_t stored_values = (empty_SENSOR >= busy_SENSOR) ? //sprawdza czy bufor sie zawija
                                          (empty_SENSOR - busy_SENSOR) : //zbiera wartości od końca tablicy
                                          (SENSOR_BUF_LEN - busy_SENSOR + empty_SENSOR);// od początku
                if (requested_index >= stored_values) { // Sprawdzenie, czy indeks istnieje
                    USART_send(e_index);
                } else {
                    uint16_t idx = busy_SENSOR;
                    for (int i = 0; i < requested_index; i++) {
                        idx = (idx + 1) % SENSOR_BUF_LEN;
                    }
                    char msg[50];
                    sprintf(msg, "id: %.1fC, %u%%", BUF_SENSOR[idx].temperature, BUF_SENSOR[idx].moisture);
                    USART_send(msg);
                }
            }
        }

    } else if (strcmp(cmd, "purge") == 0) { // Czyszczenie bufora
        if (cmd_arg[0] == '\0') {
            for (int i = 0; i < SENSOR_BUF_LEN; i++) {
                BUF_SENSOR[i].temperature = 0.0f;
                BUF_SENSOR[i].moisture = 0;
            }
            // Resetowanie wskaźników bufora
            busy_SENSOR = 0;
            empty_SENSOR = 0;
            USART_send("Cleared all data");
        } else {
            USART_send(e_arg);
        }

    } else {
        USART_send(e_cmd);
    }

    Frame_reset();
    USART_send(new_line);
}


//pomocnicza struktura do dekodowania aby przekazać wynik z flagą
typedef struct {
    char decoded_sign; // Rozpoznany znak
    bool is_decoded;   // Czy znak został zdekodowany
} DecodeResult;

// funkcja do dekodowania
DecodeResult Decode(char sign) {
    DecodeResult result = {sign, false}; // Domyślnie: znak nie jest zdekodowany

    if (decoding) {
        decoding = false; // reset stanu po przetworzeniu znaku
        if 		(sign == '1')result = (DecodeResult){'<', true};
        else if (sign == '2')result = (DecodeResult){'>', true};
        else 				 result = (DecodeResult){sign, false}; // Nieoczekiwany znak
    } else if (sign == '/') {
        decoding = true;
        result = (DecodeResult){'\0', false}; // Ignoruj '/'
    }

    return result;
}

void FrameRd()
{
	int8_t x = USART_get_char();
	if (x < 0) return;
	char sign = x;

	if (sign == '<'){
		Frame_reset();
		detection = Source;
		return;
	}

	if (decoding == FrameStart) return; // zapobiega błędnemu rozpoznawaniu kodowania poza ramką
	DecodeResult decode_result = Decode(sign);

	switch (detection){
		case FrameStart:
			break;
		case Source:
			src[frame_index] = sign;
			frame_index++;
			if (frame_index == 2){
				frame_index = 0;
				detection = Destination;
			}
			break;
		case Destination:
			dst[frame_index] = sign;
			frame_index++;
			if (frame_index == 2){
				frame_index = 0;
				detection = CommandLength;
			}
			break;
		case CommandLength:
			cmd_len[frame_index] = sign;
			frame_index++;
			if (frame_index == 2){
				decoding = false;
				frame_index = 0;
				data_check = false;
				detection = Command;
				cmd_length = atoi(cmd_len);
				if (strcmp(cmd_len, "00") == 0) {
					data_check = true;
					detection = Data;
					cmd_length = 0;
				}
			}
			if (!isdigit(sign)) {
				USART_send(new_line);
				USART_send(e_len);
				USART_send(new_line);
				Frame_reset();
			}
			break;
		case Command:
			if(Size_check(cmd, 63, e_cmd_too_long)) return;
			sign = decode_result.decoded_sign;
			if(sign == '\0') return;
			if (sign == '['){	//zaczyna argument
				frame_index = 0;
				decoding = false;
				detection = Argument;
				return;
			} else{
				cmd[frame_index] = sign; //sprawdza reszte liter komendy
				frame_index++;
				cmd_length--;
			}
			if (cmd_length < 0) {
				USART_send(new_line);
				USART_send(e_cmd_too_long);
				USART_send(new_line);
				Frame_reset();
			}
			break;
		case Argument:
			if(Size_check(cmd_arg, 60, e_arg_too_long)) return;
			sign = decode_result.decoded_sign;
			if(sign == '\0') return;
			if (sign == ']'){  // konczy argument
				frame_index = 0;
				decoding = false;
				detection = FrameEnd;
				return;
			} else{
				cmd_arg[frame_index] = sign;
				frame_index++;
				cmd_length--;
			}
			if (cmd_length < 0) {
				USART_send(new_line);
				USART_send(e_arg_too_long);
				USART_send(new_line);
				Frame_reset();
			}
			break;
		case Data:
			if(Size_check(data, 64, e_data_too_long)) return;
			sign = decode_result.decoded_sign;
			if(sign == '\0') {return;}
			else if (sign == '>' && !decode_result.is_decoded){  // konczy dane, else aby rozkodowany znak nie kończł ramki
				Frame_compare();
				return;
			}
			data[frame_index] = sign;
			frame_index++;
			if (frame_index >= 64) {
				USART_send(new_line);
				USART_send(e_data_too_long);
				USART_send(new_line);
				Frame_reset();
			}
			break;
		case FrameEnd:
			if (sign == '>'){
				Frame_compare();
			} else {
				USART_send(new_line);
				USART_send(e_frame);
				USART_send(new_line);
				Frame_reset();
			}
			break;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){//wypisywanie w konsoli
	if(huart == &huart2) {//weryfikacja poprawności wskaźnika
		if(busy_TX != empty_TX){
			uint8_t tmp = BUF_TX[busy_TX];
			busy_TX++;
			if(busy_TX >= TX_BUF_LEN) busy_TX = 0;//zapętlenie indexu
			HAL_UART_Transmit_IT(&huart2, &tmp, 1);
		}
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){//odbieranie z konsoli
	if(huart == &huart2) {//weryfikacja poprawności wskaźnika
		if ((empty_RX + 1) % RX_BUF_LEN != busy_RX) {
			//index update
			empty_RX++;
			if(empty_RX >= RX_BUF_LEN)	empty_RX = 0;// zapętlenie bufora
			HAL_UART_Receive_IT(&huart2, &BUF_RX[empty_RX], 1);
		}
		HAL_UART_Transmit_IT(&huart2, &BUF_RX[empty_RX-1], 1);//DEBUG
		//wyświetla ostatni odebrany znak w konsoli do debugowania
	}
}

void welcome() {
	USART_send(new_line);
	USART_send("Hi!!!");
	USART_send(new_line);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  welcome();
  Frame_reset();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &BUF_RX[0],1);
  HAL_UART_RxCpltCallback(&huart2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  sensor_process();
	  FrameRd();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
