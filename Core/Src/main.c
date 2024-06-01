/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author  		: A. Nahuel Medina
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "ESP01.h"

#include "util.h"
#include "sh1106.h"
#include "MPU6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* COMUNICACION CODE BEGIN PV */
volatile _rx rxUSB;
_tx txUSB;
uint8_t bufUSB_Rx[256];
uint8_t bufUSB_Tx[256];
uint8_t bytes_to_send;

/*Comunicacion USART (comunicacion entre el micro y la ESP01)*/
volatile _rx rxUSART;
_tx txUSART;
uint8_t bufUSART_Rx[256];
uint8_t bufUSART_Tx[256];

/*Transmision desde el micro a la pc mediante WIFI*/
_tx txAUX;
uint8_t bufAUX_Tx[256];

_sESP01Handle ESP01Manager;
//Configuracion de valores para conexion UDP
// FACULTAD

const char SSID[]= "FCAL-Personal";
const char PASSWORD[]= "fcal-uner+2019";
const char RemoteIP[]="172.22.243.121";
uint16_t RemotePORT = 30010;
uint16_t LocalPORT = 30001;


//CASA
/*
const char SSID[]= "ANM2";
const char PASSWORD[]= "anm157523";
const char RemoteIP[]="192.168.0.11";
uint16_t RemotePORT = 30010;
uint16_t LocalPORT = 30001;
*/

/* COMUNICACION CODE EDN PV */

/* MPU6050 CODE BEGIN PV */
extern int32_t Ax, Ay, Az, Gx, Gy, Gz;
/* MPU6050 CODE END PV */

/* MANIPULACION DE DATOS Y BANDERAS CODE BEGIN PV */
uint8_t robotMode = IDLE;
_work w;
_flag flag1;
_flag flag2;
/* MANIPULACION DE DATOS Y BANDERAS CODE END PV */

/* HEARTBEAT CODE BEGIN PV */
uint32_t SECUENCE_LED_IDLE = MASK_IDLE;
uint32_t SECUENCE_LED_FOLLOW_LINE = MASK_FOLLOW_LINE;
uint8_t indexHb = 0;
/* HEARTBEAT CODE END PV */

/* ADC CODE BEGIN PV */
uint16_t ADCData[LENGHT_MAX_CONV_ADC][8];
uint16_t indexADC;
uint16_t analogValue_IR[8];
uint16_t digitalValue_IR[8];

//Variables de calibracion


//PID_IZQ
uint8_t posMINCenter=0,posMINRight=0,posMINLeft=0;
uint16_t sensorValue=0;
int32_t xMin=0,fx2_fx3,fx2_fx1,x2_x1,x2_x3,x2_x1cuad,x2_x3cuad,denominador, numerador, cuenta;
const int coorRefer[]={-50,-40,-30,-20,-10,10,20,30,40,50};

int16_t posicion_IR[9];
uint16_t umbralIR = 2000;
int16_t setpoint = 35;

int16_t suma_ponderada, suma, posicion;

int32_t KP_IZQ = 10;
int32_t KI_IZQ = 0;
int32_t KD_IZQ = 0;
//PID_DER
int32_t KP_DER = 10;
int32_t KI_DER = 0;
int32_t KD_DER = 0;

int16_t VEL_IZQ = 40;
int16_t VEL_DER = 20;

int16_t freno_adelante = 50;
int16_t freno_atras = -50;

//Variables PID
int32_t e_t = 0;
int32_t e_t_integral = 0;
int32_t e_t_derivativo = 0;
int32_t e_t_last = 0;
int32_t u_t_DER = 0;
int32_t u_t_IZQ = 0;

/* ADC CODE END PV */
int32_t VELX;
int32_t velx_last;

int32_t POSX;
int32_t posx_last;

int32_t ANGLEZ;
int32_t anglez_last;


/*Contador para TICKER 1MS*/
uint8_t time10ms;
uint8_t time100ms;
uint8_t time1seg;
uint16_t time5seg;


/* VARIABLES ESPECIFICAS SEGUN COMANDO CODE BEGIN PV */
const char FIRMWAREVERSION[] = "20231007_0102";
/* VARIABLES ESPECIFICAS SEGUN COMANDO CODE END PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* COMUNICACION CODE BEGIN PFP */
void OnUSBRx(uint8_t *buf, int length);
void DecodeHeader(_rx *RX, _tx *tx);                           	  //funcion que decodifica cabecera
void DecodeCMD(_rx *RX, _tx *tx);                                 //decodifica comando
void PutBufOnTx(_tx *TX, uint8_t *buf, uint8_t length);           //escribe datos en buffer de transmision
void PutByteOnTx(_tx *TX, uint8_t value);                         //escribe solo un valor en buffer de transmision
void PutHeaderOnTx(_tx *TX, uint8_t id, uint8_t lCmd);            //cabecera en buf de tx
void CalcAndPutCksOnTx(_tx *TX);                                  //calcula checksum
void PutStrOnTx(_tx *TX, const char *str);                        //escribe texto en buf de tx
uint8_t GetByteFromRx (_rx *RX, int8_t pre, int8_t pos);          //lee datos de buf de recepcion
void TransmitUSB();
void TransmitUSART();
/* COMUNICACION CODE END PFP */

/* SET LEDS CODE BEGIN PFP */
void setMode();
void hearbeatTask();
/* SET LEDS CODE END PFP */

/* PWM CODE BEGIN PV */
void PWM_init();

/* PWM CODE END PV */

/* ESP01 CODE BEGIN PFP */
void CHENState (uint8_t value);
int PutByteOnTxESP01(uint8_t value);
void GetESP01StateChange(_eESP01STATUS newState);
void GetESP01dbg(const char *dbgStr);
/* ESP01 CODE END PFP */

/* CONTROL CODE BEGIN PFP */
void RobotWorkout();
void MotorControl(int16_t setMotorRight, int16_t setMotorLeft);

void findLine();
void analogToDigital();
void correctionPID();
/* CONTROL CODE END PFP */

/* GRAFICAS SH1106 CODE BEGIN PFP */
void graficar_oled();
/* GRAFICAS SH1106 CODE END PFP */

/* CALCULO DE POSICION CODE BEGIN PFP */
void position_calculation();

/* UPDATE CODE BEGIN PFP */
void updateDataQT();
/* UPDATE CODE END PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){						//Timer 2 (Se da cada 250us = 42 * 500 / 84Mhz)
	if(htim->Instance == TIM2){
		time10ms--;
		if(!time10ms){
			IS10MS = 1;																//habilita la funcion On1ms() en el main, que maneja todos los tickers
			time10ms = 40;															// 250us * 4 = 1ms
		}
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADCData[indexADC], 8);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	uint16_t indexADC_aux = indexADC;

	analogValue_IR[0] = ADCData[indexADC_aux][0];
	analogValue_IR[1] = ADCData[indexADC_aux][1];
	analogValue_IR[2] = ADCData[indexADC_aux][2];
	analogValue_IR[3] = ADCData[indexADC_aux][3];
	analogValue_IR[4] = ADCData[indexADC_aux][4];
	analogValue_IR[5] = ADCData[indexADC_aux][5];
	analogValue_IR[6] = ADCData[indexADC_aux][6];
	analogValue_IR[7] = ADCData[indexADC_aux][7];
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	ESP01_WriteRX(bufUSART_Rx[rxUSART.iw]);
	rxUSART.iw++;
	rxUSART.iw &= rxUSART.maskSize;									//Mantiene la circularidad del buffer
	HAL_UART_Receive_IT(&huart1, &rxUSART.buf[rxUSART.iw], 1);		//Lo recibido en el uart1, lo almaceno en rxUSART[], recibo de a 1 byte															//Permite que el procesador haga cosas mas importantes mientras el periferico del USART se encarga de la recepcion

}

void OnUSBRx(uint8_t *myBuf, int length){
	for(int i=0; i<length; i++){
	  	rxUSB.buf[rxUSB.iw++] = myBuf[i];			//Cargo el dato recibido en el USB en el buffer de recepcion USB
	  	rxUSB.iw &= rxUSB.maskSize;           	//Mantiene el buffer circular
	}
	DATAREADY = 1;
}

void DecodeHeader(_rx *RX, _tx *tx){												//Decodifico la cabecera de lo que llegue por USB
    uint8_t i;

    i = RX->iw;                         	//auxiliar para que no cambie valor del indice de escritura mientras recorro el buffer de recepcion

    while(RX->ir != i){
        switch(RX->header){
            case 0:
                if(RX->buf[RX->ir] == 'U'){
                    RX->header = 1;
                    RX->timeout = 5;
                }
            break;
            case 1:
                if(RX->buf[RX->ir] == 'N'){
                    RX->header = 2;
                } else {
                    RX->header = 0;
                    RX->ir--;
                }
            break;
            case 2:
                if(RX->buf[RX->ir] == 'E'){
                    RX->header = 3;
                } else {
                    RX->header = 0;
                    RX->ir--;
                }
            break;
            case 3:
                if(RX->buf[RX->ir] == 'R'){
                    RX->header = 4;
                } else {
                    RX->header = 0;
                    RX->ir--;
                }
            break;
            case 4: //length
                RX->nbytes = RX->buf[RX->ir];
                RX->header = 5;
                RX->nbytesdata = RX->buf[RX->ir]-2;
            break;
            case 5: //token
                if(RX->buf[RX->ir] == ':'){
                    RX->header = 6;
                    RX->iData = RX->ir + 1;     //indice a partir de donde se encuentran los datos del comando
                    RX->iData &= RX->maskSize;
                    RX->cks = 'U' ^ 'N' ^ 'E' ^ 'R' ^ RX->nbytes ^ ':';
                } else {
                    RX->header = 0;
                    RX->ir--;
                }
            break;
            case 6: //datos de comando
                RX->nbytes--;
                if(RX->nbytes > 0){
                    RX->cks ^= RX->buf[RX->ir];
                } else {
                    RX->header = 0;
                    if(RX->cks == RX->buf[RX->ir]) 		//llega al checksum
                        RX->ISCMD = 1;					//es un comando valido
                }
            break;
            default:
                RX->header = 0;
            break;
        }
        RX->ir &= RX->maskSize;
        RX->ir++;
        RX->ir &= RX->maskSize;
    }

}

void DecodeCMD(_rx *RX, _tx *tx){												//Decodifico el comando que llegue por USB
	RX->ISCMD = 0;

    switch(RX->buf[RX->iData]){                 //segun el comando enviado
        case 0xF0:  //ALIVE
            PutHeaderOnTx(tx, 0xF0, 2);
            PutByteOnTx(tx, 0x0D);
            CalcAndPutCksOnTx(tx);
            tx->length = 3;
            break;
        case 0xF1: //firmware
            PutHeaderOnTx(tx, 0xF1, strlen(FIRMWAREVERSION) + 1);
            PutStrOnTx(tx, FIRMWAREVERSION);
            CalcAndPutCksOnTx(tx);
            break;
        case 0xA0: //ADC
            PutHeaderOnTx(tx, 0xA0, 16 + 1);
            PutBufOnTx(tx, (uint8_t *)analogValue_IR ,16);
            CalcAndPutCksOnTx(tx);
            break;
        case 0xA1: //MPU
        	PutHeaderOnTx(tx, 0xA1, 25);
        	MPU6050_Read_Accel();
        	w.i32 = Ax;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	w.i32 = Ay;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	w.i32 = Az;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);

        	MPU6050_Read_Gyro();
        	w.i32 = Gx;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	w.i32 = Gy;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	w.i32 = Gz;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	CalcAndPutCksOnTx(tx);
		break;

        case 0xA2: //DATACONTROL
        	PutHeaderOnTx(tx, 0xA2, 32);
        	w.i32 = KP_IZQ;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	w.i32 = KI_IZQ;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	w.i32 = KD_IZQ;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	w.i16[0] = VEL_IZQ;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);

        	w.i32 = KP_DER;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	w.i32 = KI_DER;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	w.i32 = KD_DER;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	w.i16[0] = VEL_DER;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);

        	PutByteOnTx(tx, robotMode);
        	CalcAndPutCksOnTx(tx);
        break;

        case 0xA3: //SETDATACONTROL
        	//KP_IZQ---------------------------
        	w.u8[0] = RX->buf[RX->iData+1];
        	w.u8[1] = RX->buf[RX->iData+2];
        	w.u8[2] = RX->buf[RX->iData+3];
        	w.u8[3] = RX->buf[RX->iData+4];
        	KP_IZQ = w.i32;
        	//KI_IZQ---------------------------
        	w.u8[0] = RX->buf[RX->iData+5];
        	w.u8[1] = RX->buf[RX->iData+6];
        	w.u8[2] = RX->buf[RX->iData+7];
        	w.u8[3] = RX->buf[RX->iData+8];
        	KI_IZQ = w.i32;
        	//KD_IZQ---------------------------
        	w.u8[0] = RX->buf[RX->iData+9];
        	w.u8[1] = RX->buf[RX->iData+10];
        	w.u8[2] = RX->buf[RX->iData+11];
        	w.u8[3] = RX->buf[RX->iData+12];
        	KD_IZQ = w.i32;
        	//VEL_IZQ--------------------------
        	w.u8[0] = RX->buf[RX->iData+13];
        	w.u8[1] = RX->buf[RX->iData+14];
        	VEL_IZQ = w.i16[0];

        	//KP_DER---------------------------
        	w.u8[0] = RX->buf[RX->iData+15];
        	w.u8[1] = RX->buf[RX->iData+16];
        	w.u8[2] = RX->buf[RX->iData+17];
        	w.u8[3] = RX->buf[RX->iData+18];
        	KP_DER = w.i32;
        	//KI_DER---------------------------
        	w.u8[0] = RX->buf[RX->iData+19];
        	w.u8[1] = RX->buf[RX->iData+20];
        	w.u8[2] = RX->buf[RX->iData+21];
        	w.u8[3] = RX->buf[RX->iData+22];
        	KI_DER = w.i32;
        	//KD_DER---------------------------
        	w.u8[0] = RX->buf[RX->iData+23];
        	w.u8[1] = RX->buf[RX->iData+24];
        	w.u8[2] = RX->buf[RX->iData+25];
        	w.u8[3] = RX->buf[RX->iData+26];
        	KD_DER = w.i32;
        	//VEL_DER--------------------------
        	w.u8[0] = RX->buf[RX->iData+27];
        	w.u8[1] = RX->buf[RX->iData+28];
        	VEL_DER = w.i16[0];

        	PutHeaderOnTx(tx, 0xA3, 2);
        	PutByteOnTx(tx, 0x0D);
        	CalcAndPutCksOnTx(tx);

        	//Realizar un comando de CHECK
        break;

        case 0xA4: //SETFREQ
        	PutHeaderOnTx(tx, 0xA4, 5);
        	w.i32 = e_t;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	CalcAndPutCksOnTx(tx);
        break;
        case 0xA5: //SETROBOTMODE
        	robotMode = RX->buf[RX->iData+1];
        break;
        case 0xA6:
        	PutHeaderOnTx(tx, 0xA6, 5);
        	w.i32 = e_t;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	CalcAndPutCksOnTx(tx);
        break;
        default:
            PutHeaderOnTx(tx, RX->buf[RX->iData], 2);        //Devuelve 0xFF si hay un comando que no se interpretó
            PutByteOnTx(tx, 0xFF);
            CalcAndPutCksOnTx(tx);
        break;
    }
}

void PutBufOnTx(_tx *TX, uint8_t *buf, uint8_t length){								//Pongo buffer en Tx (Sirve para payload)
    uint8_t i;

    for(i=0; i < length; i++){
        TX->buf[TX->iw++] = buf[i];
        TX->iw &= TX->maskSize;
    }
}

void PutByteOnTx(_tx *TX, uint8_t value){											//Coloco byte en buffer de transmision
	TX->buf[TX->iw++] = value;
    TX->iw &= TX->maskSize;
}

void PutHeaderOnTx(_tx *TX, uint8_t id, uint8_t lCmd){								//Coloco cabecera hasta id en buffer de transmision
	TX->buf[TX->iw++] = 'U';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = 'N';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = 'E';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = 'R';
    TX->iw &= TX->maskSize;
    TX->length = lCmd; //largo del comando
    TX->buf[TX->iw++] = lCmd + 1; // lCmd = ID + payload || 1 byte se le suma por el checksum
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = ':';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = id;
    TX->iw &= TX->maskSize;
}

void CalcAndPutCksOnTx(_tx *TX){													//Calculo checksum y coloco en buffer de transmision
    uint8_t cks, i;

    //recorro el indice y voy guardando el cks
    cks = 0;
    i = TX->length +6;
    i = TX->iw-i;
    i &= TX->maskSize;
    while(i != TX->iw){
        cks ^= TX->buf[i++];
        i &= TX->maskSize;
    }

    TX->buf[TX->iw++] = cks;
    TX->iw &= TX->maskSize;
}

void PutStrOnTx(_tx *TX, const char *str){                  						//Coloco cadena de caracteres en buffer de transmision
    uint8_t i = 0;

    while(str[i]){
        TX->buf[TX->iw++] = str[i++];
        TX->iw &= TX->maskSize;
    }

}

uint8_t GetByteFromRx (_rx *RX, int8_t pre, int8_t pos){							//Obtengo payload del buffer de recepcion
    uint8_t aux;

    RX->iData += pre;
    RX->iData &= RX->maskSize;
    aux = RX->buf[RX->iData];
    RX->iData += pos;
    RX->iData &= RX->maskSize;

    return aux;
}

void TransmitUSB(){
	uint8_t freeBytes;

	freeBytes = 0;
	if(!AUX_USB){
	   freeBytes = 255 - txUSB.ir;							//Calculo cuanto mas puede avanzar mi indice de lectura para enviar el dato
	  AUX_USB = 1;
	}
	if((txUSB.length+7 <= freeBytes) && (bytes_to_send == 0)){	//Si en mi indice de lectura me da para mandar el largo del comando lo mando directo
	  if(USBD_OK == CDC_Transmit_FS(&txUSB.buf[txUSB.ir], txUSB.length+7)){//Envio dato
		  bytes_to_send = 0;											//Cantidad de datos que faltaron enviar
		  txUSB.ir = txUSB.iw;
		  AUX_USB = 0;
	  }
	}else{
	  if((txUSB.length+7 > freeBytes) && (bytes_to_send == 0)){//Si el largo de mi comando no entra en el indice de lectura envio por partes
		  if(USBD_OK == CDC_Transmit_FS(&txUSB.buf[txUSB.ir], freeBytes)){//Envio primera parte del dato
			  txUSB.ir += freeBytes;
			  bytes_to_send = (txUSB.length+7) - freeBytes;				//Cantidad de datos que faltaron enviar
		  }
	  }
	  if(bytes_to_send > 0){
		  if(USBD_OK == CDC_Transmit_FS(&txUSB.buf[txUSB.ir], bytes_to_send)){//Envio dato bytes_to_send que falto enviar
			  bytes_to_send = 0;											//Cantidad de datos que faltaron enviar
			  txUSB.ir = txUSB.iw;
			  AUX_USB = 0;
		  }
	  }
	}
}

void TransmitUSART(){
	uint16_t len;

	len = txUSART.iw - txUSART.ir;
	len &= 0x00FF;
	if(ESP01_Send(txUSART.buf, txUSART.ir, len, 256) == ESP01_SEND_READY)
		txUSART.ir = txUSART.iw;

}

void setMode(){
	if(STARTSTOP){
		STARTSTOP = 0;
		robotMode = IDLE;
		HEARBEAT_STATUS = 0;
		MotorControl(0,0);
	}else{
		STARTSTOP = 1;
		robotMode = FOLLOW_LINE;
		HEARBEAT_STATUS = 1;
	}
}

void hearbeatTask(){
	if(HEARBEAT_STATUS){
		HAL_GPIO_WritePin(HEARBEAT_LED_GPIO_Port, HEARBEAT_LED_Pin, SECUENCE_LED_IDLE & (1<<indexHb));
		HAL_GPIO_WritePin(LED_MODE_GPIO_Port, LED_MODE_Pin, SECUENCE_LED_IDLE & (1<<indexHb));
		indexHb++;
		if(indexHb >= 32){
			indexHb = 0;
		}
	}else{
		HAL_GPIO_WritePin(HEARBEAT_LED_GPIO_Port, HEARBEAT_LED_Pin, SECUENCE_LED_FOLLOW_LINE & (1<<indexHb));
		HAL_GPIO_WritePin(LED_MODE_GPIO_Port, LED_MODE_Pin, SECUENCE_LED_FOLLOW_LINE & (1<<indexHb));
		indexHb++;
		if(indexHb >= 32){
			indexHb = 0;
		}
	}
}

void PWM_init(){

	HAL_TIM_Base_Start(&htim4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

}

void On10ms(){	//ticker que indica cuando paso 10ms
	time5seg--;
	if(!time5seg){
		IS5SEG = 1;
		time5seg = 500;
	}

	time1seg--;
	if(!time1seg){
		IS1S = 1;
		time1seg = 100;
	}

	time100ms--;
	if(!time100ms){
		IS100MS = 1;
		time100ms = 10;
		hearbeatTask();
	}

	ESP01_Timeout10ms();


}

void On5s(){	//ticker que indica cuando paso 5s

	PutHeaderOnTx(&txUSART, 0xF0, 2);
	PutByteOnTx(&txUSART, 0x0D);
	CalcAndPutCksOnTx(&txUSART);

	/*PutHeaderOnTx(&txUSART, 0xA6, 21);
	w.i32 = VELX;
	PutByteOnTx(&txUSART, w.u8[0]);
	PutByteOnTx(&txUSART, w.u8[1]);
	PutByteOnTx(&txUSART, w.u8[2]);
	PutByteOnTx(&txUSART, w.u8[3]);
	w.i32 = POSX;
	PutByteOnTx(&txUSART, w.u8[0]);
	PutByteOnTx(&txUSART, w.u8[1]);
	PutByteOnTx(&txUSART, w.u8[2]);
	PutByteOnTx(&txUSART, w.u8[3]);
	w.i32 = denominador;
	PutByteOnTx(&txUSART, w.u8[0]);
	PutByteOnTx(&txUSART, w.u8[1]);
	PutByteOnTx(&txUSART, w.u8[2]);
	PutByteOnTx(&txUSART, w.u8[3]);
	w.i32 = e_t;
	PutByteOnTx(&txUSART, w.u8[0]);
	PutByteOnTx(&txUSART, w.u8[1]);
	PutByteOnTx(&txUSART, w.u8[2]);
	PutByteOnTx(&txUSART, w.u8[3]);
	w.i32 = u_t_IZQ;
	PutByteOnTx(&txUSART, w.u8[0]);
	PutByteOnTx(&txUSART, w.u8[1]);
	PutByteOnTx(&txUSART, w.u8[2]);
	PutByteOnTx(&txUSART, w.u8[3]);

	CalcAndPutCksOnTx(&txUSART);*/

	//graficar_oled();

	/*PutHeaderOnTx(&txUSART, 0xF1, 16);
	PutStrOnTx(&txUSART, ESP01_GetLocalIP());
	CalcAndPutCksOnTx(&txUSART);*/
}

void CHENState (uint8_t value){
	HAL_GPIO_WritePin(AUX1_ESP_GPIO_Port, AUX1_ESP_Pin, value);
}

int PutByteOnTxESP01(uint8_t value)
{
	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)){
		USART1->DR = value;
		return 1;
	}
	return 0;
}

void GetESP01StateChange(_eESP01STATUS newState){

	if(newState == ESP01_WIFI_CONNECTED){
		PutStrOnTx(&txUSB, "+&ESPWIFI CONNECTED\n");
	}
	if(newState == ESP01_WIFI_NEW_IP){
		PutStrOnTx(&txUSB, "+&ESPIP: ");
		PutStrOnTx(&txUSB, ESP01_GetLocalIP());
		PutStrOnTx(&txUSB, "\n");
	}
	if(newState == ESP01_UDPTCP_CONNECTED){
		PutStrOnTx(&txUSB, "+&UDP CONNECTED\n");
		//timeOutAliveUDP =10;

	}
	if(newState == ESP01_SEND_OK){
		PutStrOnTx(&txUSB, "+&UDP SEND OK\n");
	}
}

void GetESP01dbg(const char *dbgStr){
	//PutStrOnTx(&txUSB, dbgStr);
}


/* CONTROL CODE BEGIN PFP */

void RobotWorkout(){

	position_calculation();

	switch(robotMode){
	case IDLE:

		break;
	case FOLLOW_LINE:
		findLine();
		analogToDigital();
		correctionPID();
		break;
	}

}
//------------------------CONTROL DE MOTORES-----------------------------
void MotorControl(int16_t setMotorRight, int16_t setMotorLeft){
	int16_t auxSetMotor;

	auxSetMotor = (abs(setMotorRight)*TIM4->ARR)/100;
	if(setMotorRight >= 0){
		//SET MOTOR DERECHO ADELANTE
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, auxSetMotor);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	}else{
		//SET MOTOR DERECHO REVERSA
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, auxSetMotor);
	}

	auxSetMotor = (abs(setMotorLeft)*TIM4->ARR)/100;
	if(setMotorLeft >= 0){
		//SET MOTOR IZQUIERDO ADELANTE
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, auxSetMotor);
	}else{
		//SET MOTOR IZQUIERDO REVERSA
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, auxSetMotor);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	}

}
//------------------------CONTROL CUADRATICO-----------------------------

void findLine(){
	int32_t aux[10];

	sensorValue = analogValue_IR[0];
	posMINCenter = 1;

	for(uint8_t f=0 ; f<8 ; f++){ //busco el valor min
		if(sensorValue > analogValue_IR[f]){
			sensorValue = analogValue_IR[f];
			posMINCenter = f+1;
		}
		aux[f+1] = analogValue_IR[f];
	}
 	//posMINCenter++;

 	aux[0] = aux[2];
 	aux[9] = aux[7];

	posMINRight = posMINCenter - 1;
	posMINLeft = posMINCenter + 1;

	fx2_fx3 = aux[posMINCenter]-aux[posMINRight];
	fx2_fx1 = aux[posMINCenter]-aux[posMINLeft];

	//saco distancia a 0
	x2_x1 = coorRefer[posMINCenter]-coorRefer[posMINLeft];
	x2_x3 = coorRefer[posMINCenter]-coorRefer[posMINRight];

	x2_x1cuad = (x2_x1*x2_x1);
	x2_x3cuad = (x2_x3*x2_x3);

	denominador = (2*(x2_x1*fx2_fx3 - x2_x3*fx2_fx1));

	if(denominador != 0){
		xMin = coorRefer[posMINCenter]-((x2_x1cuad*fx2_fx3 - x2_x3cuad*fx2_fx1)/denominador);
	}

	e_t = -xMin;

}

//---------------------------CONTROL PID---------------------------------
void correctionPID(){
	int16_t VEL_DER_aux,VEL_IZQ_aux;


	//e_t_integral = e_t_integral + (e_t*TM) + ((TM*(e_t - e_t_last))/2);

	if(e_t_integral > 100){e_t_integral = 100;}
	if(e_t_integral < -100){e_t_integral = -100;}

	//e_t_derivativo= (e_t - e_t_last)/TM;

	e_t_last = e_t;

	//PID MOTOR DERECHO
	u_t_DER= (KP_DER * e_t) + (KD_DER * e_t_derivativo) + (KI_DER * e_t_integral);
	//PID MOTOR IZQUIERDO
	u_t_IZQ= (KP_IZQ * e_t) + (KD_IZQ * e_t_derivativo) + (KI_IZQ * e_t_integral);


	VEL_DER_aux = (int16_t)(VEL_DER + u_t_DER/10);
	VEL_IZQ_aux = (int16_t)(VEL_IZQ - u_t_IZQ/10);

	if(VEL_DER_aux > VEL_MAX){VEL_DER_aux = VEL_MAX;}
	if(VEL_IZQ_aux > VEL_MAX){VEL_IZQ_aux = VEL_MAX;}

	if(posicion >= 65){
		MotorControl(freno_adelante,freno_atras);
	}else if(posicion <= 5){
		MotorControl(freno_atras, freno_adelante);
	}else{
		MotorControl(VEL_DER_aux, VEL_IZQ_aux);
	}

	if(IS1S){
		IS1S = 0;
		PutHeaderOnTx(&txUSART, 0xA4, 5);
		w.i32 = e_t;
		PutByteOnTx(&txUSART, w.u8[0]);
		PutByteOnTx(&txUSART, w.u8[1]);
		PutByteOnTx(&txUSART, w.u8[2]);
		PutByteOnTx(&txUSART, w.u8[3]);
		CalcAndPutCksOnTx(&txUSART);
	}

}
//---------------------CONTROL DE SUMA PONDERADA-------------------------
void analogToDigital(){
	for(int i=0; i<8; i++){
		if(analogValue_IR[i] >= umbralIR){
			digitalValue_IR[i] = 0;
			posicion_IR[i] = 0;
		}else{
			digitalValue_IR[i] = 1;
			posicion_IR[i] = 1;
		}
	}

	suma_ponderada = (70*digitalValue_IR[0] + 60*digitalValue_IR[1] +
					  50*digitalValue_IR[2] + 40*digitalValue_IR[3] +
					  30*digitalValue_IR[4] + 20*digitalValue_IR[5] +
					  10*digitalValue_IR[6] + 0*digitalValue_IR[7]);

	suma = (digitalValue_IR[0] + digitalValue_IR[1] +
			digitalValue_IR[2] + digitalValue_IR[3] +
			digitalValue_IR[4] + digitalValue_IR[5] +
			digitalValue_IR[6] + digitalValue_IR[7]);

	if(suma != 0){
		posicion = (suma_ponderada / suma);
	}

	posicion_IR[8] = posicion;
}

/* CONTROL CODE END PFP */

/* GRAFICAS SH1106 CODE BEGIN PFP */
/*
void graficar_oled(){
	uint8_t y = 0;
	sh1106_Fill(Black);
	char buf[10];
	static uint8_t oledMode =0;
	#ifdef SH1106_INCLUDE_FONT_7x10
	switch(oledMode){
		case DATACONEXION:
			//Text MARK II
			sh1106_SetCursor(40, y);
			sh1106_WriteString("MARK II", Font_7x10, White);
			y+=10;

			//IP
			sh1106_SetCursor(2, y);
			sh1106_WriteString("IP: ", Font_7x10, White);
			sh1106_WriteString(ESP01_GetLocalIP(), Font_7x10, White);
			y+=10;

			//PORT
			sh1106_SetCursor(2, y);
			sh1106_WriteString("PORT:", Font_7x10, White);
			sprintf(buf,"%lu",LocalPORT);
			sh1106_WriteString(buf, Font_7x10, White);
			y+=10;

			//STATUS WIFI
			sh1106_SetCursor(2, y);
			if(ESP01_StateWIFI() == ESP01_WIFI_CONNECTED){
				sh1106_WriteString("WIFI:CONNECTED", Font_7x10, White);
			}else{
				sh1106_WriteString("WIFI:DISCONNECTED", Font_7x10, White);
			}
			y+=10;

			//STATUS UDP
			sh1106_SetCursor(2, y);
			if(ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED){
				sh1106_WriteString("UDP:CONNECTED", Font_7x10, White);
			}else{
				sh1106_WriteString("UDP:DISCONNECTED", Font_7x10, White);
			}
			y+=10;
			//MODE
			sh1106_SetCursor(2, y);
			sh1106_WriteString("MODE:", Font_7x10, White);
			if(robotMode == IDLE){
				sh1106_WriteString("IDLE", Font_7x10, White);
			}else{
				sh1106_WriteString("FOLLOW LINE", Font_7x10, White);
			}

			break;
		case DATAACCEL:
			//Text MPU6050
			sh1106_SetCursor(40, y);
			sh1106_WriteString("MPU6050", Font_7x10, White);

			//acceleration values in 'g
			sprintf (buf, "%u", Ax);
			sh1106_SetCursor(2, 15);
			sh1106_WriteString("Ax:", Font_7x10, White);
			sh1106_WriteString(buf, Font_7x10, White);
			//sh1106_WriteString("g", Font_7x10, White);

			//gyroscope values in dps (degrees/s)
			sprintf (buf, "%u", Gx);
			sh1106_WriteString(" Gx:", Font_7x10, White);
			sh1106_WriteString(buf, Font_7x10, White);

			//acceleration values in 'g
			sprintf (buf, "%u", Ay);
			sh1106_SetCursor(2, 30);
			sh1106_WriteString("Ay:", Font_7x10, White);
			sh1106_WriteString(buf, Font_7x10, White);
			//sh1106_WriteString("g", Font_7x10, White);

			//gyroscope values in dps (degrees/s)
			sprintf (buf, "%u", Gy);
			sh1106_WriteString(" Gy:", Font_7x10, White);
			sh1106_WriteString(buf, Font_7x10, White);

			//acceleration values in 'g
			sprintf (buf, "%u", Az);
			sh1106_SetCursor(2, 45);
			sh1106_WriteString("Az:", Font_7x10, White);
			sh1106_WriteString(buf, Font_7x10, White);
				//sh1106_WriteString("g", Font_7x10, White);

			//gyroscope values in dps (degrees/s)
			sprintf (buf, "%u", Gz);
			sh1106_WriteString(" Gz:", Font_7x10, White);
			sh1106_WriteString(buf, Font_7x10, White);
			break;
		case DATARECORDVEL:
			//Text MARK II
			sh1106_SetCursor(40, y);
			sh1106_WriteString("MPU6050", Font_7x10, White);
			//ANGLEZ
			sprintf (buf, "%u", ANGLEZ);
			sh1106_SetCursor(2, 10);
			sh1106_WriteString("ANGLEZ: ", Font_7x10, White);
			sh1106_WriteString(buf, Font_7x10, White);

			//VELX
			sprintf (buf, "%u", VELX);
			sh1106_SetCursor(2, 20);
			sh1106_WriteString("VELX:", Font_7x10, White);
			sh1106_WriteString(buf, Font_7x10, White);

			//POSX
			sprintf (buf, "%u", POSX);
			sh1106_SetCursor(2, 30);
			sh1106_WriteString("POSX:", Font_7x10, White);
			sh1106_WriteString(buf, Font_7x10, White);
			break;
	}

	#endif

	//oledMode++;

	//if(oledMode >=3){
	//	oledMode = 0;
	//}

	sh1106_UpdateScreen();

}
*/
/* GRAFICAS SH1106 CODE END PFP */

/* CALCULO DE POSICION CODE BEGIN PFP */
void position_calculation(){

	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();

	ANGLEZ = anglez_last + Gz*TM;

	anglez_last = ANGLEZ;

	VELX = velx_last + Ax*TM;

	velx_last = VELX;

	POSX = posx_last + VELX*TM;

	posx_last = POSX;

}

/* CALCULO DE POSICION CODE END PFP */

/* UPDATE CODE BEGIN PFP */
void updateDataQT(){

	if(IS1S){
		IS1S = 0;
		PutHeaderOnTx(&txUSB, 0xA1, 25);
		MPU6050_Read_Accel();
		w.i32 = Ax;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		w.i32 = Ay;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		PutByteOnTx(&txUSB, w.u8[3]);
		w.i32 = Az;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		PutByteOnTx(&txUSB, w.u8[3]);

		MPU6050_Read_Gyro();
		w.i32 = Gx;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		PutByteOnTx(&txUSB, w.u8[3]);
		w.i32 = Gy;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		PutByteOnTx(&txUSB, w.u8[3]);
		w.i32 = Gz;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		PutByteOnTx(&txUSB, w.u8[3]);
		CalcAndPutCksOnTx(&txUSB);
	}

	if(IS100MS){
		IS100MS = 0;

		PutHeaderOnTx(&txUSB, 0xA6, 21);
		w.i32 = VELX;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		PutByteOnTx(&txUSB, w.u8[3]);
		w.i32 = POSX;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		PutByteOnTx(&txUSB, w.u8[3]);
		w.i32 = denominador;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		PutByteOnTx(&txUSB, w.u8[3]);
		w.i32 = e_t;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		PutByteOnTx(&txUSB, w.u8[3]);
		w.i32 = u_t_IZQ;
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		PutByteOnTx(&txUSB, w.u8[2]);
		PutByteOnTx(&txUSB, w.u8[3]);

		CalcAndPutCksOnTx(&txUSB);
	}
}
/* UPDATE CODE END PFP */


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

  /*Flags*/
	flag1.byte = 0;                             				//pongo todos los bits en 0
	flag2.byte = 0;

  //CONFIGURACION DE COMUNICACION
  /*Buffers de recepcion y transmision USB y USART*/

    rxUSB.buf = (uint8_t *)bufUSB_Rx;							//Puntero a bufUSB_Rx
  	rxUSB.ir = 0;
  	rxUSB.iw = 0;
  	rxUSB.maskSize = 0xFF;
  	rxUSB.header = 0;

  	rxUSART.buf = (uint8_t *)bufUSART_Rx;						//Puntero a busUSART_Rx
  	rxUSART.ir = 0;
  	rxUSART.iw = 0;
  	rxUSART.maskSize = 0xFF;
  	rxUSART.header = 0;

  	txAUX.buf = bufAUX_Tx;										//Puntero a bufTx
  	txAUX.ir = 0;
  	txAUX.iw = 0;
  	txAUX.maskSize = 0xFF;

  	txUSB.buf = bufUSB_Tx;										//Puntero a bufUSB_Tx
  	txUSB.ir = 0;
  	txUSB.iw = 0;
  	txUSB.maskSize = 0xFF;

  	txUSART.buf = bufUSART_Tx;									//Puntero a busUSART_Tx
  	txUSART.ir = 0;
  	txUSART.iw = 0;
  	txUSART.maskSize = 0xFF;

  	/*Envio de datos USB*/
  	bytes_to_send = 0;

  	/*Contador para ticker base de 1ms*/
  	time10ms = 40;
  	time5seg = 500;

  	//Configuracion requerida ESP01.h
  	ESP01Manager.aDoCHPD = CHENState;
  	ESP01Manager.aWriteUSARTByte = PutByteOnTxESP01;
  	ESP01Manager.bufRX = bufUSART_Rx;
  	ESP01Manager.iwRX = &txUSART.iw;
  	ESP01Manager.sizeBufferRX = 256;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);

  //COMUNICACION USB
  CDC_Attach_RxFun(OnUSBRx);
  DATAREADY = 0;
  //END COMUNICACION USB

  //CONFIGURACION UDP
  ESP01_Init(&ESP01Manager);
  ESP01_SetWIFI(SSID, PASSWORD);
  ESP01_StartUDP(RemoteIP, RemotePORT, LocalPORT);
  ESP01_AttachChangeState(GetESP01StateChange);
  ESP01_AttachDebugStr(GetESP01dbg);

  HAL_UART_Receive_IT(&huart1, &rxUSART.buf[rxUSART.iw], 1);			//Se habilita la interrupcion del USART1 (Recibo de a 1 byte)
  //END CONFIGURACION UDP

  HAL_TIM_Base_Start_IT (&htim2);										//Se habilita la interrupcion del timer2

  PWM_init();

  //CONFIGURACION DE MPU6050
  MPU6050_Init();

  //CONFIGURACION DE SH1106 OLED
  sh1106_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ESP01_Task();

	  if(IS10MS){
		  IS10MS = 0;
		  On10ms();
		  RobotWorkout();
	  }

	  if(IS5SEG){
		  IS5SEG = 0;
		  On5s();
	  }

	  if(rxUSB.ISCMD)                          					//Si se encontro un comando
		  DecodeCMD((_rx *)&rxUSB, (_tx *)&txUSB);

	  if(rxUSB.iw != rxUSB.ir)                   				//Mientras tenga datos en el buffer
		  DecodeHeader((_rx *)&rxUSB, (_tx *)&txUSB);

	  if(txUSB.iw != txUSB.ir)									//Si lo indices del tx son distintos es porque hay algo para enviar
		  TransmitUSB();

	  if(rxUSART.ISCMD)											//Si llego un comando por WIFI lo decodifico
		  DecodeCMD((_rx *)&rxUSART, (_tx *)&txUSART);

	  if(rxUSART.iw != rxUSART.ir)								//Si hay algo para decodificar en el USART
		  DecodeHeader((_rx *)&rxUSART, (_tx *)&txUSART);

	  if(txUSART.iw != txUSART.ir)								//Si hay algo para transmitir por el USART
		  TransmitUSART();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 18000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HEARBEAT_LED_GPIO_Port, HEARBEAT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AUX1_ESP_Pin|LED_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HEARBEAT_LED_Pin */
  GPIO_InitStruct.Pin = HEARBEAT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEARBEAT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AUX1_ESP_Pin LED_MODE_Pin */
  GPIO_InitStruct.Pin = AUX1_ESP_Pin|LED_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTOM_MODE_Pin */
  GPIO_InitStruct.Pin = BUTTOM_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTOM_MODE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
