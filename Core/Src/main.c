#include "main.h"

CAN_HandleTypeDef     hcan;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;

#define Block_Id 0x00FEF70C //Testing Block ID

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void CAN_Filter_Init(void); //custom filter configuration
static void CAN_Start(void); //custom start can & notification
static void Create_TxData(uint8_t *TxData, uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7); //custom create send data array
static void Send_CAN(uint32_t ExtID, uint32_t DATA_type, uint32_t Dlen, uint8_t *TxData); //custom can send function
static void CAN_Transmite_manual(uint8_t *TxData, uint16_t ID_CAN, uint8_t DLC_CAN, uint8_t *DATA_CAN); //function for retransmission CAN package

uint8_t TxData[8] = {0};

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN_Init();

  CAN_Filter_Init();
  CAN_Start();

  while (1)
  {
	uint8_t TxData[8] = {0};

    Create_TxData(TxData,0xAA,0x12,0xCC,0xDD,0x35,0xFF,0x00,0x01);
	Send_CAN(Block_Id,CAN_RTR_DATA,8,TxData);

	HAL_Delay(1000);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_CAN_Init(void)
{
  hcan.Instance = CAN; //Speed 125kb\s
  hcan.Init.Prescaler = 24;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
}

static void CAN_Filter_Init(void) //Filter disable for all packages can transmit in bus
{
	CAN_FilterTypeDef	CAN_Filter;

	CAN_Filter.FilterBank = 0;
	CAN_Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN_Filter.FilterMode = CAN_FILTERSCALE_32BIT;
	CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_Filter.FilterIdHigh = 0x0000;
	CAN_Filter.FilterIdLow = 0x0000;
	CAN_Filter.FilterMaskIdHigh = 0x0000;
	CAN_Filter.FilterMaskIdLow = 0x0000;
	CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &CAN_Filter);
}

static void CAN_Start(void)
{
	HAL_CAN_Start(&hcan);
	HAL_Delay(50); //Delay for CAN start
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING);
}

static void Send_CAN(uint32_t ExtID, uint32_t DATA_type, uint32_t Dlen, uint8_t *TxData)
{
	uint32_t TxMailbox = 0;

	TxHeader.ExtId = ExtID;
	TxHeader.RTR = DATA_type;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = Dlen;
	TxHeader.TransmitGlobalTime = DISABLE;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

static void Create_TxData(uint8_t *TxData, uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7)
{
	TxData[0] = byte0;
	TxData[1] = byte1;
	TxData[2] = byte2;
	TxData[3] = byte3;
	TxData[4] = byte4;
	TxData[5] = byte5;
	TxData[6] = byte6;
	TxData[7] = byte7;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t RxData[8] = {0};

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	HAL_Delay(500);
	CAN_Transmite_manual(TxData, RxHeader.ExtId,RxHeader.DLC,RxData);
}

void CAN_Transmite_manual(uint8_t *TxData, uint16_t ID_CAN, uint8_t DLC_CAN, uint8_t *DATA_CAN)
{
	uint32_t TxMailbox = 0;

	TxHeader.ExtId = ID_CAN;
	TxHeader.DLC = DLC_CAN;
	TxData[0] = DATA_CAN[0];
	TxData[1] = DATA_CAN[1];
	TxData[2] = DATA_CAN[2];
	TxData[3] = DATA_CAN[3];
	TxData[4] = DATA_CAN[4];
	TxData[5] = DATA_CAN[5];
	TxData[6] = DATA_CAN[6];
	TxData[7] = DATA_CAN[7];
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
