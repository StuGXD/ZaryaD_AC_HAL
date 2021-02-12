#include "main.h"

#define Block_Id 0x00FEF70C //Testing Block ID

CAN_HandleTypeDef hcan;

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

static void MX_CAN_Init(CAN_HandleTypeDef *hcan)
{
	hcan->Instance = CAN; //Speed 125kb\s
	hcan->Init.Prescaler = 24;
	hcan->Init.Mode = CAN_MODE_NORMAL;
	hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan->Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan->Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan->Init.TimeTriggeredMode = DISABLE;
	hcan->Init.AutoBusOff = ENABLE;
	hcan->Init.AutoWakeUp = ENABLE;
	hcan->Init.AutoRetransmission = DISABLE;
	hcan->Init.ReceiveFifoLocked = DISABLE;
	hcan->Init.TransmitFifoPriority = ENABLE;
	if (HAL_CAN_Init(hcan) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
}

static void CAN_Filter_Init(CAN_HandleTypeDef *hcan) //Filter disable for all packages can transmit in bus
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
	HAL_CAN_ConfigFilter(hcan, &CAN_Filter);
}

static void CAN_Start(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_Start(hcan);
	HAL_Delay(50); //Delay for CAN start
	HAL_CAN_ActivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING);
}

static void Send_CAN(CAN_HandleTypeDef *hcan, uint32_t ext_id, uint32_t data_type, uint32_t data_len, uint8_t *data)
{
	uint32_t				TxMailbox = 0;
	CAN_TxHeaderTypeDef		TxHeader = {0};

	TxHeader.ExtId = ext_id;
	TxHeader.RTR = data_type;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = data_len;
	TxHeader.TransmitGlobalTime = DISABLE;

	HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);
}

static void Create_TxData(uint8_t *data, uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7)
{
	data[0] = byte0;
	data[1] = byte1;
	data[2] = byte2;
	data[3] = byte3;
	data[4] = byte4;
	data[5] = byte5;
	data[6] = byte6;
	data[7] = byte7;
}

void CAN_Transmite_manual(CAN_HandleTypeDef *hcan, uint8_t *data, uint16_t recieved_id, uint8_t recieved_dlc, uint8_t *recieved_data)
{
	uint32_t				TxMailbox = 0;
	CAN_TxHeaderTypeDef		TxHeader = {0};

	TxHeader.ExtId = recieved_id;
	TxHeader.DLC = recieved_dlc;
	data[0] = recieved_data[0];
	data[1] = recieved_data[1];
	data[2] = recieved_data[2];
	data[3] = recieved_data[3];
	data[4] = recieved_data[4];
	data[5] = recieved_data[5];
	data[6] = recieved_data[6];
	data[7] = recieved_data[7];

	HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t				TxData[8] = {0};
	uint8_t				RxData[8] = {0};
	CAN_RxHeaderTypeDef	RxHeader = {0};

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	HAL_Delay(500);
	CAN_Transmite_manual(hcan, TxData, RxHeader.ExtId, RxHeader.DLC, RxData);
}

void Error_Handler(void)
{
	__disable_irq();
	while (1) {}
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_CAN_Init(&hcan);

	CAN_Filter_Init(&hcan);
	CAN_Start(&hcan);

	while (1)
	{
		uint8_t TxData[8] = {0};

		Create_TxData(TxData,0xAA,0x12,0xCC,0xDD,0x35,0xFF,0x00,0x01);
		Send_CAN(&hcan, Block_Id,CAN_RTR_DATA,8,TxData);

		HAL_Delay(1000);
	}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
