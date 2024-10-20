#include "openmv.h"
#include "usart.h"

/*
1  2  3
R  G  B
�� �� ��
[e,321,132] ʶ���ά��
[y,0.5348,0.5076,1] ʶ������λ�ú���ɫ
[c,0.5348,0.5076,1] �߷ֱ���ʶ������λ�ú���ɫ
[l��0.6500��-14] ֱ�߾���ͽǶ�
*/
OPENMV_data openmv_data;
uint8_t openmv_rx_data[50] = {0};
uint8_t openmv2_rx_data[50] = {0};
uint8_t openmv_rx_flag = 0;
void openmv_uart_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&OPENMV_UART_HANDLE, openmv_rx_data, 49);
	__HAL_DMA_DISABLE_IT(&OPENMV_UART_DMA_HANDLE, DMA_IT_HT);
}
void openmv2_uart_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&OPENMV2_UART_HANDLE, openmv2_rx_data, 49);
	__HAL_DMA_DISABLE_IT(&OPENMV2_UART_DMA_HANDLE, DMA_IT_HT);
}
void openmv_uart_rx_callback(uint16_t Size)
{
	// HAL_UART_Transmit(&huart1, openmv_rx_data, 50, HAL_MAX_DELAY);
	char buffer[100]; // ���ڴ洢ת������ַ���
	strncpy(buffer, (char *)openmv_rx_data, sizeof(buffer) - 1);

	if (buffer[0] == '[')
	{
		char type = buffer[1]; // ��ȡ�������ͱ�ʶ��
		switch (type)
		{
		case 'e':
			if (sscanf(buffer, "[e,%d,%d]", &openmv_data.object_list[0], &openmv_data.object_list[1]) == 2)
			{
				// printf("Object list: %d, %d\n", openmv_data.object_list[0], openmv_data.object_list[1]);
			}
			break;

		case 'y': // ʶ��Բ����λ��
			if (sscanf(buffer, "[y,%f,%f,%d]", &openmv_data.object_position_x, &openmv_data.object_position_y, &openmv_data.identify_color) == 3)
			{
				openmv_data.hsa_circle = 1;
				if (openmv_data.identify_color == 1 || openmv_data.identify_color == 2 || openmv_data.identify_color == 3)
				{
					openmv_data.last_identify_color = openmv_data.identify_color;
				}
				// printf("Object position: %f, %f,%d\n", openmv_data.object_position_x, openmv_data.object_position_y, openmv_data.push_color);
			}
			break;

		case 'c': // �߷ֱ���ʶ��Բ����λ��
			if (sscanf(buffer, "[c,%f,%f,%d]", &openmv_data.object_position_x, &openmv_data.object_position_y, &openmv_data.last_identify_color) == 3)
			{
				openmv_data.hsa_circle = 1;
				if (openmv_data.identify_color == 1 || openmv_data.identify_color == 2 || openmv_data.identify_color == 3)
				{
					openmv_data.last_identify_color = openmv_data.identify_color;
				}
				// printf("Object high position: %f, %f,%d\n", openmv_data.object_position_x, openmv_data.object_position_y, openmv_data.push_color);
			}
			break;

		case 'l':
			if (sscanf(buffer, "[l,%f,%f]", &openmv_data.line_distance, &openmv_data.line_angle) == 2)
			{
				// printf("Line distance: %f, Line angle: %f\n", openmv_data.line_distance, openmv_data.line_angle);
			}
			break;
		default:
			break;
		}
	}
	else if (buffer[0] == '0')
	{
		// printf("Received data: %s\n", buffer);
	}
	else if (buffer[0] == 'N' && buffer[1] == 'O' && buffer[2] == 'N' && buffer[3] == 'E')
	{
		openmv_data.hsa_circle = 0;
	}

	openmv_uart_init();
}
void openmv2_uart_rx_callback(uint16_t Size)
{
	// HAL_UART_Transmit(&huart1, openmv_rx_data, 50, HAL_MAX_DELAY);
	char buffer[100]; // ���ڴ洢ת������ַ���
	strncpy(buffer, (char *)openmv2_rx_data, sizeof(buffer) - 1);

	if (buffer[0] == '[')
	{
		char type = buffer[1]; // ��ȡ�������ͱ�ʶ��
		switch (type)
		{
		case 'e':
			if (sscanf(buffer, "[e,%d,%d]", &openmv_data.object_list[0], &openmv_data.object_list[1]) == 2)
			{
				printf("Object list: %d, %d\n", openmv_data.object_list[0], openmv_data.object_list[1]);
			}
			break;
		default:
			break;
		}
	}
	openmv2_uart_init();
}

// ��ȡ��λ���ĵڼ�λ
int extract_digit(int number, int position)
{
	if (position < 1 || position > 3)
	{
		// ���λ�ò��� 1 �� 3 ֮�䣬���� -1 ��ʾ����
		return -1;
	}

	switch (position)
	{
	case 1: // ��λ
		return number % 10;
	case 2: // ʮλ
		return (number / 10) % 10;
	case 3: // ��λ
		return number / 100;
	default:
		return -1; // ��Ӧ��������
	}
}

// �����ַ���
void Camera_SendString(const char *str)
{
	HAL_UART_Transmit(&OPENMV_UART_HANDLE, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
uint8_t Camera_now_mode = QR_MODE; // ��ǰģʽ
/*
�л�ģʽ
0��e ��ά��ʶ��ģʽ
1��y ʶ��Բ����λ��
2��c ��ɫʶ��ģʽ
3��l ֱ�߾���ͽǶ�ʶ��ģʽ
 */
void Camera_switch_mode(uint8_t mode)
{
	switch (mode)
	{
	case QR_MODE:
		Camera_SendString("e");
		Camera_now_mode = QR_MODE;
		break;
	case CENTER_POSITION_MODE:
		Camera_SendString("y");
		Camera_now_mode = CENTER_POSITION_MODE;
		break;
	case HIGH_CENTER_POSITION_MODE:
		Camera_SendString("c");
		Camera_now_mode = HIGH_CENTER_POSITION_MODE;
		break;
	case FIND_LINE_MODE:
		Camera_SendString("l");
		Camera_now_mode = FIND_LINE_MODE;
		break;
	default:
		break;
	}
}

float Get_find_line_angle(void)
{
	return openmv_data.line_angle;
}

float Get_find_line_angle_avg(void)
{
	float sum = 0;
	for (uint8_t i; i < 10; i++)
	{
		sum += Get_find_line_angle();
		osDelay(10);
	}
	return sum / 10;
}

float Get_find_line_distance(void)
{
	return openmv_data.line_distance;
}
