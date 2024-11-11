#include "openmv.h"
#include "usart.h"

/*
1  2  3
R  G  B
红 绿 蓝
[e,321,132] 识别二维码
[y,0.5348,0.5076,1] 识别中心位置和颜色
[c,0.5348,0.5076,1] 高分辨率识别中心位置和颜色
[l，0.6500，-14] 直线距离和角度
*/
OPENMV_data openmv_data;
uint8_t openmv_rx_data[50] = {0};
uint8_t openmv_rx_flag = 0;
void openmv_uart_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&OPENMV_UART_HANDLE, openmv_rx_data, 49);
	__HAL_DMA_DISABLE_IT(&OPENMV_UART_DMA_HANDLE, DMA_IT_HT);
}

void openmv_uart_rx_callback(uint16_t Size)
{
	HAL_UART_Transmit(&huart1, openmv_rx_data, 50, HAL_MAX_DELAY);
	char buffer[100]; // 用于存储转换后的字符串
	strncpy(buffer, (char *)openmv_rx_data, sizeof(buffer) - 1);

	if (buffer[0] == '[')
	{
		char type = buffer[1]; // 获取数据类型标识符
		switch (type)
		{
		case 'e':
			if (sscanf(buffer, "[e,%d,%d]", &openmv_data.object_list[0], &openmv_data.object_list[1]) == 2)
			{
				printf("Object list: %d, %d\n", openmv_data.object_list[0], openmv_data.object_list[1]);
			}
			break;

		case 'y': // 识别圆中心位置
			if (sscanf(buffer, "[y,%f,%f,%d,%hu]", &openmv_data.object_position_x, &openmv_data.object_position_y, &openmv_data.identify_color, &openmv_data.edge_diastance) == 4)
			{
				openmv_data.hsa_circle = 1;
				if (openmv_data.identify_color == 1 || openmv_data.identify_color == 2 || openmv_data.identify_color == 3)
				{
					openmv_data.last_identify_color = openmv_data.identify_color;
				}
				// printf("Object position: %f, %f,%d\n", openmv_data.object_position_x, openmv_data.object_position_y, openmv_data.push_color);
			}
			break;

		case 'c': // 高分辨率识别圆中心位置
			if (sscanf(buffer, "[y,%f,%f,%d,%hu]", &openmv_data.object_position_x, &openmv_data.object_position_y, &openmv_data.identify_color, &openmv_data.edge_diastance) == 4)
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

// 获取三位数的第几位
int extract_digit(int number, int position)
{
	if (position < 1 || position > 3)
	{
		// 如果位置不在 1 到 3 之间，返回 -1 表示错误
		return -1;
	}

	switch (position)
	{
	case 1: // 个位
		return number % 10;
	case 2: // 十位
		return (number / 10) % 10;
	case 3: // 百位
		return number / 100;
	default:
		return -1; // 不应到达这里
	}
}

// 发送字符串
void Camera_SendString(const char *str)
{
	HAL_UART_Transmit(&OPENMV_UART_HANDLE, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
uint8_t Camera_now_mode = QR_MODE; // 当前模式
/*
切换模式
0：e 二维码识别模式
1：y 识别圆中心位置
2：c 颜色识别模式
3：l 直线距离和角度识别模式
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
		Camera_SendString("c"); // y
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

float Get_find_line_angle_avg(uint8_t times)
{
	float sum = 0;
	for (uint8_t i; i < times; i++)
	{
		sum += Get_find_line_angle();
		osDelay(1);
	}
	return sum / times;
}

float Get_find_line_distance(void)
{
	return openmv_data.line_distance;
}
