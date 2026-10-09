#include "stm32f10x.h"
#include "OLED.h"
#include "oledfont.h"

/* ============ 引脚层：软件 I2C（PB6=SCL，PB7=SDA） ============ */
/* 软件 I2C = 把 GPIO 当普通推挽输出，手动翻转时序，不用 I2C1 外设 */

#define OLED_SCL(x)  GPIO_WriteBit(GPIOB, GPIO_Pin_6, (BitAction)(x))
#define OLED_SDA(x)  GPIO_WriteBit(GPIOB, GPIO_Pin_7, (BitAction)(x))

/* SDA 方向切换：out=1 推挽输出，out=0 输入+内部上拉。
 * 只在"读应答位"那一拍切成输入 —— 从机拉低时不会被推挽输出顶着打架。 */
static void OLED_SDA_Mode(uint8_t out)
{
	GPIO_InitTypeDef gi;
	gi.GPIO_Pin   = GPIO_Pin_7;
	gi.GPIO_Speed = GPIO_Speed_50MHz;
	gi.GPIO_Mode  = out ? GPIO_Mode_Out_PP : GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &gi);
}

static void I2C_Delay(void)          /* 约 2us @72MHz，纯空转 */
{
	volatile uint8_t i = 40;
	while (i--);
}

static void I2C_Start(void)          /* 起始条件：SCL 高时 SDA 下降沿 */
{
	OLED_SDA(1);
	OLED_SCL(1);
	I2C_Delay();
	OLED_SDA(0);
	I2C_Delay();
	OLED_SCL(0);                     /* 钳住总线 */
	I2C_Delay();
}

static void I2C_Stop(void)           /* 停止条件：SCL 高时 SDA 上升沿 */
{
	OLED_SDA(0);
	I2C_Delay();
	OLED_SCL(1);
	I2C_Delay();
	OLED_SDA(1);
	I2C_Delay();
}
  
/* 发 1 字节：高位在前，发完显式补发第 9 个（应答）时钟。
 * ★ 2026-10-06 改：原来是靠后面的 Start/Stop 动作顺带蹭出一个边沿当应答位，
 *   实测这套写法点不亮屏（同一块屏、同样 PB6/PB7/0x78，测试程序能亮）。
 *   改成显式补第 9 个时钟 + 时钟放慢一倍，与当天验证通过的测试程序一致。 */
static void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_SDA((Byte & 0x80) ? 1 : 0);
		Byte <<= 1;
		I2C_Delay();
		OLED_SCL(1);                 /* SCL 高电平期间采样 SDA */
		I2C_Delay();
		OLED_SCL(0);
		I2C_Delay();
	}

	OLED_SDA_Mode(0);                /* 放开 SDA（内部上拉顶着），让从机应答 */
	I2C_Delay();
	OLED_SCL(1);                     /* 第 9 个时钟 = 应答位 */
	I2C_Delay();
	OLED_SCL(0);
	I2C_Delay();
	OLED_SDA_Mode(1);                /* 收回 SDA，恢复推挽 */
	OLED_SDA(1);
}

/* ============ SSD1306 命令/数据层 ============ */
/* 帧格式：[Start][0x78][控制字节][1字节内容][Stop]  控制字节 0x00=命令 0x40=数据 */

static void OLED_WriteCommand(uint8_t Command)
{
	I2C_Start();
	I2C_SendByte(0x78);              /* 从机地址（写） */
	I2C_SendByte(0x00);              /* 后面跟命令 */
	I2C_SendByte(Command);
	I2C_Stop();
}

static void OLED_WriteData(uint8_t Data)
{
	I2C_Start();
	I2C_SendByte(0x78);
	I2C_SendByte(0x40);              /* 后面跟显存数据 */
	I2C_SendByte(Data);
	I2C_Stop();
}

/* 页寻址模式：Y = 页 0~7（每页 8 行像素），X = 列 0~127 */
static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);                    /* 页地址 */
	OLED_WriteCommand(0x10 | ((X >> 4) & 0x0F));    /* 列地址高 4 位 */
	OLED_WriteCommand(0x00 | (X & 0x0F));           /* 列地址低 4 位 */
}

/* ============ 对外接口 ============ */

void OLED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* 软件 I2C：PB6/PB7 当普通推挽输出用（这两个脚兼是 I2C1 外设脚，但不开外设） */
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	OLED_SCL(1);
	OLED_SDA(1);                     /* I2C 空闲态：两线都拉高 */

	/* SSD1306 标准初始化序列（各命令含义见 SSD1306 手册） */
	OLED_WriteCommand(0xAE);         /* 关显示（初始化期间不闪） */
	OLED_WriteCommand(0xD5);         /* 时钟分频 / 振荡频率 */
	OLED_WriteCommand(0x80);
	OLED_WriteCommand(0xA8);         /* 复用率 = 64 行 */
	OLED_WriteCommand(0x3F);
	OLED_WriteCommand(0xD3);         /* 显示偏移 = 0 */
	OLED_WriteCommand(0x00);
	OLED_WriteCommand(0x40);         /* 显示起始行 = 0 */
	OLED_WriteCommand(0x8D);         /* 电荷泵使能（板子没外供 VPP，必须开） */
	OLED_WriteCommand(0x14);
	OLED_WriteCommand(0x20);         /* 寻址模式 = 页寻址 */
	OLED_WriteCommand(0x02);
	OLED_WriteCommand(0xA1);         /* 段重映射（列方向镜像） */
	OLED_WriteCommand(0xC8);         /* COM 扫描方向（行方向镜像） */
	OLED_WriteCommand(0xDA);         /* COM 引脚配置 */
	OLED_WriteCommand(0x12);
	OLED_WriteCommand(0x81);         /* 对比度 */
	OLED_WriteCommand(0xCF);
	OLED_WriteCommand(0xD9);         /* 预充电周期 */
	OLED_WriteCommand(0xF1);
	OLED_WriteCommand(0xDB);         /* VCOMH 电平 */
	OLED_WriteCommand(0x30);
	OLED_WriteCommand(0xA4);         /* 显示跟随 RAM 内容（非全亮） */
	OLED_WriteCommand(0xA6);         /* 正常显示（非反色） */
	OLED_WriteCommand(0xAF);         /* 开显示 */
}

void OLED_Clear(void)
{
	uint8_t page, col;
	for (page = 0; page < 8; page++)
	{
		OLED_SetCursor(page, 0);
		for (col = 0; col < 128; col++)
			OLED_WriteData(0x00);
	}
}

/* 一个字符 = 8x16 像素 = 占上下两页。Line 1~4，Column 1~16 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
	uint8_t page  = (Line - 1) * 2;
	uint8_t col   = (Column - 1) * 8;
	uint8_t index = (uint8_t)Char - 32;    /* 字库从空格(0x20)起 */
	uint8_t i;

	OLED_SetCursor(page, col);
	for (i = 0; i < 8; i++)
		OLED_WriteData(F8X16[index * 16 + i]);       /* 上半 8 字节 */

	OLED_SetCursor(page + 1, col);
	for (i = 0; i < 8; i++)
		OLED_WriteData(F8X16[index * 16 + 8 + i]);   /* 下半 8 字节 */
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
		OLED_ShowChar(Line, Column + i, String[i]);
}

/* 整数幂（避免用 math.h 的 double pow） */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
		Result *= X;
	return Result;
}

/* 显示无符号数：固定 Length 位，高位补 0（Length=3、Num=25 -> "025"） */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Num, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i,
			'0' + (Num / OLED_Pow(10, Length - 1 - i)) % 10);
	}
}
