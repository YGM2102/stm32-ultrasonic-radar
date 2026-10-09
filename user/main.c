/**
 * ============================================================================
 *  STM32F103C8T6 倒车雷达（自写版 · 标准库 3.5）
 * ----------------------------------------------------------------------------
 *  硬件连接（与 原理图_倒车雷达.html 网络编号一致）：
 *    HC-SR04 Trig -> PA6   推挽输出
 *    HC-SR04 Echo -> PA7   下拉输入（串 1K 限流进 PA7，TTa 口 5V 耐受）
 *    蜂鸣器模块   -> PB12  推挽输出（S8550 驱动，低电平响）
 *    按键         -> PB13  上拉输入 + EXTI13 下降沿（按键另一端直接接 GND，按下拉低）
 *    OLED SSD1306 -> PB6=SCL / PB7=SDA 软件 I2C（驱动见 Hardware/OLED.c）
 *
 *  相比 PDD 参考工程修掉的三个缺陷（面试素材）：
 *    1) 测距两段等待都加了 DWT 超时 —— 原版死等，模块没接直接卡死
 *    2) 按键内部上拉（IPU）+ 外部 1K 接地 —— 原版 NOPULL 悬浮误触发
 *    3) 报警用 DWT 毫秒计数做非阻塞翻转 —— 原版报警节拍拖慢主循环测距
 *
 *  阶段 2 升级口子：把 HCSR04_ReadCm() 里的 DWT 忙等量脉宽
 *  换成定时器输入捕获，对外接口（返回 cm）不变。
 * ============================================================================
 */
#include "stm32f10x.h"
#include "OLED.h"

/* ---------------- 引脚定义（改接线只动这里） ---------------- */
#define TRIG_PORT        GPIOA
#define TRIG_PIN         GPIO_Pin_6
#define ECHO_PORT        GPIOA
#define ECHO_PIN         GPIO_Pin_7
#define BEEP_PORT        GPIOB
#define BEEP_PIN         GPIO_Pin_12
#define KEY_PORT         GPIOB
#define KEY_PIN          GPIO_Pin_13
#define LED_PORT         GPIOC
#define LED_PIN          GPIO_Pin_13   /* 板载 LED，低电平亮：开机自检 + 报警跟随，不占面包板 */

#define TRIG_H()         GPIO_SetBits(TRIG_PORT, TRIG_PIN)
#define TRIG_L()         GPIO_ResetBits(TRIG_PORT, TRIG_PIN)
#define ECHO_READ()      GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN)
#define BEEP_ON()        GPIO_ResetBits(BEEP_PORT, BEEP_PIN)   /* 低电平响 */
#define BEEP_OFF()       GPIO_SetBits(BEEP_PORT, BEEP_PIN)     /* 高电平停 */

/* ---------------- DWT 内核周期计数器 ----------------
 * CYCCNT 每个内核时钟周期 +1，72MHz 下精度约 14ns，
 * 量脉宽 / 做微秒延时都比"循环数次数"准得多。
 * 寄存器地址直接用宏（DWT 是 Cortex-M3 内核部件，标准库没有封装）。 */
#define DEMCR            (*(__IO uint32_t *)0xE000EDFCUL)  /* 调试异常监视控制寄存器 */
#define DWT_CTRL         (*(__IO uint32_t *)0xE0001000UL)  /* DWT 控制寄存器 */
#define DWT_CYCCNT       (*(__IO uint32_t *)0xE0001004UL)  /* 周期计数器 */
#define DEMCR_TRCENA     (1UL << 24)   /* 跟踪使能，不开它 CYCCNT 不走 */
#define DWT_CYCCNTENA    (1UL << 0)    /* CYCCNT 计数使能 */
#define CPU_HZ           72000000UL    /* HSE 8M x PLL9 = 72M（system_stm32f10x.c 默认） */

/* ---------------- HC-SR04 参数 ---------------- */
#define HC_ECHO_WAIT_MS  30    /* 等 Echo 拉高的超时：模块没接/没触发时不能死等 */
#define HC_ECHO_HIGH_MS  24    /* Echo 高电平上限：最远 4m 往返约 23.5ms，超出当无效 */
#define HCSR04_ERR       (-1)  /* 本次测量无效的返回值 */

/* ---------------- 报警三档 ---------------- */
static const uint16_t alarm_cm[3] = {20, 50, 100};   /* mode 0/1/2 对应的报警距离 cm */

static volatile uint8_t key_flag = 0;   /* EXTI 置位、主循环消费（volatile：中断会改） */
static volatile uint16_t key_cnt = 0;   /* 【调试用】EXTI 中断触发次数，见 OLED 第 4 行 */

static uint8_t  mode = 2;               /* 上电从 100cm 档开始，演示效果最直观 */
static uint32_t last_key_ms = 0;        /* 上次有效按键时刻，用于消抖 */

/* ================= 基础函数 ================= */

/* 打开 DWT 周期计数器（调试部件，复位后默认关闭） */
static void DWT_Init(void)
{
    DEMCR    |= DEMCR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL  |= DWT_CYCCNTENA;
}

/* 忙等微秒延时：无符号减法比较，CYCCNT 约 59.6s 回绕一次也不会算错 */
void Delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT;
    uint32_t ticks = us * (CPU_HZ / 1000000UL);
    while ((DWT_CYCCNT - start) < ticks);
}

/* 毫秒延时（Delay_us 凑整，误差 1us 级，对本项目够用） */
void Delay_ms(uint32_t ms)
{
    while (ms--)
        Delay_us(1000);
}

/* 开机以来的毫秒数（配无符号减法用，别拿它直接比大小） */
static uint32_t Millis(void)
{
    return DWT_CYCCNT / (CPU_HZ / 1000UL);
}

/* ================= HC-SR04 测距 ================= */

/* 触发一次测距并量回波，返回厘米数；无回波/超时返回 HCSR04_ERR
 * 时序：Trig 给 >=10us 高 -> 模块发 8 个 40kHz 脉冲 -> Echo 高电平宽度 = 声波往返时间 */
static int16_t HCSR04_ReadCm(void)
{
    uint32_t t0, us;

    TRIG_H();
    Delay_us(15);               /* 手册要求 >=10us，留点余量 */
    TRIG_L();

    /* 第一段：等 Echo 拉高。超时 = 模块不在/没响应，绝不能死等 */
    t0 = DWT_CYCCNT;
    while (ECHO_READ() == 0)
    {
        if ((DWT_CYCCNT - t0) > (uint32_t)HC_ECHO_WAIT_MS * (CPU_HZ / 1000UL))
            return HCSR04_ERR;
    }

    /* 第二段：量 Echo 高电平宽度。t0 既是超时基准也是脉宽起点 */
    while (ECHO_READ() == 1)
    {
        if ((DWT_CYCCNT - t0) > (uint32_t)HC_ECHO_HIGH_MS * (CPU_HZ / 1000UL))
            return HCSR04_ERR;
    }

    us = (DWT_CYCCNT - t0) / (CPU_HZ / 1000000UL);   /* 高电平宽度，单位 us */
    /* 声速 340m/s = 0.034cm/us，往返除 2：距离 = us * 0.017，即 1cm 约 58.8us */
    return (int16_t)((us * 10) / 588);
}

/* ================= 报警与显示 ================= */

/* 非阻塞报警：分组鸣叫，越近越急
 *
 *   报警圈内（还没贴脸）-> 嘀嘀嘀   3 声短滴一组，一组之间停 gap_ms
 *   贴脸（<= 报警圈 1/4）-> 滴滴     2 声长滴一组，一组之间停得很短，听着是连着急鸣
 *
 * ★ 蜂鸣器是"有源"的（模块自带振荡），音调固定改不了，
 *   做节奏只能靠"响多久 / 隔多久"这两个时间量。
 *   全程非阻塞：用 DWT 毫秒计数比时间，主循环不被拖住。 */
static void Alarm_Update(int16_t dist)
{
    static uint32_t t_last  = 0;     /* 上一次动作（开或关）的时刻 */
    static uint8_t  beep_on = 0;
    static uint8_t  n_done  = 0;     /* 本组已经响完几声 */
    uint8_t  need;                   /* 一组要响几声 */
    uint32_t beep_ms, inner_ms, group_ms, wait_ms;

    /* 安全区 / 无回波：停声、收尾、熄灭板载灯 */
    if (dist == HCSR04_ERR || dist > alarm_cm[mode])
    {
        beep_on = 0;
        n_done  = 0;
        BEEP_OFF();
        GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
        return;
    }

    if (dist <= (int16_t)(alarm_cm[mode] / 4))
    {
        /* 贴脸：滴滴 —— 两声稍长的，组间几乎不歇 */
        need = 2;  beep_ms = 200;  inner_ms = 200;  group_ms = 400;
    }
    else
    {
        /* 报警圈：嘀嘀嘀 —— 照"请注意，倒车"那个标准提示音的节奏：
         *   每声 150ms、声间 150ms、三声一组，组间停 1 秒出头再重复。
         *   组间停顿随距离略缩短（100cm 约 1.2s，25cm 约 0.8s），所以靠近会更催。 */
        need = 3;  beep_ms = 150;  inner_ms = 150;  group_ms = 700 + (uint32_t)dist * 5;
    }

    if (beep_on)                  wait_ms = beep_ms;                    /* 响够了就停 */
    else if (n_done < need)       wait_ms = inner_ms;                   /* 组内间隔 */
    else                          wait_ms = group_ms;                   /* 组间间隔 */

    if (Millis() - t_last >= wait_ms)
    {
        t_last = Millis();
        if (beep_on)
        {
            beep_on = 0;
            BEEP_OFF();
        }
        else
        {
            if (n_done >= need)       /* 刚歇完一组，开新组 */
                n_done = 0;
            beep_on = 1;
            BEEP_ON();
            n_done++;
        }
    }

    /* 板载 LED 跟随报警节拍：面包板外设没电/没接时，看这颗灯就知道固件活着、测到东西没 */
    GPIO_WriteBit(LED_PORT, LED_PIN, beep_on ? Bit_RESET : Bit_SET);
}

/* 刷新 OLED：固定宽度字段覆盖写，不清全屏，无闪烁（SSD1306 直接写显存） */
static void Show_Info(int16_t dist)
{
    uint8_t danger = (dist != HCSR04_ERR && dist <= alarm_cm[mode]);

    OLED_ShowString(1, 1, "Dist:");
    if (dist == HCSR04_ERR)
        OLED_ShowString(1, 6, " -- cm");
    else
    {
        OLED_ShowNum(1, 6, (uint32_t)dist, 3);
        OLED_ShowString(1, 9, " cm");
    }

    OLED_ShowString(2, 1, "Alarm:");
    OLED_ShowNum(2, 7, alarm_cm[mode], 3);
    OLED_ShowString(2, 10, " cm");

    OLED_ShowString(3, 1, "Mode:");
    OLED_ShowNum(3, 6, mode + 1, 1);          /* 显示成 1/2/3 档 */
    OLED_ShowString(3, 9, danger ? "DANGER" : "SAFE  ");

    /* 【调试用】第 4 行：PB13 当前电平 + EXTI 中断次数。
     *   按下按钮时 KEY 应该从 low 变 HIGH；变了但 Mode 不变 -> 软件问题；
     *   一直不变 -> 按键/接线问题。查完把这一段删掉即可。 */
    OLED_ShowString(4, 1, "KEY:");
    OLED_ShowString(4, 5, GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) ? "HIGH " : "low  ");
    OLED_ShowString(4, 10, "n=");
    OLED_ShowNum(4, 12, key_cnt % 1000, 3);
}

/* ================= 主流程 ================= */

int main(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    int16_t dist;
    uint8_t i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC, ENABLE);

    /* 先把 PB12 的输出寄存器置 1 再配成输出：
     * GPIO_Init 生效瞬间不会意外输出低电平，蜂鸣器上电不"嘀" */
    GPIO_SetBits(BEEP_PORT, BEEP_PIN);

    /* TRIG（PA6）推挽输出，默认低 */
    GPIO_InitStruct.GPIO_Pin   = TRIG_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRIG_PORT, &GPIO_InitStruct);

    /* BEEP（PB12）推挽输出（低响高停） */
    GPIO_InitStruct.GPIO_Pin = BEEP_PIN;
    GPIO_Init(BEEP_PORT, &GPIO_InitStruct);

    /* 板载 LED（PC13）：先置高（灭）再配输出 */
    GPIO_SetBits(LED_PORT, LED_PIN);
    GPIO_InitStruct.GPIO_Pin = LED_PIN;
    GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* ECHO（PA7）下拉输入：模块没插时读到 0 而不是悬空乱跳 */
    GPIO_InitStruct.GPIO_Pin  = ECHO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(ECHO_PORT, &GPIO_InitStruct);

    /* KEY（PB13）上拉输入：按键另一端直接接 GND，按下 = 被拉低。
     * ★ 2026-10-06 改：原来配的是 IPD + 上升沿（要求按键接 3.3V），
     *   但硬件是"按键接地"——接地才是键盘/按键的经典接法
     *   （上拉 + 按键接地抗干扰更好），所以把极性反过来配。
     *   实测：按键与地之间不要再串电阻，内部 40k 上拉已经够用。 */
    GPIO_InitStruct.GPIO_Pin  = KEY_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(KEY_PORT, &GPIO_InitStruct);

    /* EXTI13 <- PB13，下降沿触发（按下 = 被拉低） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    EXTI_InitStruct.EXTI_Line    = EXTI_Line13;
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel                   = EXTI15_10_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    DWT_Init();

    /* 开机自检：板载 LED 闪 5 下 = 复位后固件确实在跑（不依赖面包板任何外设）。
     * 不闪：先查 BOOT0 跳线是否在 0 位，再怀疑固件/供电 */
    for (i = 0; i < 5; i++)
    {
        GPIO_ResetBits(LED_PORT, LED_PIN);   /* 亮 */
        Delay_ms(150);
        GPIO_SetBits(LED_PORT, LED_PIN);     /* 灭 */
        Delay_ms(150);
    }

    /* 开机画面（打串口终端的字符串才强制 ASCII，OLED 上本来也用英文） */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(2, 3, "Parking Radar");
    Delay_ms(800);
    OLED_Clear();

    while (1)
    {
        /* 按键在中断里只置标志，这里消费：200ms 内的重复沿都当抖动丢掉 */
        if (key_flag)
        {
            key_flag = 0;
            if (Millis() - last_key_ms > 200)
            {
                last_key_ms = Millis();
                mode = (mode < 2) ? (mode + 1) : 0;
            }
        }

        dist = HCSR04_ReadCm();     /* 测距 -> 显示 -> 报警，一圈约 60ms+ */
        Show_Info(dist);
        Alarm_Update(dist);

        Delay_ms(60);               /* HC-SR04 两次触发间隔建议 >=60ms，防余波串扰 */
    }
}

/* ================= 中断服务函数 ================= */

/* PB13 按键：EXTI 线 10~15 共用这一个中断号
 * 原则：中断里只置标志，去抖和切档放主循环做（快进快出） */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line13) != RESET)
    {
        key_flag = 1;
        key_cnt++;                       /* 【调试用】仅用于 OLED 第 4 行显示 */
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}
