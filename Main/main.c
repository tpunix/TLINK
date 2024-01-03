

#include "main.h"


/******************************************************************************/



void system_init(void)
{
	int hse_tmout = 0;

	// HSI On
	RCC->CTLR |= 0x00000001;
	RCC->CFGR0 = 0;
	// HSI On, HSE On, PLL Off
	RCC->CTLR = 0x00010001;
	// Wait HSE
	while((RCC->CTLR&0x00020000)==0){
		hse_tmout += 1;
	}
	if(hse_tmout>10000){
		// wait for IAP
		while(1){};
	}

	// USBDCLK=SYSCLK/3 AHB=SYSCLK APB1=AHB APB2=AHB ADC=APB2/8
#ifdef OSC_12M
	RCC->CFGR0 = 0x00a9c000;  // PLL: 12M*12 = 144M
#else
	RCC->CFGR0 = 0x009dc000;  // PLL: 16M*9  = 144M
#endif
	// PLL On
	RCC->CTLR |= 0x01000000;
	while((RCC->CTLR&0x02000000)==0);

	// PLLCLK as SYSCLK
	RCC->CFGR0 |= 0x02;
}

// 10M  2M  50M
//  1,  2,  3: 推拉输出
//  5,  6,  7: 开漏输出
//  9,  A,  B: 推拉功能输出
//  D,  E,  F: 开漏功能输出
//
// 0: 模拟输入
// 4: 浮空输入
// 8: 上下拉输入
//

void device_init(void)
{
	RCC->AHBPCENR  = 0x00000007; // SRAM DMA2 DMA1
	RCC->APB2PCENR = 0x0000007d; // GPIO AFIO
	RCC->APB1PCENR = 0x10044000; // PWR UART3 SPI2

	AFIO->PCFR1  = 0x04000000;   // SWD off
	AFIO->PCFR2  = 0x00000000;

	GPIOA->CFGLR = 0x44444424;   // PA1:o
	GPIOA->CFGHR = 0x44444442;   // PA8:o
	GPIOA->OUTDR = 0x00000000;

	GPIOB->CFGLR = 0x44444444;   //
	GPIOB->CFGHR = 0x44428944;   // PB10(TX) PB11(RX) PB12:o
	GPIOB->OUTDR = 0x00000000;

	GPIOC->CFGLR = 0x44444444;   //
	GPIOC->CFGHR = 0x44444444;   //
	GPIOC->OUTDR = 0x00000000;

	uart1_init(115200);
}


/******************************************************************************/


u64 get_mcycle(void)
{
	u32 h0, h1, low;

	do{
		h0  = read_csr(mcycleh);
		low = read_csr(mcycle);
		h1  = read_csr(mcycleh);
	}while(h0!=h1);

	return ((u64)h0<<32) | low;
}


/******************************************************************************/


void timer_init(void)
{
	reset_timer();
}


void reset_timer(void)
{
	SYSTICK->CTLR = 0x20;
	SYSTICK->CTLR = 0x01;
}


u32 get_timer(void)
{
	return SYSTICK->CNTL;
}


void udelay(int us)
{
	reset_timer();
	u32 end = get_timer() + us*(TIMER_HZ/1000000);
	while(get_timer()<end);
}


void mdelay(int ms)
{
	reset_timer();
	u32 end = get_timer() + ms*(TIMER_HZ/1000);
	while(get_timer()<end);
}


/******************************************************************************/

//#define DEBUGOUT

#define DUART UART3

void uart1_init(int baudrate)
{
	int div;

	div = (APB2CLK_FREQ + baudrate/2)/baudrate;

	DUART->CTLR1 = 0;
	DUART->CTLR2 = 0;
	DUART->CTLR3 = 0;
	DUART->BRR = div;
	DUART->CTLR1 = 0x200c;
}


int _getc(int tmout)
{
#ifdef DEBUGOUT
	while((DUART->STATR&0x0020)==0);
	return (DUART->DATAR)&0xff;
#else
	while(1);
	return 0;
#endif
}


void _putc(int ch)
{
#ifdef DEBUGOUT
	while((DUART->STATR&0x0080)==0);
	DUART->DATAR = ch;
#endif
}

void _puts(char *str)
{
	int ch;
	while((ch=*str++)){
		if(ch=='\n')
			_putc('\r');
		_putc(ch);
	}
}


/******************************************************************************/


void gpio_set(int group, int bit, int val)
{
	volatile GPIO_REGS *gpio = (GPIO_REGS*)(0x40010800+group*0x0400);
	int mask = 1<<bit;

	if(val)
		gpio->BSHR = mask;
	else
		gpio->BCR = mask;
}


int gpio_get(int group, int bit)
{
	volatile GPIO_REGS *gpio = (GPIO_REGS*)(0x40010800+group*0x0400);
	int mask = 1<<bit;

	return (gpio->INDR & mask) ? 1: 0;
}


void gpio_mode(int group, int bit, int mode, int ioval)
{
	int mreg;
	volatile GPIO_REGS *gpio = (GPIO_REGS*)(0x40010800+group*0x0400);

	if(ioval){
		gpio->OUTDR |= 1<<bit;
	}else{
		gpio->OUTDR &= ~(1<<bit);
	}

	if(bit<8){
		mreg = gpio->CFGLR;
		mreg &= ~(0x0f<<(4*bit));
		mreg |= mode<<(4*bit);
		gpio->CFGLR = mreg;
	}else{
		bit -= 8;
		mreg = gpio->CFGHR;
		mreg &= ~(0x0f<<(4*bit));
		mreg |= mode<<(4*bit);
		gpio->CFGHR = mreg;
	}
}

/******************************************************************************/

/******************************************************************************/


static char *excp_msg[16] = {
	"Inst Align",
	"Inst Fault",
	"Inst Illegal",
	"Breakpoint",
	"Load Align",
	"Load Fault",
	"Store Align",
	"Store Fault",
	"ECall_U",
	"ECall_S",
	"Reserved",
	"ECall_M",
	"IPage Fault",
	"LPage Fault",
	"Reserved",
	"SPage Fault",
};


// __attribute__((interrupt)) 会保存所有临时寄存器和浮点寄存器
// __attribute__((interrupt("WCH-Interrupt-fast"))) 只保存浮点寄存器
// 如果不考虑浮点寄存器, 可以不用这些属性. 但必须在start.S里面写入口，手动mret返回。

void HardFault_Handler(void)
{
    int cause = read_csr(mcause);
    printk("\n[%s!]: EPC=%08x TVAL=%08x CAUSE=%d\n", excp_msg[cause], read_csr(mepc), read_csr(mtval), cause);
    while(1){
    }
}


void int_enable(int id)
{
    PFIC->IENR[id/32] = 1<<(id&31);
}


void int_disable(int id)
{
    PFIC->IRER[id/32] = 1<<(id&31);
}

// 优先级: 0-7
void int_priority(int id, int p)
{
    PFIC->IPRIOR[id] = p<<5;
}


/******************************************************************************/


int main(void)
{
	system_init();
	device_init();

	printk("\n\nCH32V305 start!\n");
	printk(" mstatus: %08x\n", read_csr(mstatus));
	printk("   mtvec: %08x\n", read_csr(mtvec));
	printk("    mie: %08x\n", read_csr(mie));
	printk("    mip: %08x\n", read_csr(mip));

	void usb_dc_user_init(void);
	usb_dc_user_init();

	//simple_shell();
#if 1
	void dap_handle(void);
	while(1){
		//WFI(); // WFI会影响正在工作的外设.
		dap_handle();
	}
#else
	while(1){
		gpio_set(0, 1, 0);
		mdelay(500);
		gpio_set(0, 1, 1);
		mdelay(500);
	}
#endif
	return 0;
}



