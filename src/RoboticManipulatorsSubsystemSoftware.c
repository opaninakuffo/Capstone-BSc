#include <MKL25Z4.H>
#include <stdio.h>
#include "queues.h"
#include <math.h>

#define MASK(x)					(1UL << x)
#define R_gripper_PWM_PIN		(1)
#define R_wrist_PWM_PIN			(0)
#define R_elbow_PWM_PIN			(2)
#define R_shoulder_PWM_PIN		(3)
#define L_gripper_PWM_PIN		(2)
#define L_wrist_PWM_PIN			(3)
#define L_elbow_PWM_PIN			(0)
#define L_shoulder_PWM_PIN		(1)
#define OB_RED 					(18)
#define L_RX					(1)		//PTA1
#define L_TX					(2)		//PTA2
#define R_RX					(3)		//PTC3
#define R_TX					(4)		//PTC4
#define OSR 					(15)		//over sample rate (like a pre-scaler)
#define BAUD_RATE  				(9600)	//my communication rate on BT
#define SYS_CLOCK				(20971520u)
#define BUS_CLOCK				(10485760u) // half of sys_clock


volatile char rxChar;
enum parts{Lgripper, Lwrist, Lelbow, Lshoulder ,Rgripper, Rwrist, Relbow, Rshoulder};
static enum parts L_next_part = Lgripper;
static enum parts R_next_part = Rgripper;
Q_T L_RxQ, R_RxQ;
void init_pins();
void init_TPM_timer();
void init_UART();
void doLeftRx_task();
void doRightRx_task();
void handle_L_input(uint8_t c);
void handle_R_input(uint8_t c);
void R_gripper_input(uint8_t c);
void R_wrist_input(uint8_t c);
void R_elbow_input(uint8_t c);
void R_shoulder_input(uint8_t c);
void L_gripper_input(uint8_t c);
void L_wrist_input(uint8_t c);
void L_elbow_input(uint8_t c);
void L_shoulder_input(uint8_t c);


int main() {

	Q_Init(&L_RxQ);
	Q_Init(&R_RxQ);
	init_pins();
	init_UART();
	init_TPM_timer();
	while(1) {
		doLeftRx_task();
		doRightRx_task();
	}
}

void init_TPM_timer() {
	//Clock gate
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;	//*******TPM2 channel 0
	//Select clock source in SIM_SOPT
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);	//1- MCGPLLCLK/2, or MCGFLLCLK 01 (ext?), 2-ext OSCERCLK, 3-internal MCGIRCLK
	//Configure registers
	TPM2->MOD = 0xCCCC;  //20ms or 50Hz period
	TPM1->MOD = 0xCCCC;  //20ms or 50Hz period
	TPM0->MOD = 0xCCCC;  //20ms or 50Hz period

	//working wth TPM2_C0SC (L_Gripper)
	//output compare + edge aligned PWM MSBA: 10, ELSBA:10
	TPM0->CONTROLS[0].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM0->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;  //clear spurious interrupts
	TPM0->CONTROLS[0].CnV =0xF5C;  //1.5ms (neutral position) - gripper

	//working wth TPM2_C1SC (L_Wrist)
	//output compare + edge aligned PWM MSBA: 10, ELSBA:10
	TPM0->CONTROLS[2].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM0->CONTROLS[2].CnSC |= TPM_CnSC_CHF_MASK;  //clear spurious interrupts
	TPM0->CONTROLS[2].CnV =0xF5C;  //1.5ms (neutral position) - wrist

	//working wth TPM1_C0SC (L_Elbow)
	//output compare + edge aligned PWM MSBA: 10, ELSBA:10
	TPM0->CONTROLS[3].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM0->CONTROLS[3].CnSC |= TPM_CnSC_CHF_MASK;  //clear spurious interrupts
	TPM0->CONTROLS[3].CnV =0xA3E;  //1ms (upward position) - elbow

	//working wth TPM1_C0SC (L_Shoulder)
	//output compare + edge aligned PWM MSBA: 10, ELSBA:10
	TPM0->CONTROLS[1].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM0->CONTROLS[1].CnSC |= TPM_CnSC_CHF_MASK;  //clear spurious interrupts
	TPM0->CONTROLS[1].CnV =0xF5C;  //1.5ms (upward position) - shoulder

	//working wth TPM2_C0SC (R_Gripper)
	//output compare + edge aligned PWM MSBA: 10, ELSBA:10
	TPM2->CONTROLS[0].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM2->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;  //clear spurious interrupts
	TPM2->CONTROLS[0].CnV =0xF5C;  //1.5ms (neutral position) - gripper

	//working wth TPM2_C1SC (R_Wrist)
	//output compare + edge aligned PWM MSBA: 10, ELSBA:10
	TPM2->CONTROLS[1].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM2->CONTROLS[1].CnSC |= TPM_CnSC_CHF_MASK;  //clear spurious interrupts
	TPM2->CONTROLS[1].CnV =0xF5C;  //1.5ms (neutral position) - wrist

	//working wth TPM1_C0SC (R_Elbow)
	//output compare + edge aligned PWM MSBA: 10, ELSBA:10
	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;  //clear spurious interrupts
	TPM1->CONTROLS[0].CnV =0xA3E;  //1ms (upward position) - elbow

	//working wth TPM1_C0SC (R_Shoulder)
	//output compare + edge aligned PWM MSBA: 10, ELSBA:10
	TPM1->CONTROLS[1].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM1->CONTROLS[1].CnSC |= TPM_CnSC_CHF_MASK;  //clear spurious interrupts
	TPM1->CONTROLS[1].CnV =0xF5C;  //1.5ms (upward position) - shoulder

	TPM0->SC |= TPM_SC_TOF_MASK | TPM_SC_PS(3) | TPM_SC_TOIE_MASK;
	TPM0->SC |= TPM_SC_CMOD(1); //enable internal clock to run

	TPM2->SC |= TPM_SC_TOF_MASK | TPM_SC_PS(3) | TPM_SC_TOIE_MASK;
	TPM2->SC |= TPM_SC_CMOD(1); //enable internal clock to run

	TPM1->SC |= TPM_SC_TOF_MASK | TPM_SC_PS(3) | TPM_SC_TOIE_MASK;
	TPM1->SC |= TPM_SC_CMOD(1); //enable internal clock to run

	NVIC_ClearPendingIRQ(TPM2_IRQn);
	NVIC_SetPriority(TPM2_IRQn, 3);
	NVIC_EnableIRQ(TPM2_IRQn);

	NVIC_ClearPendingIRQ(TPM1_IRQn);
	NVIC_SetPriority(TPM1_IRQn, 3);
	NVIC_EnableIRQ(TPM1_IRQn);

	NVIC_ClearPendingIRQ(TPM0_IRQn);
	NVIC_SetPriority(TPM0_IRQn, 3);
	NVIC_EnableIRQ(TPM0_IRQn);
}

void init_pins() {
	//clock gating necessary ports
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

	//configure UART
	PORTA->PCR[L_RX] &= ~PORT_PCR_MUX_MASK;  	//clear mux
	PORTA->PCR[L_RX] |=  PORT_PCR_MUX(2); 	//set for UART0 RX
	PORTA->PCR[L_TX] &= ~PORT_PCR_MUX_MASK;	//clear
	PORTA->PCR[L_TX] |=  PORT_PCR_MUX(2); 	//set for UART0 TX

	PORTC->PCR[R_RX] &= ~PORT_PCR_MUX_MASK;  	//clear mux
	PORTC->PCR[R_RX] |=  PORT_PCR_MUX(3); 	//set for UART1 RX
	PORTC->PCR[R_TX] &= ~PORT_PCR_MUX_MASK;	//clear
	PORTC->PCR[R_TX] |=  PORT_PCR_MUX(3); 	//set for UART1 TX

	//configure TPM/pwm  output
	PORTB->PCR[R_gripper_PWM_PIN] &= ~PORT_PCR_MUX_MASK;	//Clear mux
	PORTB->PCR[R_gripper_PWM_PIN] |= PORT_PCR_MUX(3);	//***setup to be output of TPM2_CH0****
	PORTB->PCR[R_wrist_PWM_PIN] &= ~PORT_PCR_MUX_MASK;	//Clear mux
	PORTB->PCR[R_wrist_PWM_PIN] |= PORT_PCR_MUX(3);	//***setup to be output of TPM2_CH1****
	PORTB->PCR[R_elbow_PWM_PIN] &= ~PORT_PCR_MUX_MASK;	//Clear mux
	PORTB->PCR[R_elbow_PWM_PIN] |= PORT_PCR_MUX(3);	//***setup to be output of TPM1_CH0****
	PORTB->PCR[R_shoulder_PWM_PIN] &= ~PORT_PCR_MUX_MASK;	//Clear mux
	PORTB->PCR[R_shoulder_PWM_PIN] |= PORT_PCR_MUX(3);	//***setup to be output of TPM1_CH0****

	PORTD->PCR[L_gripper_PWM_PIN] &= ~PORT_PCR_MUX_MASK;	//Clear mux
	PORTD->PCR[L_gripper_PWM_PIN] |= PORT_PCR_MUX(4);	//***setup to be output of TPM0_CH0****
	PORTD->PCR[L_wrist_PWM_PIN] &= ~PORT_PCR_MUX_MASK;	//Clear mux
	PORTD->PCR[L_wrist_PWM_PIN] |= PORT_PCR_MUX(4);	//***setup to be output of TPM0_CH2****
	PORTD->PCR[L_elbow_PWM_PIN] &= ~PORT_PCR_MUX_MASK;	//Clear mux
	PORTD->PCR[L_elbow_PWM_PIN] |= PORT_PCR_MUX(4);	//***setup to be output of TPM0_CH3****
	PORTD->PCR[L_shoulder_PWM_PIN] &= ~PORT_PCR_MUX_MASK;	//Clear mux
	PORTD->PCR[L_shoulder_PWM_PIN] |= PORT_PCR_MUX(4);	//***setup to be output of TPM0_CH1****

	//configure LEDS
	PORTB->PCR[OB_RED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[OB_RED] |= PORT_PCR_MUX(1);
	PTB->PDDR |= MASK(OB_RED);
	PTB->PSOR = MASK(OB_RED);
}

void init_UART() {
	//select clock for uart0 (disabled by default), MCGFLLCLK/system clk as UART0 clock
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);
	// clock gate UART0
	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK | SIM_SCGC4_UART1_MASK;		//clock gate UART0
	//compute set baud rate (SBR), choosing baud rate of 9600 for BT
	uint8_t sbr0 = (uint16_t)((SYS_CLOCK)/((OSR+1) *BAUD_RATE ));	//default OSR is 15, 	sbr0=136.5 if SYS_CLOCK =20971520u
	//UART0->BDH |=((sbr>>8) & 0x1F);	//generic. set only bottom 5 bits
	UART0->BDH =0;			//0x0 for this calculation, other fields are default 0.
	UART0->BDL=sbr0;			//0x88 for this calculation
	// Rx Interrupt enabled, Tx & RX enable
	UART0->C2  |= UART_C2_RIE_MASK | UART_C2_TE_MASK | UART_C2_RE_MASK;
	//note: default is 8N1 if uart0->C1=0

	uint8_t sbr1 = (uint16_t)((BUS_CLOCK)/(16 *BAUD_RATE ));	//default OSR is 15, 	sbr1=68.3 if BUS_CLOCK =20971520u/2

	UART1->BDH =0;			//0x0 for this calculation, other fields are default 0.
	UART1->BDL=sbr1;			//0x88 for this calculation
	// Rx Interrupt enabled, Tx & RX enable
	UART1->C2  |= UART_C2_RIE_MASK | UART_C2_TE_MASK | UART_C2_RE_MASK;

	NVIC_SetPriority(UART0_IRQn, 3);
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);

	NVIC_SetPriority(UART1_IRQn, 3);
	NVIC_ClearPendingIRQ(UART1_IRQn);
	NVIC_EnableIRQ(UART1_IRQn);
}

void TPM2_IRQHandler(){
	TPM2->SC |= TPM_SC_TOF_MASK ; //clear the interrupt
}

void TPM1_IRQHandler(){
	TPM1->SC |= TPM_SC_TOF_MASK ; //clear the interrupt
}

void TPM0_IRQHandler(){
	TPM0->SC |= TPM_SC_TOF_MASK ; //clear the interrupt
}


void handle_R_input(uint8_t c) {
	switch(R_next_part) {
	case Rgripper:
//		printf("RG by %d\n", c);
		R_gripper_input(c);
		break;
	case Rwrist:
//		printf("RW by %d\n", c);
		R_wrist_input(c);
		break;
	case Relbow:
//		printf("RE by %d\n", c);
		R_elbow_input(c);
		break;
	case Rshoulder:
//		printf("RS by %d\n", c);
		R_shoulder_input(c);
		break;
	default:
		break;
	}
}

void handle_L_input(uint8_t c) {
	switch(L_next_part) {
	case Lgripper:
//		printf("LG by %d\n", c);
		L_gripper_input(c);
		break;
	case Lwrist:
//		printf("LW by %d\n", c);
		L_wrist_input(c);
		break;
	case Lelbow:
//		printf("LE by %d\n", c);
		L_elbow_input(c);
		break;
	case Lshoulder:
//		printf("LS by %d\n", c);
		L_shoulder_input(c);
		break;
	default:
		break;
	}
}

void R_gripper_input(uint8_t c){
	int PWM_val = c*(5242-2622)/180+2622;
	TPM2->CONTROLS[0].CnV = PWM_val;
}

void R_wrist_input(uint8_t c){
	int PWM_val = c*(5242-2622)/180+2622;
	TPM2->CONTROLS[1].CnV = PWM_val;
}

void R_elbow_input(uint8_t c){
	int PWM_val = c*(5242-2622)/180+2622;
	TPM1->CONTROLS[0].CnV = PWM_val;
}

void R_shoulder_input(uint8_t c){
	int PWM_val = c*(5242-2622)/180+2622;
	TPM1->CONTROLS[1].CnV = PWM_val;
}

void L_gripper_input(uint8_t c){
	int PWM_val = c*(5242-2622)/180+2622;
	TPM0->CONTROLS[0].CnV = PWM_val;
}

void L_wrist_input(uint8_t c){
	int PWM_val = c*(5242-2622)/180+2622;
	TPM0->CONTROLS[2].CnV = PWM_val;
}

void L_elbow_input(uint8_t c){
	int PWM_val = c*(5242-2622)/180+2622;
	TPM0->CONTROLS[3].CnV = PWM_val;
}

void L_shoulder_input(uint8_t c){
	int PWM_val = c*(5242-2622)/180+2622;
	TPM0->CONTROLS[1].CnV = PWM_val;
}

void UART0_IRQHandler(void) {
	uint8_t ch;

	if (UART0->S1 & (UART_S1_OR_MASK |UART_S1_NF_MASK |
		UART_S1_FE_MASK | UART_S1_PF_MASK)) {
			// clear the error flags
			UART0->S1 |= UART_S1_OR_MASK | UART_S1_NF_MASK |
									UART_S1_FE_MASK | UART_S1_PF_MASK;
			// read the data register to clear RDRF
			ch = UART0->D;
	}
	if (UART0->S1 & UART_S1_RDRF_MASK) {
//		PTB->PTOR = MASK(OB_RED);
		// received a character. handle it in the handle input func
		ch = UART0->D;
//		rxChar=ch;		//to enable me take some action
		if ((int) ch != 194) { //removing 194 because after ascii 127, it displays 194, before actual ascii
//			printf("%d\n", ch);
//			handle_input();		//deal with input
			if (!Q_Full(&L_RxQ)) {
				Q_Enqueue(&L_RxQ, ch);
//				printf("%d\n", RxQ.Size);
			} else {
				// error - queue full, discard character
			}
		}
	}
}

void UART1_IRQHandler(void) {
	uint8_t ch;

	if (UART1->S1 & (UART_S1_OR_MASK |UART_S1_NF_MASK |
		UART_S1_FE_MASK | UART_S1_PF_MASK)) {
			// clear the error flags
			PTB->PTOR = MASK(OB_RED);

			// read the data register to clear RDRF
			ch = UART1->D;
	}
	if (UART1->S1 & UART_S1_RDRF_MASK) {
//		PTB->PTOR = MASK(OB_RED);
		// received a character. handle it in the handle input func
		ch = UART1->D;
//		rxChar=ch;		//to enable me take some action
		if ((int) ch != 194) { //removing 194 because after ascii 127, it displays 194, before actual ascii
//			printf("%d\n", ch);
//			handle_input();		//deal with input
			if (!Q_Full(&R_RxQ)) {
				Q_Enqueue(&R_RxQ, ch);
//				printf("%d\n", RxQ.Size);
			} else {
				// error - queue full, discard character
			}
		}
	}
}


void doRightRx_task() {
	uint8_t c;
	if (!Q_Empty(&R_RxQ)) {
		//remove character received from queue
		c = Q_Dequeue(&R_RxQ);
		switch(c) {
		case 181:
			R_next_part = Rgripper;
			break;
		case 182:
			R_next_part = Rwrist;
			break;
		case 183:
			R_next_part = Relbow;
			break;
		case 184:
			R_next_part = Rshoulder;
			break;
		default:
			handle_R_input(c);
			break;
		}
	}
}

void doLeftRx_task() {
	uint8_t c;
	if (!Q_Empty(&L_RxQ)) {
		//remove character received from queue
		c = Q_Dequeue(&L_RxQ);
		switch(c) {
		case 185:
			L_next_part = Lgripper;
			break;
		case 186:
			L_next_part = Lwrist;
			break;
		case 187:
			L_next_part = Lelbow;
			break;
		case 188:
			L_next_part = Lshoulder;
			break;
		default:
			handle_L_input(c);
			break;
		}
	}
}
