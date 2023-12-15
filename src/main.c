/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"
#include "conf_board.h"
#include "arm_math.h"
#include "logo_horizontal.c"


/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

#define LV_HOR_RES_MAX          (240)
#define LV_VER_RES_MAX          (320)
#define LV_DISP_ROT_90 

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;

lv_obj_t * labelHours;
lv_obj_t * labelDots1;
lv_obj_t * labelMinutes;
lv_obj_t * labelDots2;
lv_obj_t * labelSeconds;
lv_obj_t * labelAcceleration;

TimerHandle_t xTimer;

SemaphoreHandle_t xSemaphoreSeg;

/************************************************************************/
/* RTC                                                                  */
/************************************************************************/
int flag_rtc_alarm = 0;


typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;


void RTC_Handler(void) {
    uint32_t ul_status = rtc_get_status(RTC);

    /* seccond tick */
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
	// o código para irq de segundo vem aqui
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    	xSemaphoreGiveFromISR(xSemaphoreSeg, &xHigherPriorityTaskWoken);
    }

    /* Time or date alarm */
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
    	// o código para irq de alame vem aqui
        flag_rtc_alarm = 1;
    }

    rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
    rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
    rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
    rtc_clear_status(RTC, RTC_SCCR_CALCLR);
    rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

void RTC_Disable(Rtc *p_rtc)
{
    /* Disable the specified interrupt sources */
    p_rtc->RTC_IDR = 0xFFFFFFFF;

    /* Disable RTC interrupt */
    NVIC_DisableIRQ(RTC_IRQn);
}

uint32_t current_hour, current_min, current_sec;
calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};

/************************************************************************/
/* LED e Botão                                                          */
/************************************************************************/
// LED
#define LED_RAMP_PIO      PIOC
#define LED_RAMP_PIO_ID   ID_PIOC
#define LED_RAMP_IDX      8
#define LED_RAMP_IDX_MASK (1 << LED_RAMP_IDX)

// Bot�o
#define BUT_RAMP_PIO      PIOA
#define BUT_RAMP_PIO_ID   ID_PIOA
#define BUT_RAMP_IDX  11
#define BUT_RAMP_IDX_MASK (1 << BUT_RAMP_IDX)


/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_LCD_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_SIMULATOR_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_SIMULATOR_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_LED_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY (tskIDLE_PRIORITY)

#define RAIO 0.508/2
#define VEL_MAX_KMH  5.0f
#define VEL_MIN_KMH  0.5f

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/** Semaforo a ser usado pela task led */
SemaphoreHandle_t xSemaphoreBut_RAMP;
SemaphoreHandle_t xSemaphoreLED;

QueueHandle_t xQueueSpeed;

/** prototypes */
void but_callback(void);
float kmh_to_hz(float vel, float raio);
void io_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_RAMP_callback(void)
{
	 BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	 xSemaphoreGiveFromISR(xSemaphoreBut_RAMP, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/

static void event_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

lv_obj_t * lblSpeed;

void lv_ex_btn_1() {
	LV_IMG_DECLARE(logo_horizontal);
	lv_obj_t * img = lv_img_create(lv_scr_act());
	lv_img_set_src(img, &logo_horizontal);
	lv_obj_align(img, LV_ALIGN_TOP_LEFT, 0, 0);

	lblSpeed = lv_label_create(lv_scr_act());
	lv_obj_align(lblSpeed, LV_ALIGN_LEFT_MID, 100, 0);
	lv_obj_set_style_text_font(lblSpeed, &lv_font_montserrat_46, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(lblSpeed, lv_color_black(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(lblSpeed, "%d", 0);
	
	labelAcceleration = lv_label_create(lv_scr_act());
	lv_obj_align(labelAcceleration, LV_ALIGN_RIGHT_MID, -30, 0);
	lv_obj_set_style_text_font(labelAcceleration, &lv_font_montserrat_46, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelAcceleration, lv_color_black(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelAcceleration, "%d", 0);
	
	labelSeconds = lv_label_create(lv_scr_act());
	lv_obj_align(labelSeconds, LV_ALIGN_TOP_RIGHT, -5 , 5);
	lv_obj_set_style_text_font(labelSeconds, &lv_font_montserrat_14, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelSeconds, lv_color_black(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelSeconds, "%02d", 0);

	labelDots1 = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelDots1, labelSeconds, LV_ALIGN_OUT_LEFT_TOP, 25, 0);
	lv_obj_set_style_text_font(labelDots1, &lv_font_montserrat_14, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelDots1, lv_color_black(), LV_STATE_DEFAULT);
	lv_label_set_text(labelDots1, ":");

	labelMinutes = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelMinutes, labelDots1, LV_ALIGN_OUT_LEFT_TOP, 10, 0);
	lv_obj_set_style_text_font(labelMinutes, &lv_font_montserrat_14, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelMinutes, lv_color_black(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelMinutes, "%02d", 0);


	labelDots2 = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelDots2, labelMinutes, LV_ALIGN_OUT_LEFT_TOP, 25, 0);
	lv_obj_set_style_text_font(labelDots2, &lv_font_montserrat_14, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelDots2, lv_color_black(), LV_STATE_DEFAULT);
	lv_label_set_text(labelDots2, ":");

	labelHours = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelHours, labelDots2, LV_ALIGN_OUT_LEFT_TOP, 15, 0);
	lv_obj_set_style_text_font(labelHours, &lv_font_montserrat_14, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelHours, lv_color_black(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelHours, "%02d", 22);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_lcd(void *pvParameters) {
	int px, py;
	
	lv_ex_btn_1();

	for (;;)  {
		lv_tick_inc(50);
		lv_task_handler();
		vTaskDelay(50);
	}
}

static void task_simulador(void *pvParameters) {


	pmc_enable_periph_clk(ID_PIOC);
	pio_set_output(PIOC, PIO_PC31, 1, 0, 0);

	float vel = VEL_MAX_KMH;
	float vel_ant = VEL_MAX_KMH;
	float f;
	int ramp_up = 1;
	int ramp_status = 0;
	while(1){

		if (xSemaphoreTake(xSemaphoreBut_RAMP, 1)) {
			ramp_status = !ramp_status;
			 xSemaphoreGive(xSemaphoreLED);
		}
		pio_clear(PIOC, PIO_PC31);
		delay_ms(1);
		pio_set(PIOC, PIO_PC31);


		if(ramp_status == 1){
			if (ramp_up) {
				printf("[SIMU] ACELERANDO: %d \n", (int) (10*vel));
				vel_ant = vel;
				vel += 0.5;
				} else {
				printf("[SIMU] DESACELERANDO: %d \n",  (int) (10*vel));
				vel_ant = vel;
				vel -= 0.5;
			}

			if (vel >= VEL_MAX_KMH)
			ramp_up = 0;
			
			else if (vel <= VEL_MIN_KMH)
			ramp_up = 1;
		}

		else{
			vel = 5;
			printf("[SIMU] CONSTANTE: %d \n", (int) (10*vel));
		}
		
		
		if (vel_ant < vel){
			lv_label_set_text(labelAcceleration, LV_SYMBOL_UP);
		}
		else if (vel_ant > vel){
			lv_label_set_text(labelAcceleration, LV_SYMBOL_DOWN);
		}
		else if (vel_ant == vel){
			lv_label_set_text(labelAcceleration, " ");
		}
		
		
		lv_label_set_text_fmt(lblSpeed, "%d", (int) (10*vel));

		f = kmh_to_hz(vel, RAIO);
		int t = 500*(1.0/f); //UTILIZADO 965 como multiplicador ao inv�s de 1000
							 //para compensar o atraso gerado pelo Escalonador do freeRTOS
		delay_ms(t);
	}

}

static void task_led(void *pvParameters) {




	int ramp_status = 0;

	while(1){

		if (xSemaphoreTake(xSemaphoreLED, 1)) {
			ramp_status = !ramp_status;
		}

		if(ramp_status == 1) {

			pin_toggle(LED_RAMP_PIO, LED_RAMP_IDX_MASK); 
		}

		else {
			pio_set(LED_RAMP_PIO, LED_RAMP_IDX_MASK); 
		}

	}
}

static void task_rtc(void *pvParameters) {
	uint32_t current_hour, current_min, current_sec;

	RTC_init(RTC, ID_RTC, rtc_initial,RTC_IER_SECEN);
	int pisca = 0;
	for (;;)  {
		if(xSemaphoreTake(xSemaphoreSeg,0)){
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			if(pisca){
				lv_label_set_text_fmt(labelHours, "%02d", current_hour);
				lv_label_set_text_fmt(labelMinutes, "%02d", current_min);
				lv_label_set_text_fmt(labelSeconds, "%02d", current_sec);
				lv_label_set_text(labelDots1, " ");
				lv_label_set_text(labelDots2, " ");

				pisca = 0;
			}
			else{
				lv_label_set_text_fmt(labelHours, "%02d", current_hour);
				lv_label_set_text_fmt(labelMinutes, "%02d", current_min);
				lv_label_set_text_fmt(labelSeconds, "%02d", current_sec);
				lv_label_set_text(labelDots1, ":");
				lv_label_set_text(labelDots2, ":");
				pisca = 1;	
			}
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}


void io_init(void)
{

	// Configura led
	pmc_enable_periph_clk(LED_RAMP_PIO_ID);
	pio_configure(LED_RAMP_PIO, PIO_OUTPUT_0, LED_RAMP_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do perif�rico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_RAMP_PIO_ID);

	// Configura PIO para lidar com o pino do bot�o como entrada
	// com pull-up
	pio_configure(BUT_RAMP_PIO, PIO_INPUT, BUT_RAMP_IDX_MASK, PIO_PULLUP);
	pio_set_debounce_filter(BUT_RAMP_PIO, BUT_RAMP_IDX_MASK, 1000);

	// Configura interrup��o no pino referente ao botao e associa
	// fun��o de callback caso uma interrup��o for gerada
	// a fun��o de callback � a: but_callback()
	pio_handler_set(BUT_RAMP_PIO,
	BUT_RAMP_PIO_ID,
	BUT_RAMP_IDX_MASK,
	PIO_IT_EDGE,
	but_RAMP_callback);

	// Ativa interrup��o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_RAMP_PIO, BUT_RAMP_IDX_MASK);
	pio_get_interrupt_status(BUT_RAMP_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr�ximo de 0 maior)
	NVIC_EnableIRQ(BUT_RAMP_PIO_ID);
	NVIC_SetPriority(BUT_RAMP_PIO_ID, 4); // Prioridade 4
}

	/**
	* raio 20" => 50,8 cm (diametro) => 0.508/2 = 0.254m (raio)
	* w = 2 pi f (m/s)
	* v [km/h] = (w*r) / 3.6 = (2 pi f r) / 3.6
	* f = v / (2 pi r 3.6)
	* Exemplo : 5 km / h = 1.38 m/s
	*           f = 0.87Hz
	*           t = 1/f => 1/0.87 = 1,149s
	*/
	float kmh_to_hz(float vel, float raio) {
		float f = vel / (2*PI*raio*3.6);
		return(f);
	}
/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void configure_lcd(void) {
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	
	ili9341_init();
	ili9341_backlight_on();
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	
	/* IMPORTANT!!!
	* Inform the graphics library that you are ready with the flushing*/
	lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	
	if (readPoint(&px, &py))
		data->state = LV_INDEV_STATE_PRESSED;
	else
		data->state = LV_INDEV_STATE_RELEASED; 
	
	data->point.x = py;
	data->point.y = 320 - px;
}

void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	sysclk_init();
	io_init();
	configure_console();

	/* LCd, touch and lvgl init*/
	configure_lcd();
	ili9341_set_orientation(ILI9341_FLIP_Y | ILI9341_SWITCH_XY);
	configure_touch();
	configure_lvgl();

		/* Attempt to create a semaphore. */
	xSemaphoreBut_RAMP = xSemaphoreCreateBinary();
	if (xSemaphoreBut_RAMP == NULL)	 printf("falha em criar o semaforo \n");
	/* Attempt to create a semaphore. */
	xSemaphoreLED = xSemaphoreCreateBinary();
	if (xSemaphoreLED == NULL)	 printf("falha em criar o semaforo \n");
	
	xQueueSpeed = xQueueCreate(10, sizeof(int));
	if (xQueueSpeed == NULL) printf("Falha ao criar fila");

	/* Attempt to create a semaphore. */
	xSemaphoreSeg = xSemaphoreCreateBinary();
	if (xSemaphoreSeg == NULL)	 printf("falha em criar o semaforo \n");

	if (xTaskCreate(task_simulador, "SIMUL", TASK_SIMULATOR_STACK_SIZE, NULL, TASK_SIMULATOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}

	if (xTaskCreate(task_led, "LED", TASK_LED_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}

	if (xTaskCreate(task_rtc, "RTC", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
	printf("Failed to create rtc task\r\n");
	}

	/* Create task to control oled */
	if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){ }
}
