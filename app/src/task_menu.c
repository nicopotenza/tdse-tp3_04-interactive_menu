/*
 * Copyright (c) 2023 Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file   : task_menu.c
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/
/* Project includes */
#include "main.h"

/* Demo includes */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes */
#include "board.h"
#include "app.h"
#include "task_menu_attribute.h"
#include "task_menu_interface.h"
#include "display.h"

/********************** macros and definitions *******************************/
#define G_TASK_MEN_CNT_INI			0ul
#define G_TASK_MEN_TICK_CNT_INI		0ul

#define DEL_MEN_XX_MIN				0ul
#define DEL_MEN_XX_MED				50ul
#define DEL_MEN_XX_MAX				500ul

/********************** internal data declaration ****************************/
task_menu_dta_t task_menu_dta = { DEL_MEN_XX_MIN, ST_MEN_XX_MOTOR,
		EV_MEN_ENT_IDLE, false };
//task_menu_dta_t task_menu_dta = { DEL_MEN_XX_MIN, ST_MEN_XX_IDLE, EV_MEN_ENT_IDLE, false };

#define MENU_DTA_QTY	(sizeof(task_menu_dta)/sizeof(task_menu_dta_t))

typedef struct {
	int power;
	int speed;
	int spin;
} motor_dta_t;

motor_dta_t motor[2];

int motor_n;
int power_value;
int speed_value;
int spin_value;

char deleteLine[] = "                ";

/********************** internal functions declaration ***********************/
void task_menu_statechart(void);

/********************** internal data definition *****************************/
const char *p_task_menu = "Task Menu (Interactive Menu)";
const char *p_task_menu_ = "Non-Blocking & Update By Time Code";

/********************** external data declaration ****************************/
uint32_t g_task_menu_cnt;
volatile uint32_t g_task_menu_tick_cnt;

/********************** external functions definition ************************/
void task_menu_init(void *parameters) {
	task_menu_dta_t *p_task_menu_dta;
	task_menu_st_t state;
	task_menu_ev_t event;
	bool b_event;
	char menu_str[16];

	/* Print out: Task Initialized */
	LOGGER_INFO(" ");
	LOGGER_INFO("  %s is running - %s", GET_NAME(task_menu_init), p_task_menu);
	LOGGER_INFO("  %s is a %s", GET_NAME(task_menu), p_task_menu_);

	/* Init & Print out: Task execution counter */
	g_task_menu_cnt = G_TASK_MEN_CNT_INI;
	LOGGER_INFO("   %s = %lu", GET_NAME(g_task_menu_cnt), g_task_menu_cnt);

	init_queue_event_task_menu();

	/* Update Task Actuator Configuration & Data Pointer */
	p_task_menu_dta = &task_menu_dta;

	/* Init & Print out: Task execution FSM */
//	state = ST_MEN_XX_IDLE;
	state = ST_MEN_XX_MOTOR;
	p_task_menu_dta->state = state;

	event = EV_MEN_ENT_IDLE;
	p_task_menu_dta->event = event;

	b_event = false;
	p_task_menu_dta->flag = b_event;

	LOGGER_INFO(" ");
	LOGGER_INFO("   %s = %lu   %s = %lu   %s = %s", GET_NAME(state),
			(uint32_t )state, GET_NAME(event), (uint32_t )event,
			GET_NAME(b_event), (b_event ? "true" : "false"));

	/* Init & Print out: LCD Display */
	displayInit(DISPLAY_CONNECTION_GPIO_4BITS);

	displayCharPositionWrite(0, 0);
	displayStringWrite("TdSE Bienvenidos");

	displayCharPositionWrite(0, 1);
	displayStringWrite("Test Nro: ");

	motor[0].power = 0;
	motor[0].speed = 0;
	motor[0].spin = 0;
	motor[1].power = 0;
	motor[1].speed = 0;
	motor[1].spin = 0;
	motor_n = 0;
	power_value = motor[motor_n].power;
	speed_value = motor[motor_n].speed;
	spin_value = motor[motor_n].spin;

	// Print out: LCD Display
	snprintf(menu_str, sizeof(menu_str), "Motor %d", motor_n);
	displayCharPositionWrite(0, 0);
	displayStringWrite(menu_str);
	LOGGER_INFO("--> %s", menu_str);
	// Print out: LCD Display
	snprintf(menu_str, sizeof(menu_str), "P: %d|S: %d|D: %d", power_value,
			speed_value, spin_value);
	displayCharPositionWrite(0, 1);
	displayStringWrite(menu_str);
	LOGGER_INFO("--> %s", menu_str);
}

void task_menu_update(void *parameters) {
	bool b_time_update_required = false;

	/* Protect shared resource */
	__asm("CPSID i");
	/* disable interrupts */
	if (G_TASK_MEN_TICK_CNT_INI < g_task_menu_tick_cnt) {
		/* Update Tick Counter */
		g_task_menu_tick_cnt--;
		b_time_update_required = true;
	}
	__asm("CPSIE i");
	/* enable interrupts */

	while (b_time_update_required) {
		/* Update Task Counter */
		g_task_menu_cnt++;

		/* Run Task Menu Statechart */
		task_menu_statechart();

		/* Protect shared resource */
		__asm("CPSID i");
		/* disable interrupts */
		if (G_TASK_MEN_TICK_CNT_INI < g_task_menu_tick_cnt) {
			/* Update Tick Counter */
			g_task_menu_tick_cnt--;
			b_time_update_required = true;
		} else {
			b_time_update_required = false;
		}
		__asm("CPSIE i");
		/* enable interrupts */
	}
}

void task_menu_statechart(void) {
	task_menu_dta_t *p_task_menu_dta;
	char menu_str[17];

	/* Update Task Menu Data Pointer */
	p_task_menu_dta = &task_menu_dta;

	if (true == any_event_task_menu()) {
		p_task_menu_dta->flag = true;
		p_task_menu_dta->event = get_event_task_menu();
	}

	switch (p_task_menu_dta->state) {
	case ST_MEN_XX_MOTOR:
		if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_POWER;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Motor %d | Power", motor_n);
			displayCharPositionWrite(0, 0);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), deleteLine);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		}
		if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event)) {
			motor[motor_n].power = power_value;
			motor[motor_n].speed = speed_value;
			motor[motor_n].spin = spin_value;
			if (motor_n == 0) {
				motor_n = 1;

			} else if (motor_n == 1) {
				motor_n = 0;
			}
			power_value = motor[motor_n].power;
			speed_value = motor[motor_n].speed;
			spin_value = motor[motor_n].spin;
			p_task_menu_dta->flag = false;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Motor %d", motor_n);
			displayCharPositionWrite(0, 0);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "P: %d|S: %d|D: %d",
					power_value, speed_value, spin_value);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		}
		break;
	case ST_MEN_XX_POWER:
		if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_POWER_VALUE;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Value: %d", power_value);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		} else if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_SPEED;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Motor %d | Speed", motor_n);
			displayCharPositionWrite(0, 0);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), deleteLine);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		} else if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_MOTOR;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Motor %d", motor_n);
			displayCharPositionWrite(0, 0);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "P: %d|S: %d|D: %d",
					power_value, speed_value, spin_value);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		}
		break;
	case ST_MEN_XX_POWER_VALUE:
		if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_POWER;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), deleteLine);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		} else if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			if (1 > power_value)
				power_value++;
			else if (1 == power_value)
				power_value = 0;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Value: %d", power_value);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		}
		break;
	case ST_MEN_XX_SPEED:
		if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_SPEED_VALUE;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Value: %d", speed_value);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), deleteLine);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		} else if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_SPIN;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Motor %d | Spin", motor_n);
			displayCharPositionWrite(0, 0);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), deleteLine);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		} else if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_MOTOR;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Motor %d", motor_n);
			displayCharPositionWrite(0, 0);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "P: %d|S: %d|D: %d",
					power_value, speed_value, spin_value);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		}
		break;
	case ST_MEN_XX_SPEED_VALUE:
		if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_SPEED;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), deleteLine);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		} else if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			if (9 > speed_value)
				speed_value++;
			else if (9 == speed_value)
				speed_value = 0;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Value: %d", speed_value);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		}
		break;
	case ST_MEN_XX_SPIN:
		if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_SPIN_DIR;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Value: %d", spin_value);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		} else if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_POWER;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Motor %d | Power", motor_n);
			displayCharPositionWrite(0, 0);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), deleteLine);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		} else if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_MOTOR;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Motor %d", motor_n);
			displayCharPositionWrite(0, 0);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "P: %d|S: %d|D: %d",
					power_value, speed_value, spin_value);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		}
		break;
	case ST_MEN_XX_SPIN_DIR:
		if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			p_task_menu_dta->state = ST_MEN_XX_SPIN;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), deleteLine);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		} else if ((true == p_task_menu_dta->flag)
				&& (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event)) {
			p_task_menu_dta->flag = false;
			if (1 > spin_value)
				spin_value++;
			else if (1 == spin_value)
				spin_value = 0;
			// Print out: LCD Display
			snprintf(menu_str, sizeof(menu_str), "Value: %d", spin_value);
			displayCharPositionWrite(0, 1);
			displayStringWrite(menu_str);
			LOGGER_INFO("--> %s", menu_str);
		}

		break;
		/*
		 case ST_MEN_XX_IDLE:

		 if ((true == p_task_menu_dta->flag) && (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event))
		 {
		 p_task_menu_dta->tick = DEL_MEN_XX_MAX;
		 p_task_menu_dta->flag = false;
		 p_task_menu_dta->state = ST_MEN_XX_ACTIVE;
		 }

		 break;

		 case ST_MEN_XX_ACTIVE:

		 if ((true == p_task_menu_dta->flag) && (EV_MEN_ENT_IDLE == p_task_menu_dta->event))
		 {
		 p_task_menu_dta->flag = false;
		 p_task_menu_dta->state = ST_MEN_XX_IDLE;
		 }
		 else
		 {
		 p_task_menu_dta->tick--;
		 if (DEL_MEN_XX_MIN == p_task_menu_dta->tick)
		 {
		 p_task_menu_dta->tick = DEL_MEN_XX_MAX;

		 // Print out: LCD Display
		 snprintf(menu_str, sizeof(menu_str), "%lu", (g_task_menu_cnt/1000ul));
		 displayCharPositionWrite(10, 1);
		 displayStringWrite(menu_str);
		 }
		 }

		 break;
		 */
	default:

		p_task_menu_dta->tick = DEL_MEN_XX_MIN;
//			p_task_menu_dta->state = ST_MEN_XX_IDLE;
		p_task_menu_dta->state = ST_MEN_XX_MOTOR;
		p_task_menu_dta->event = EV_MEN_ENT_IDLE;
		p_task_menu_dta->flag = false;
		motor_n = 0;
		power_value = motor[motor_n].power;
		speed_value = motor[motor_n].speed;
		spin_value = motor[motor_n].spin;
		// Print out: LCD Display
		snprintf(menu_str, sizeof(menu_str), "Motor %d", motor_n);
		displayCharPositionWrite(0, 0);
		displayStringWrite(menu_str);
		LOGGER_INFO("--> %s", menu_str)
		;
		// Print out: LCD Display
		snprintf(menu_str, sizeof(menu_str), "P: %d|S: %d|D: %d", power_value,
				speed_value, spin_value);
		displayCharPositionWrite(0, 1);
		displayStringWrite(menu_str);
		LOGGER_INFO("--> %s", menu_str)
		;
		break;
	}
}

/********************** end of file ******************************************/
