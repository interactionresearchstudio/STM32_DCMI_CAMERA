/*
 ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */
#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include "shell.h"

#include "hwinit.h"
#include "SCCB.h"
#include "OV2640.h"
#include "evtimer.h"
#include "ff.h"

/*SD CARD */
#define POLLING_INTERVAL                10
#define POLLING_DELAY                   10
static EventSource inserted_event, removed_event;

/**
 * @brief   Card monitor timer.
 */
static VirtualTimer tmr;

/**
 * @brief   Debounce counter.
 */
static unsigned cnt;

static void tmrfunc(void *p) {
	BaseBlockDevice *bbdp = p;

	chSysLockFromIsr()
	;
	if (cnt > 0) {
		if (blkIsInserted(bbdp)) {
			if (--cnt == 0) {
				chEvtBroadcastI(&inserted_event);
			}
		} else
			cnt = POLLING_INTERVAL;
	} else {
		if (!blkIsInserted(bbdp)) {
			cnt = POLLING_INTERVAL;
			chEvtBroadcastI(&removed_event);
		}
	}
	chVTSetI(&tmr, MS2ST(POLLING_DELAY), tmrfunc, bbdp);
	chSysUnlockFromIsr();
}

static void tmr_init(void *p) {

	chEvtInit(&inserted_event);
	chEvtInit(&removed_event);
	chSysLock()
	;
	cnt = POLLING_INTERVAL;
	chVTSetI(&tmr, MS2ST(POLLING_DELAY), tmrfunc, p);
	chSysUnlock();
}

/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
FATFS MMC_FS;

/**
 * MMC driver instance.
 */
MMCDriver MMCD1;

/* FS mounted and ready.*/
static bool_t fs_ready = FALSE;

/* Maximum speed SPI configuration (18MHz, CPHA=0, CPOL=0, MSb first).*/
static SPIConfig hs_spicfg = { NULL, GPIOB, GPIOB_PIN12, 0 };

/* Low speed SPI configuration (281.250kHz, CPHA=0, CPOL=0, MSb first).*/
static SPIConfig ls_spicfg = { NULL, GPIOB, GPIOB_PIN12, SPI_CR1_BR_2
		| SPI_CR1_BR_1 };

/* MMC/SD over SPI driver configuration.*/
static MMCConfig mmccfg = { &SPID2, &ls_spicfg, &hs_spicfg };

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

	(void) arg;
	chRegSetThreadName("blinker");
	while (TRUE) {
		palTogglePad(GPIOD, GPIOD_LED4);
		chThdSleepMilliseconds(fs_ready ? 125 : 500);
	}
	return 0;
}

static void InsertHandler(eventid_t id) {
	FRESULT err;

	(void) id;
	/*
	 * On insertion MMC initialization and FS mount.
	 */
	if (mmcConnect(&MMCD1)) {
		return;
	}
	err = f_mount(0, &MMC_FS);
	if (err != FR_OK) {
		mmcDisconnect(&MMCD1);
		return;
	}
	fs_ready = TRUE;
}

/*
 * MMC card removal event.
 */
static void RemoveHandler(eventid_t id) {

	(void) id;
	mmcDisconnect(&MMCD1);
	fs_ready = FALSE;
}

// Question related
#define MAXQUESTIONS 50
uint16_t questionPositions[MAXQUESTIONS];

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/
static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_cam_id(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_cam_init(BaseSequentialStream *chp, int argc, char *argv[]); // -
static void cmd_cam_on(BaseSequentialStream *chp, int argc, char *argv[]); // -
static void cmd_cam_off(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_cam_reset(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_cam_capture(BaseSequentialStream *chp, int argc, char *argv[]); // -
static void cmd_cam_save(BaseSequentialStream *chp, int argc, char *argv[]); // - call it save. take in file name arg
static void cmd_cam_status(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_cam_reg_write(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_cam_reg_read(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_index_questions(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_get_total_questions(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_get_question(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_mark_question(BaseSequentialStream *chp, int argc, char *argv[]);

static uint8_t AsciiToHex(char c);
static void cam_init();
static void cam_on();
static void cam_capture();
static void cam_save(char* filename);

// Question variables
uint16_t questionPositions[50];
uint8_t numOfQuestions;

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)
#define BUFFER_SIZE     100000               // Max Image Size

static const ShellCommand commands[] = { { "mem", cmd_mem }, { "threads",
		cmd_threads }, { "cam_id", cmd_cam_id }, { "cam_init", cmd_cam_init }, {
		"cam_on", cmd_cam_on }, { "cam_off", cmd_cam_off }, { "cam_reset",
		cmd_cam_reset }, { "cam_capture", cmd_cam_capture }, { "cam_save",
		cmd_cam_save }, { "cam_status", cmd_cam_status }, { "cam_reg_write",
		cmd_cam_reg_write }, { "cam_reg_read", cmd_cam_reg_read },
		{ "index_qs", cmd_index_questions }, { "get_total_qs", cmd_get_total_questions },
		{ "get_question", cmd_get_question }, { "mark_question", cmd_mark_question },
		{ NULL, NULL } };

static const ShellConfig shell_cfg1 =
		{ (BaseSequentialStream *) &SD2, commands };

int FrameCount = 0; // Number of frames received

void frameEndCb(DCMIDriver* dcmip) {
	(void) dcmip;
	// We count reception of ten frames. DMA does not store the first few frames
	// received for some reason, so we must skip the first few frames and stop
	// the DCMI on tenth frame reception, otherwise nothing is saved by the DMA
	// The tenth frame is arbitrary, but it seems it always works, while stopping
	// on less than 10 frames does not always work.
	FrameCount++;
	if (FrameCount >= 10) {
		dcmiStop(&DCMID1);
		FrameCount = 0;
	} palTogglePad(GPIOD, 12) ; // Green
}

void dmaTxferEndCb(DCMIDriver* dcmip) {
	(void) dcmip;
	palTogglePad(GPIOD, 15); // Blue
	// This Never Occurs!
}

/* Status Registers */
uint8_t power = 0; // 0 - OFF, 1 - ON
uint8_t busy = 0;  // 0 - NOT BUSY, 1 - BUSY
uint8_t init = 0;  // 0 - NOT INITiated, 1 - INITiated
uint8_t captured = 0; // 0 - image NOT captured, 1 - image captured and buffered
uint8_t error = 0x00; // Error register

/* DMA and DCMI Registers */
uint32_t DmaMode; // DMA Mode Setting to be loaded here
const stm32_dma_stream_t *DmaStreamType; // DMA Stream Select
uint8_t ImageBuffer[BUFFER_SIZE]; // This will hold the JPEG data after acquisition
/* Split the ImageBuffer into two pointers for DCMI driver */
uint8_t *ImageBuffer0 = ImageBuffer;
uint8_t *ImageBuffer1 = &ImageBuffer[BUFFER_SIZE / 2];

/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/
int main(void) {

	static const evhandler_t evhndl[] = { InsertHandler, RemoveHandler };
	Thread *shelltp = NULL;
	struct EventListener el0, el1;

	/* Initializes ChibiOS HAL and RTOs */
	halInit();
	chSysInit();
	/* Initializes Project Specific HW resources */
	hwInit();

	sdStart(&SD2, NULL);
	palSetPadMode(GPIOA, GPIOA_PIN2, PAL_MODE_ALTERNATE(7)); palSetPadMode(GPIOA, GPIOA_PIN3, PAL_MODE_ALTERNATE(7));

	/* Shell initialization. */
	shellInit();

	mmcObjectInit(&MMCD1);
	mmcStart(&MMCD1, &mmccfg);
	palSetPad(GPIOB, 12); palSetPadMode(GPIOB, GPIOB_PIN12, PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST); /* NSS.     */
	palSetPadMode(GPIOB, GPIOB_PIN13, PAL_MODE_ALTERNATE(5) |
			PAL_STM32_OSPEED_HIGHEST); /* SCK.     */
	palSetPadMode(GPIOB, GPIOB_PIN14, PAL_MODE_ALTERNATE(5)); /* MISO.    */
	palSetPadMode(GPIOB, GPIOB_PIN15, PAL_MODE_ALTERNATE(5) |
			PAL_STM32_OSPEED_HIGHEST); /* MOSI.    */
	/*
	 * Activates the card insertion monitor.
	 */
	tmr_init(&MMCD1);

	/* Normal main() thread activity, in this demo it just performs */
	/* a shell respawn upon its termination.
	 *                       */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and listen for events.
	 */
	chEvtRegister(&inserted_event, &el0, 0);
	chEvtRegister(&removed_event, &el1, 1);

	palSetPadMode(GPIOA,0, PAL_MODE_INPUT);

	while (TRUE) {

		if (!shelltp)
			shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminated(shelltp)) {
			chThdRelease(shelltp); /* Recovers memory of the previous shell.   */
			shelltp = NULL; /* Triggers spawning of a new shell.        */
		}
		chEvtDispatch(evhndl, chEvtWaitOne(ALL_EVENTS));
	}
	return 0;
}

/*===========================================================================*/
/* Command line commands.                                                    */
/*===========================================================================*/
static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
	size_t n, size;

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: mem\r\n");
		return;
	}
	n = chHeapStatus(NULL, &size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
	chprintf(chp, "heap fragments   : %u\r\n", n);
	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
	static const char *states[] = { THD_STATE_NAMES };
	Thread *tp;

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: threads\r\n");
		return;
	}
	chprintf(chp, "    addr    stack prio refs     state time\r\n");
	tp = chRegFirstThread();
	do {
		chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n", (uint32_t) tp,
				(uint32_t) tp->p_ctx.r13, (uint32_t) tp->p_prio,
				(uint32_t) (tp->p_refs - 1), states[tp->p_state],
				(uint32_t) tp->p_time);
		tp = chRegNextThread(tp);
	} while (tp != NULL);
}

static void cmd_cam_id(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Reads the CAM id registers via the SCCB bus and returns them */
	/* This can be called BEFORE cam_init to verify the camera presence */
	(void) argc;
	(void) argv;
	uint8_t val;
	/* Jump to DSP bank of camera regs */
	chprintf(chp, "Reading Cam ID Registers\r\n");
	if (cam_write_reg(0xFF, 0x01) != 0) {
		chprintf(chp, "Error setting page\r\n");
	}
	if (cam_read_reg(0x0A, &val) != 0) {
		chprintf(chp, "Error Reading High Byte\r\n");
	} else {
		chprintf(chp, "Cam ID high is: %x\r\n", val);
	}
	if (cam_read_reg(0x0B, &val) != 0) {
		chprintf(chp, "Error Reading Low Byte\r\n");
	} else {
		chprintf(chp, "Cam ID low is: %x\r\n", val);
	}
}

static void cmd_cam_init(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Sends the required register arrays to the camera via the SCCB bus
	 * in order to configure the camera for JPEG capture mode. Registers defined
	 * in OV2640 files
	 */
	(void) argc;
	(void) argv;

	cam_init();
}

static void cmd_cam_off(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Drives the control signals of the power supplies that power the camera
	 * to turn them off, effectively turning the camera power off. First, puts
	 * the camera in power down mode.
	 */
	(void) argc;
	(void) argv;
	chprintf(chp, "Turning CAM OFF\r\n");
	/* Stop the clock */
	palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_OPENDRAIN); palSetPad(GPIOA, 8);

	/* Enter Powerdown mode */
	palSetPad(GPIOA, 1); // PWDN pin

	chThdSleepMilliseconds(5);

	/* Drive the power supply enable signals LOW here! */
	power = 0;
	init = 0;
	chprintf(chp, "Camera is OFF\r\n");
}

static void cmd_cam_on(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Drives the control signals of the power supplies that power the camera
	 * to turn them on in the correct sequence, effectively turning the power
	 * to the camera ON. A reset is required after this, then initialization.
	 */
	(void) argc;
	(void) argv;
	chprintf(chp, "Setting CAM ON\r\n");
	cam_on();
}

static void cmd_cam_reset(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Drives the camera reset and powerdown control signals in the correct
	 * sequence to perform a camera reset via hardware.
	 */
	(void) argc;
	(void) argv;

	chprintf(chp, "Resetting CAM\r\n");
	/* reset the camera */
	palClearPad(GPIOA, 0);
	chThdSleepMilliseconds(5); palSetPad(GPIOA, 0);
	chThdSleepMilliseconds(5);
	/* set PWDN low to exit power down */
	palClearPad(GPIOA, 1);
	chThdSleepMilliseconds(10);
	init = 0;
	captured = 0;
	busy = 0;
}

static void cmd_cam_capture(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Starts the DCMI and DMA streams in order to capture a single JPEG frame
	 * from the camera via DVP interface and store it in internal MCU memory.
	 */
	(void) argc;
	(void) argv;

	cam_capture();
	chprintf(chp, "Image capture complete.\r\n");
}

static void cmd_cam_save(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Sends the captured JPEG frame from the internal MCU memory via serial
	 * interface.
	 */

	/*
	 (void)argc;
	 (void)argv;
	 uint16_t i;

	 for (i = 0; i < BUFFER_SIZE; i++) {
	 if ( (ImageBuffer[i] == 0xFF) && (ImageBuffer[i+1] == 0xD9) ) {

	 chprintf(chp, "%c", ImageBuffer[i]);
	 chprintf(chp, "%c", ImageBuffer[i+1]);
	 break;
	 }
	 chprintf(chp, "%c", ImageBuffer[i]);
	 }
	 */

	if (argc != 1) {
		chprintf(chp, "Wrong number of arguments!\r\n");
		chprintf(chp, "Enter: cmd_cam_save FILENAME.jpg\r\n");
		return;
	}

	char incomingChar = 0x01;
	char filename[13];
	uint8_t i = 0;

	while (incomingChar != 0) {
		incomingChar = argv[0][i];
		filename[i] = incomingChar;
		i++;
	}

	chprintf(chp, filename);
	chprintf(chp, " being saved...\r\n");

	cam_save(filename);
}

static void cmd_cam_status(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Returns the current camera status.
	 * Status flags
	 * Power ON/OFF
	 * Busy YES/NO
	 * Initialized YES/NO
	 * Image Buffered YES/NO (have we captured but not sent an image?)
	 */
	(void) argc;
	(void) argv;
	chprintf(chp, "CAMERA STATUS\r\n");
	if (power) {
		chprintf(chp, "Power:\t\t\tON\r\n");
	} else {
		chprintf(chp, "Power:\t\t\tOFF\r\n");
	}

	if (init) {
		chprintf(chp, "Initialized:\t\tYES\r\n");
	} else {
		chprintf(chp, "Initialized:\t\tNO\r\n");
	}

	if (captured) {
		chprintf(chp, "Image Buffered:\tYES\r\n");
	} else {
		chprintf(chp, "Image Buffered:\tNO\r\n");
	}

	if (busy) {
		chprintf(chp, "Busy:\t\t\tYES\r\n");
	} else {
		chprintf(chp, "Busy:\t\t\tNO\r\n");
	}

	chprintf(chp, "Error Register:\t0x%02x\r\n", error);
}

static void cmd_cam_reg_write(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Writes the value to the register specified, where value and register
	 * are passed in as arguments. For example cmd_cam_reg_write FF 34 would
	 * write the value 0x34 to the register 0xFF.
	 */
	uint8_t val;
	uint8_t reg;

	if (argc != 2) {
		chprintf(chp, "Wrong number of arguments!\r\n");
		chprintf(chp,
				"Enter: cam_reg_write AA BB, where AA is register and BB is the value.\r\n");
		return;
	}

	/* Prepare reg and value from command line arguments */
	reg = (AsciiToHex(argv[0][0]) << 4) ^ AsciiToHex(argv[0][1]);
	val = (AsciiToHex(argv[1][0]) << 4) ^ AsciiToHex(argv[1][1]);
	chprintf(chp, "Writing Register Location 0x%x with Value 0x%x\r\n", reg,
			val);
	if (cam_write_reg(reg, val) != 0) {
		chprintf(chp, "Error writing to camera registers.\r\n");
		return;
	}
	chprintf(chp, "Register write complete\r\n");
}

static void cmd_cam_reg_read(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Reads the register specified and passed in as an argument. For instance
	 * issuing a cmd_cam_reg_read FF would read and return the value of the
	 * 0xFF register on the camera.
	 */
	uint8_t val;
	uint8_t reg = 0x00;

	if (argc != 1) {
		chprintf(chp, "Wrong number of arguments!\r\n");
		chprintf(chp,
				"Enter: cam_reg_read AA, where AA is register location\r\n");
		return;
	}

	reg = (AsciiToHex(argv[0][0]) << 4) ^ AsciiToHex(argv[0][1]);
	chprintf(chp, "Reading Register Location 0x%x\r\n", reg);
	if (cam_read_reg(reg, &val) != 0) {
		chprintf(chp, "Error reading specified register.\r\n");
	} else {
		chprintf(chp, "Read Value in HEX is: 0x%x\r\n", val);
	}
}

static void cmd_index_questions(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Indexes the question file, storing the start index of each line.
	 * No returns.
	 * No parameters.
	 */
	FIL fsrc; /* file object */
	FRESULT err;

	chprintf(chp, "Indexing questions...\r\n");

	err = f_open(&fsrc, "q.txt", FA_READ);
	if (err != FR_OK) {
		chprintf(chp, "Failed to open q.txt.\r\n");
		//verbose_error(chp, err);
		return;
	} else {
		chprintf(chp, "q.txt opened.\r\n");
		numOfQuestions = 0;
		uint16_t filesize = f_size(&fsrc);
		uint16_t index;
		chprintf(chp, "%d bytes in file.\r\n", filesize);
		while (!f_eof(&fsrc)) {
			char inString[100];
			uint8_t numOfBytesRead;
			f_gets(&inString, 100, &fsrc);
			chprintf(chp, inString);
			questionPositions[numOfQuestions+1] = f_tell(&fsrc);
			numOfQuestions++;
		}
		chprintf(chp, "\r\n%d questions found.\r\n", numOfQuestions);
	}

	f_close(&fsrc);
}

static void cmd_get_total_questions(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Gets the total number of questions in the question file.
	 * Also calls cmd_index_questions in order to index the file prior to counting.
	 * Returns a uint8_t through UART.
	 * No parameters.
	 */
	chprintf(chp, "Total num of questions: %d\r\n", numOfQuestions);
}

static void cmd_get_question(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Retrieves a question from the question file, along with the number of answers.
	 * Returns a packet containing the question content, followed by the number of answers (ASCII)
	 * through UART.
	 * Parameter - The question index.
	 */
	uint8_t val = 0;
	FIL fsrc; /* file object */
	FRESULT err;

	if (argc != 1) {
		chprintf(chp, "Wrong number of arguments!\r\n");
		chprintf(chp, "Enter: get_question AA, where AA is an integer from 1 to 99\r\n");
		return;
	}

	if (argv[0][1] == 0) val = AsciiToHex(argv[0][0]);
	else val = (AsciiToHex(argv[0][0]) * 10) + AsciiToHex(argv[0][1]);
	chprintf(chp, "Getting question %d\r\n", val);
	chprintf(chp, "Opening q.txt...\r\n");

	err = f_open(&fsrc, "q.txt", FA_READ);
	if (err != FR_OK) {
		chprintf(chp, "Failed to open q.txt.\r\n");
		//verbose_error(chp, err);
		return;
	} else {
		chprintf(chp, "q.txt opened.\r\n");
		char inString[100];
		f_lseek(&fsrc, questionPositions[val]);
		f_gets(&inString, 100, &fsrc);
		chprintf(chp, inString);
	}
	f_close(&fsrc);
}
static void cmd_mark_question(BaseSequentialStream *chp, int argc, char *argv[]) {
	/* Marks a question as answered in the questions file.
	 * No returns.
	 * Parameter - The question index.
	 */
	uint8_t val;
	FIL fsrc; /* file object */
	FRESULT err;

	if (argc != 1) {
		chprintf(chp, "Wrong number of arguments!\r\n");
		chprintf(chp, "Enter: mark_question n, where n is an integer from 1 to 99\r\n");
		return;
	}

	if (argv[0][1] == 0) val = AsciiToHex(argv[0][0]);
	else val = (AsciiToHex(argv[0][0]) * 10) + AsciiToHex(argv[0][1]);
	chprintf(chp, "Marking question %d\r\n", val);
	chprintf(chp, "Opening q.txt...\r\n");

	err = f_open(&fsrc, "q.txt", FA_READ | FA_WRITE);
	if (err != FR_OK) {
		chprintf(chp, "Failed to open q.txt.\r\n");
		//verbose_error(chp, err);
		return;
	} else {
		chprintf(chp, "q.txt opened.\r\n");

		uint16_t filesize = f_size(&fsrc);

		f_lseek(&fsrc, filesize+1);

		char *inChar;
		inChar = (char *)malloc(sizeof(char));

		char thisChar;
		char nextChar;

		f_lseek(&fsrc, questionPositions[val]);
		f_read(&fsrc, inChar, 1, 1);
		nextChar = *inChar;
		f_lseek(&fsrc, f_tell(&fsrc)-1);
		f_putc('#', &fsrc);

//		while(!f_eof(&fsrc)) {
		uint16_t index;
		for(index = questionPositions[val]; index < filesize; index++) {
			thisChar = nextChar;
			f_read(&fsrc, inChar, 1, 1);
			nextChar = *inChar;
			f_lseek(&fsrc, f_tell(&fsrc)-1);
			f_putc(thisChar, &fsrc);
		}
		//f_lseek(&fsrc, f_tell(&fsrc)+1);
		//f_putc(nextChar, &fsrc);

		chprintf(chp, "Question marked.\r\n");
	}
	f_close(&fsrc);
}

static void cam_init() {
	/* Send the required arrays to init and set the cam to JPEG output */
	if (cam_write_array(ov2640_reset_regs) != 0) {
		//chprintf(chp, "reset regs write failed\r\n");
		error |= 0x01;
	}

	chThdSleepMilliseconds(250);

	if (cam_write_array(ov2640_jpeg_init_regs) != 0) {
		//chprintf(chp, "init regs write failed\r\n");
		error |= 0x02;
	}
	if (cam_write_array(ov2640_yuv422_regs) != 0) {
		//chprintf(chp, "yuv422 regs write failed\r\n");
		error |= 0x04;
	}

	if (cam_write_reg(0xFF, 0x01) != 0) {
		//chprintf(chp, "Error setting page\r\n");
	}

	if (cam_write_reg(0x15, 0x00) != 0) {
		//chprintf(chp, "Error setting page\r\n");
	}

	/* To change resolutions change the below register */
	if (cam_write_array(ov2640_jpeg_regs) != 0) {
		//chprintf(chp, "jpeg regs write failed\r\n");
		error |= 0x08;
	}

	chThdSleepMilliseconds(100);

	/* To change resolutions change the below register */
	/* For 320x240 use ov2640_320x240_regs             */
	/* For 352x288 use ov2640_352x288_regs             */
	/* For 640x480 use ov2640_640x480_regs             */
	/* For 800x600 use ov2640_800x600_regs             */
	/* For 1024x768 use ov2640_1024x768_regs           */
	/*For 1024x768  use ov2640_1280x1024_reg 			*/
	//if (cam_write_array(ov2640_1280x1024_regs) != 0) {
	if (cam_write_array(ov2640_1024x768_regs) != 0) {
		//chprintf(chp, "Resolution regs write failed\r\n");
		error |= 0x10;
	}

	if (cam_write_array(ov2640_jpeg_regs) != 0) {
		//chprintf(chp, "jpeg_regs write failed\r\n");
		error |= 0x20;
	}

	/* ov2640_negative */
	if (cam_write_array(ov2640_normal) != 0) {
		//chprintf(chp, "BW write failed\r\n");
	}

	if (cam_write_array(ov2640_autolight) != 0) {
		//chprintf(chp, "autolight failed");
	}

	if (error != 0x00) {
		//chprintf(chp, "CAM Init Failed.\r\n");
		init = 0;
	} else {
		//chprintf(chp, "CAM Init Completed.\r\n");
		init = 1;
	}
}

static void cam_on() {
	/* Apply Clock */
	pwmEnableChannel(&PWMD1, 0, 2);
	init = 0;
	captured = 0;
	busy = 0;
	power = 1;
	/* Define power supply ENABLE pins and then assert them here! */
}

static void cam_capture() {
	busy = 1;
	dcmiStart(&DCMID1, &dcmicfg);
	chThdSleepMilliseconds(250);
	dcmiStartReceiveOneShot(&DCMID1, BUFFER_SIZE / 2, ImageBuffer0,
			ImageBuffer1);
	chThdSleepMilliseconds(250);
	//chprintf(chp, "Image Capture Complete\r\n", dmaStreamGetTransactionSize(DCMID1.dmarx));
	busy = 0;
	captured = 1;
}

static void cam_save(char* filename) {
	FIL fsrc; /* file object */
	FRESULT err;

	err = f_open(&fsrc, filename, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
	if (err != FR_OK) {
		//chprintf(chp, "FS: f_open(\"hello.txt\") failed.\r\n");
		//	verbose_error(chp, err);
		return;
	} else {
		//chprintf(chp, "FS: f_open(\"hello.txt\") succeeded\r\n");
	}
	/*
	 * Write text to the file.
	 */

	/*
	 * In this loop, we should have an overflow condition - if we've reached
	 * 100K, we're overflown, so we should return an error.
	 */
	uint16_t i;

	for (i = 0; i < BUFFER_SIZE; i++) {
		if ((ImageBuffer[i] == 0xFF) && (ImageBuffer[i + 1] == 0xD9)) {
			/* Found END of JPEG Frame */
			f_putc(ImageBuffer[i], &fsrc);
			f_putc(ImageBuffer[i + 1], &fsrc);
			break;
		}
		f_putc(ImageBuffer[i], &fsrc);
	}

	/*
	 written = f_puts ("HELLO WORLD", &fsrc);
	 if (written == -1) {
	 chprintf(chp, "FS: f_puts(\"Hello World\",\"hello.txt\") failed\r\n");
	 } else {
	 chprintf(chp, "FS: f_puts(\"Hello World\",\"hello.txt\") succeeded\r\n");
	 }
	 */
	/*
	 * Close the file
	 */
	f_close(&fsrc);
	palTogglePad(GPIOD, 13);
	chThdSleepMilliseconds(250); palTogglePad(GPIOD, 13);

	captured = 0;
}

static uint8_t AsciiToHex(char c) {
	if (c == '0')
		return 0;
	if (c == '1')
		return 1;
	if (c == '2')
		return 2;
	if (c == '3')
		return 3;
	if (c == '4')
		return 4;
	if (c == '5')
		return 5;
	if (c == '6')
		return 6;
	if (c == '7')
		return 7;
	if (c == '8')
		return 8;
	if (c == '9')
		return 9;
	if ((c == 'A') || (c == 'a'))
		return 10;
	if ((c == 'B') || (c == 'b'))
		return 11;
	if ((c == 'C') || (c == 'c'))
		return 12;
	if ((c == 'D') || (c == 'd'))
		return 13;
	if ((c == 'E') || (c == 'e'))
		return 14;
	if ((c == 'F') || (c == 'f'))
		return 15;
	return 127;
}
