#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <hw/inout.h>
#include <sys/mman.h>
#include <pthread.h>
#include <string.h>
#include <sys/syspage.h>


#include <termios.h>
#include <fcntl.h>
#include <sys/select.h>

// Define a string for TFC3
#define ControllerName "TFC3 Rail Crossing" // Local Name for the Rail Crossing

#define BCM2711_GPIO1_BASE 0xFE200000
#define BCM2711_GPIO_SIZE  0x00000100

#define GPFSEL0 0x00
#define GPSET0  0x1C
#define GPCLR0 0x28
#define GPLEV0 0x34
#define GPIO_PUP_PDN_CNTRL_REG0 0xE4
#define GPIO_PUP_PDN_CNTRL_REG1 0xE8

// ------------------- GPIO Enums -------------------
typedef enum {
	D0 = 17,
	D1 = 27,
	D2 = 22,
	D3 = 5,
	D4 = 6,
	D5 = 13,
	D6 = 19,
	D7 = 26,
	D8 = 23,
	D9 = 18,
	D10 = 24,
	D11 = 25,
	D12 = 12,
	D13 = 21,
} gpio_output_t;

typedef enum {
	BUTTON1 = D12, BUTTON2 = D1, BUTTON3 = D6,
} gpio_input_t;

// ------------------- Globals -------------------
uintptr_t gpio_base;
pthread_mutex_t gpio_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t interval_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t interval_cond   = PTHREAD_COND_INITIALIZER;
pthread_mutex_t debounce_mutex = PTHREAD_MUTEX_INITIALIZER;


pthread_mutex_t boom_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  boom_cond  = PTHREAD_COND_INITIALIZER;   // Trigger start
pthread_cond_t  boom_done_cond = PTHREAD_COND_INITIALIZER; // Sequence complete

volatile int BoomState = 0; // Trigger flag for Boom Gate
volatile int BoomError = 0;
volatile int GateDown = 0;

// Global trigger interval in seconds
volatile int TriggerInterval = 300; // How often the trains pass the intersection in seconds


// time to blink
int BlkTime = 100000; // microseconds

//button debounce time
int debounce = 100000; //button debounce 100000uS = 0.1s


// Button states
volatile int button1_state = 0;
volatile int button2_state = 0;
volatile int button3_state = 0;

// Trigger flag for rail lights
volatile int runRailLights = 0;



// ------------------- Light Step -------------------
typedef struct {
	const char *name;
	gpio_output_t pins[4];
	int num_pins;
	int cycles;
} LightStep;

// ------------------- Rail Lights Sequence -------------------
LightStep rail_lights[] = {
		{ "RL1", { D11, D10, D8, D2 }, 4, 10 },
		{ "RL2", { D0, D4   }, 2, 10 },
		{ "RL1", { D11, D10, D8, D2 }, 4, 10 },
		{ "RL2", { D0, D4   }, 2, 10 },
		{ "RL1", { D11, D10, D8, D2 }, 4, 10 },
		{ "RL2", { D0, D4   }, 2, 10 },
		{ "RL1", { D11, D10, D8, D2 }, 4, 10 },
		{ "RL2", { D0, D4   }, 2, 10 },
		{ "RL1", { D11, D10, D8, D2 }, 4, 10 },
		{ "RL2", { D0, D4   }, 2, 10 },
		{ "RL1", { D11, D10, D8, D2 }, 4, 10 },
		{ "RL2", { D0, D4   }, 2, 10 },
		{ "RL1", { D11, D10, D8, D2 }, 4, 10 },
		{ "RL2", { D0, D4   }, 2, 10 },
		{ "RL1", { D11, D10, D8, D2 }, 4, 10 },
		{ "RL2", { D0, D4   }, 2, 10 },
};
int rail_lights_steps = sizeof(rail_lights) / sizeof(rail_lights[0]);

// ------------------- BoomGate Sequence -------------------
LightStep boomgate_seq[] = {
		{ "BG_DOWN", {  D3 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", { D9 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", {  D3 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", { D9 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", {  D3 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", { D9 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", {  D3 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", { D9 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", {  D3 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", { D9 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", {  D3 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", { D9 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", {  D3 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", { D9 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", {  D3 }, 1, 5 }, // Book Gate Lights
		{ "BG_DOWN", { D9 }, 1, 5 }, // Book Gate Lights
	  //{ "BG_DOWN", { D9, D3 }, 2, 10 }, // Book Gate Lights

		};
int boomgate_steps = sizeof(boomgate_seq) / sizeof(boomgate_seq[0]);

// ------------------- GPIO Functions -------------------
static inline void gpio_set(int pin) {
    //printf("[GPIO] GPIO %d set HIGH\n", pin);  // dynamically print the pin number
	pthread_mutex_lock(&gpio_mutex);
	out32(gpio_base + GPSET0, 1 << pin);
	pthread_mutex_unlock(&gpio_mutex);
}

static inline void gpio_clear(int pin) {
   // printf("[GPIO] GPIO %d set LOW\n", pin);  // dynamically print the pin number
	pthread_mutex_lock(&gpio_mutex);
	out32(gpio_base + GPCLR0, 1 << pin);
	pthread_mutex_unlock(&gpio_mutex);
}

static inline int gpio_read(int pin) {
	pthread_mutex_lock(&gpio_mutex);
	int val = (in32(gpio_base + GPLEV0) & (1 << pin)) ? 1 : 0;
	pthread_mutex_unlock(&gpio_mutex);
	return val;
}

void setup_pin(int pin, int output) {
	pthread_mutex_lock(&gpio_mutex);
	uintptr_t fsel_reg = gpio_base + GPFSEL0 + ((pin / 10) * 4);
	int shift = (pin % 10) * 3;
	uint32_t val = in32(fsel_reg);
	val &= ~(0b111 << shift);
	if (output)
		val |= (0b001 << shift);
	out32(fsel_reg, val);
	pthread_mutex_unlock(&gpio_mutex);
}

void setup_pull(int pin, int setting) {
	pthread_mutex_lock(&gpio_mutex);
	int reg_offset = GPIO_PUP_PDN_CNTRL_REG0 + (pin / 16) * 4;
	int shift = (pin % 16) * 2;
	uint32_t val = in32(gpio_base + reg_offset);
	val &= ~(0b11 << shift);
	val |= (setting & 0b11) << shift;
	out32(gpio_base + reg_offset, val);
	pthread_mutex_unlock(&gpio_mutex);
}

// ------------------- Rail Lights Thread -------------------
void* rail_lights_thread(void* arg) {
	(void) arg;

	while (1) {
		if (runRailLights) {
			printf("[RailLights] Starting sequence\n");
			for (int s = 0; s < rail_lights_steps; s++) {

				LightStep step = rail_lights[s];

				for (int c = 0; c < step.cycles; c++) {

					for (int p = 0; p < step.num_pins; p++)
						gpio_set(step.pins[p]);
					usleep(BlkTime);
					for (int p = 0; p < step.num_pins; p++)
						gpio_clear(step.pins[p]);
				}
			}
			printf("[RailLights] Sequence complete\n");

			if (BoomError != 1) {
				runRailLights = 0;

				gpio_clear(D3);
				gpio_clear(D9);

			}
		}
		//usleep(10000);
	}
	return NULL;
}

// ------------------- BoomGate Monitor Thread -------------------
void* boomgate_monitor_thread(void* arg) {
	(void) arg;
	printf("[BoomGateMonitor] Initialised \n");
	// Run the sequence without holding the mutex
	for (int i = 0; i < 20; i++) {
		button3_state = !gpio_read(BUTTON3);
		if (button3_state && GateDown == 0) {
			printf("[BoomGateMonitor] Button3 Pressed \n");
			GateDown = 1;
			pthread_mutex_lock(&debounce_mutex); // start critical section,  Ensures only one button handler at a time can run its debounce section.
			usleep(debounce); //button debounce 100000uS = 0.1s
			pthread_mutex_unlock(&debounce_mutex); //end critical section
		}
		usleep(250000);
	}
	printf("[BoomGateMonitor] 5 seconds passed!\n");
	pthread_exit(NULL);   // Kill this thread cleanly
}


// ------------------- BoomGate Thread -------------------
void* boomgate_thread(void* arg) {
    (void)arg;
    printf("[BoomGate] Intialised\n");

    while (1) {
        // Wait for BoomGate trigger
        pthread_mutex_lock(&boom_mutex);
        while (BoomState == 0) {
            pthread_cond_wait(&boom_cond, &boom_mutex);
        }
        pthread_mutex_unlock(&boom_mutex);

			GateDown = 0;

        // Create threads names
        pthread_t boommonitor_thread;
        pthread_create(&boommonitor_thread, NULL, boomgate_monitor_thread, NULL);
        pthread_detach(boommonitor_thread); // don't block

        printf("[BoomGate] Starting sequence\n");

        // Run the sequence without holding the mutex
        for (int i = 0; i < 20; i++) {
            gpio_set(D3);
            gpio_set(D9);
            usleep(250000);

            gpio_clear(D3);
            gpio_clear(D9);
            usleep(250000);
        }

		// Sequence complete, lock only to update state & signal monitor
		pthread_mutex_lock(&boom_mutex);
		//printf(" Boom gate state = %d \n", GateDown);
		if (GateDown == 1) {
			printf("[BoomGate] Boomgate went down in time\n");
			BoomState = 0;
			BoomError = 0; //  no problem with intersection
			gpio_set(D3);
			gpio_set(D9);
		} else {
			printf("[BoomGate] Boomgate Error!, did not go down in time\n");
			BoomState = 1; //keep flashing
			BoomError = 1; // problem with intersection
		}
        pthread_mutex_unlock(&boom_mutex);
        printf("[BoomGate] Sequence complete\n");
    }
    return NULL;
}


// ------------------- Active Boomgastes and Lights Functions -------------------
void ActivateBoomAndLights(void) {
	pthread_mutex_lock(&boom_mutex);
	BoomState = 1; // set state to running
	pthread_cond_signal(&boom_cond); // wake the Boomgate thread
	pthread_mutex_unlock(&boom_mutex);
	//set_trigger_interval(30);
	pthread_mutex_lock(&debounce_mutex); // start critical section,  Ensures only one button handler at a time can run its debounce section.
	runRailLights = 1;
}


// ------------------- Button Monitor Polling Thread -------------------
void* button_monitor(void* arg) {
	(void) arg;
	while (1) {
		button1_state = !gpio_read(BUTTON1);
		button2_state = !gpio_read(BUTTON2);
		button3_state = !gpio_read(BUTTON3);

		if (button1_state && runRailLights == 0) {
			printf("[Button1] Pressed \n");
			ActivateBoomAndLights(); // activate the boomgates and lights
			pthread_mutex_lock(&debounce_mutex); // start critical section,  Ensures only one button handler at a time can run its debounce section.
			usleep(debounce); //button debounce 100000uS = 0.1s
			pthread_mutex_unlock(&debounce_mutex); //end critical section
		}
		if (button2_state && BoomState == 0) {
			printf("[Button2] Pressed \n");
			ActivateBoomAndLights(); // activate the boomgates and lights
			/*
			 pthread_mutex_lock(&boom_mutex);
			 BoomState = 1; // set state to running
			 pthread_cond_signal(&boom_cond); // wake the Boomgate thread
			 pthread_mutex_unlock(&boom_mutex);
			 */
			//set_trigger_interval(30);
			pthread_mutex_lock(&debounce_mutex); // start critical section,  Ensures only one button handler at a time can run its debounce section.
			usleep(debounce); //button debounce 100000uS = 0.1s
			pthread_mutex_unlock(&debounce_mutex); //end critical section
		}
		/*
		 if (button3_state && GateDown == 0) {
		 printf("[Button3] Pressed \n");
		 pthread_mutex_lock(&boom_mutex);
		 if (BoomState == 1) { // set state to running
		 GateDown = 1;
		 pthread_cond_signal(&boom_done_cond); // notify monitor
		 }
		 pthread_mutex_lock(&debounce_mutex); // start critical section,  Ensures only one button handler at a time can run its debounce section.
		 usleep(debounce); //button debounce 100000uS = 0.1s
		 pthread_mutex_unlock(&debounce_mutex); //end critical section
		 }
		 */
		usleep(10000);
	}
	return NULL;
}

// ------------------- Get Interval Trigger Thread -------------------
/*
 * function gets trigger interval in seconds
 */
int get_trigger_interval(void) {
    pthread_mutex_lock(&interval_mutex);
    int val = TriggerInterval;
    pthread_mutex_unlock(&interval_mutex);
    return val;
}

// ------------------- Update Interval Trigger Thread -------------------
/*
 * function sets new trigger interval in seconds
 */
void set_trigger_interval(int newval) {
    pthread_mutex_lock(&interval_mutex);
    TriggerInterval = newval;
    pthread_cond_signal(&interval_cond);  // wake the periodic thread
    pthread_mutex_unlock(&interval_mutex);
}

// ------------------- Period Trigger Thread -------------------
void* periodic_trigger_thread(void* arg) {
    (void)arg;
    int interval;

    while (1) {
        pthread_mutex_lock(&interval_mutex);
    	interval = TriggerInterval;
        int minutes = interval / 60;
        int seconds = interval % 60;
        printf("[PeriodicTrigger] Next scheduled train will be in %d minute%s %d second%s\n",
              minutes, (minutes == 1 ? "" : "s"),
               seconds, (seconds == 1 ? "" : "s"));

        // Wait for interval time, OR until someone signals an update
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);

        ts.tv_sec += interval;
        int rc = pthread_cond_timedwait(&interval_cond, &interval_mutex, &ts);
        pthread_mutex_unlock(&interval_mutex);


        if (rc == ETIMEDOUT) {
            // Time expired -> trigger event
            printf("[PeriodicTrigger] Activating sequences\n");
            ActivateBoomAndLights();
            //runRailLights = 1;
            //BoomState = 1;
        } else {
            // Condition was signalled -> interval changed, loop and re-check
            printf("[PeriodicTrigger] Interval updated, recalculating...\n");
        }
    }
    return NULL;
}

// ------------------- Terminal Control Thread -------------------
void* terminal_thread(void* arg) {
    (void)arg;
    char cmd[128];

    // Put stdin in non-canonical mode
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);   // disable line buffering + echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Make stdin non-blocking
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    printf("[Terminal] Ready (non-blocking). Type 'help' for commands.\n");

    while (1) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // check every 100ms

        int retval = select(STDIN_FILENO+1, &readfds, NULL, NULL, &tv);

        if (retval > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
            int len = read(STDIN_FILENO, cmd, sizeof(cmd)-1);
            if (len > 0) {
                cmd[len] = '\0';

                // Strip newline if present
                cmd[strcspn(cmd, "\n")] = 0;

                if (strncmp(cmd, "help", 4) == 0) {
                    printf("Commands: boomstate, boomerror, gatedown, b1, b2, b3, interval, activate, status, quit\n");
                }
                else if (strncmp(cmd, "boomstate", 9) == 0) {
                    BoomState = atoi(cmd + 10);
                    printf("[Terminal] BoomState set to %d\n", BoomState);
                }
                else if (strncmp(cmd, "boomerror", 9) == 0) {
                    BoomError = atoi(cmd + 10);
                    printf("[Terminal] BoomError set to %d\n", BoomError);
                }
                else if (strncmp(cmd, "gatedown", 8) == 0) {
                    GateDown = atoi(cmd + 9);
                    printf("[Terminal] GateDown set to %d\n", GateDown);
                }
                else if (strncmp(cmd, "b1", 2) == 0) {
                    button1_state = atoi(cmd + 3);
                    printf("[Terminal] Button1_state set to %d\n", button1_state);
                }
                else if (strncmp(cmd, "b2", 2) == 0) {
                    button2_state = atoi(cmd + 3);
                    printf("[Terminal] Button2_state set to %d\n", button2_state);
                }
                else if (strncmp(cmd, "b3", 2) == 0) {
                    button3_state = atoi(cmd + 3);
                    printf("[Terminal] Button3_state set to %d\n", button3_state);
                }
                else if (strncmp(cmd, "interval", 8) == 0) {
                    int newval = atoi(cmd + 9);
                    if (newval > 0) {
                        set_trigger_interval(newval);
                        printf("[Terminal] TriggerInterval set to %d seconds\n", newval);
                    }
                }
                else if (strncmp(cmd, "activate", 8) == 0) {
                    printf("[Terminal] Activating boom gate and lights...\n");
                    ActivateBoomAndLights();
                }
                else if (strncmp(cmd, "status", 6) == 0) {
                    printf("[Terminal] BoomState=%d BoomError=%d GateDown=%d b1=%d b2=%d b3=%d Interval=%d\n",
                           BoomState, BoomError, GateDown,
                           button1_state, button2_state, button3_state,
                           get_trigger_interval());
                }
                else if (strncmp(cmd, "quit", 4) == 0) {
                    printf("[Terminal] Exiting...\n");
                    // Restore terminal before exit
                    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
                    exit(0);
                }
                else {
                    printf("[Terminal] Unknown command: %s\n", cmd);
                }
            }
        }

        // Keep looping, non-blocking
        usleep(50000); // small sleep to reduce CPU
    }

    // Restore terminal settings if thread ends
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return NULL;
}



// ------------------- Startup -------------------
void StartUP(void) {
printf("\nName: Nick Owen, Student Number: S3051886\n");
int num_cpus = _syspage_ptr->num_cpu;
printf("--> CPUs available: %d\n", num_cpus);
printf("--> PID: %d\n", getpid());
char hostnm[100];
memset(hostnm, '\0', 100);
gethostname(hostnm, sizeof(hostnm));
printf("--> Hostname: '%s'\n", hostnm);
printf("--> Controller Name: '%s'\n", ControllerName);
sleep(1);
}

// ------------------- Main -------------------
int main(void) {
printf("Thread-safe GPIO controller with BoomGate sequence\n");
StartUP();

gpio_base = mmap_device_io(BCM2711_GPIO_SIZE, BCM2711_GPIO1_BASE);
if (!gpio_base) {
	perror("Can't map GPIO");
	return 1;
}

// Output pins
gpio_output_t outputs[] = { D0, D3, D4, D6, D8, D9, D10, D11 };
for (int i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++)
	setup_pin(outputs[i], 1);

// Input pins
gpio_input_t inputs[] = { BUTTON1, BUTTON2, BUTTON3 };
for (int i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
	setup_pin(inputs[i], 0);
	setup_pull(inputs[i], 1); // pull-up
}


// Clear outputs
for (int i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++)
	gpio_clear(outputs[i]);

// Create threads names
pthread_t btn_thread, rail_thread, boom_thread, periodic_thread, term_thread;

pthread_create(&term_thread, NULL, terminal_thread, NULL);
pthread_detach(term_thread);

pthread_create(&rail_thread, NULL, rail_lights_thread, NULL);
pthread_detach(rail_thread); // don't block

pthread_create(&boom_thread, NULL, boomgate_thread, NULL);
pthread_detach(boom_thread); // don't block

pthread_create(&periodic_thread, NULL, periodic_trigger_thread, NULL);
pthread_detach(periodic_thread);

// Wait forever
pthread_create(&btn_thread, NULL, button_monitor, NULL);
pthread_join(btn_thread, NULL);


munmap_device_io(gpio_base, BCM2711_GPIO_SIZE);
return 0;

}
