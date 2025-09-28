#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <hw/inout.h>
#include <sys/mman.h>
#include <pthread.h>
#include <string.h>
#include <sys/syspage.h>

// Define a string for TFC3
#define ControllerName "TFC3"

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
int BlkTime = 100000; // microseconds

// Button states
volatile int button1_state = 0;
volatile int button2_state = 0;
volatile int button3_state = 0;

// Trigger flag for rail lights
volatile int runRailLights = 0;

// Trigger flag for Boom Gate
volatile int BoomState=0;

// Global trigger interval in seconds
volatile int TriggerInterval = 120; // default 2 minutes
pthread_mutex_t interval_mutex = PTHREAD_MUTEX_INITIALIZER;


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
	pthread_mutex_lock(&gpio_mutex);
	out32(gpio_base + GPSET0, 1 << pin);
	pthread_mutex_unlock(&gpio_mutex);
}

static inline void gpio_clear(int pin) {
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
			runRailLights = 0;
		}
		usleep(10000);
	}
	return NULL;
}

// ------------------- BoomGate Thread -------------------
void* boomgate_thread(void* arg) {
		(void) arg;
		while (1) {
			if (BoomState) {
				printf("[BoomGate] Starting sequence\n");
				for (int s = 0; s < boomgate_steps; s++) {
					LightStep step = boomgate_seq[s];
					for (int c = 0; c < step.cycles; c++) {
						for (int p = 0; p < step.num_pins; p++)
							gpio_set(step.pins[p]);
						usleep(BlkTime);
						for (int p = 0; p < step.num_pins; p++)
							gpio_clear(step.pins[p]);
					}
				}
				printf("[BoomGate] Sequence complete\n");
				BoomState = 0;
			}
			usleep(10000);
		}
		return NULL;
	}

// ------------------- Button Monitor Thread -------------------
void* button_monitor(void* arg) {
	(void) arg;
	while (1) {
		button1_state = !gpio_read(BUTTON1);
		button2_state = !gpio_read(BUTTON2);
		button3_state = !gpio_read(BUTTON3);

		if (button1_state && runRailLights == 0) {
			printf("[Button1] Pressed \n");
			runRailLights = 1;
			usleep(100000); //button debounce 100000uS = 0.1s

		}
		if (button2_state && BoomState == 0) {
			printf("[Button2] Pressed \n");
			BoomState = 1;
			usleep(100000); //button debounce 100000uS = 0.1s
		}

		usleep(10000);
	}
	return NULL;

}

// ------------------- Period Trigger Thread -------------------
void* periodic_trigger_thread(void* arg) {
    (void)arg;
    while (1) {
        printf("[PeriodicTrigger] Activating sequences\n");
        runRailLights = 1;
        BoomState = 1;

        // Sleep for the current interval
        int interval = get_trigger_interval();
        sleep(interval);
    }
    return NULL;
}

// ------------------- Set New Trigger Time -------------------
void set_trigger_interval(int new_interval) {
    pthread_mutex_lock(&interval_mutex);
    TriggerInterval = new_interval;
    pthread_mutex_unlock(&interval_mutex);
}

// ------------------- Get Trigger Time -------------------
int get_trigger_interval() {
    pthread_mutex_lock(&interval_mutex);
    int interval = TriggerInterval;
    pthread_mutex_unlock(&interval_mutex);
    return interval;
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
gpio_output_t outputs[] = { D0, D4, D6, D8, D10, D11 };
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

// Create threads
pthread_t btn_thread, rail_thread, boom_thread, periodic_thread;

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
