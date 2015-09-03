#ifndef WAKEEVENT_H
#define WAKEEVENT_H

typedef enum wake_t { 
			DEFAULTEV, 
			PERIODIC_EV,
			ACCEL_EV, 
			PIR_EV 
} wake_t;

typedef struct wakeEvent {
	wake_t wake_type;
	struct wakeEvent* next;
} wakeEvent;

void addEvent(struct wakeEvent* event);
struct wakeEvent* popEvent(void);
void printEventHead(void);

#endif /* WAKEEVENT_H */
