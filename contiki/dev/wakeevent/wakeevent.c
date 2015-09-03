#include "stdint.h"
#include "stdio.h"
#include "wakeevent.h"

struct wakeEvent* head = NULL;

void addEvent(struct wakeEvent* event) {
	if (head == NULL) {
		head = event;
	} else {
		struct wakeEvent* temp = head;
		while (temp->next != NULL) {
			temp = temp->next;
		}
		temp->next = event;
	}

	return;
}

struct wakeEvent* popEvent(void) {
	if (head == NULL) {
		return head;
	} 
	struct wakeEvent* temp = head;

	head = (head)->next;

	temp->next = NULL;

	return temp;
}

void printEventHead(void) {
	if (head == NULL) {
		printf("no head\n");
		return;
	}

	printf("head: %i\n", head->wake_type);
	
	return;
}
