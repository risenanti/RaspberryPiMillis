#ifndef millis_hpp
#define millis_hpp
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

static long long int count;

long long int millis(void)
{
	return count;
}

void createMillisTimer(void)
{
	struct sigaction sa;
	struct itimerval timer;

	static const auto increment = [&count] (int signum) {++count;};

	/* Install timer_handler as the signal handler for SIGVTALRM. */
	memset (&sa, 0, sizeof (sa));
	sa.sa_handler = increment;
	sigaction (SIGVTALRM, &sa, NULL);

	/* Configure the timer to expire after 1 msec... */
	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = 1000;
	/* ... and every 1 msec after that. */
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = 1000;
	/* Start a virtual timer. It counts down whenever this process is
	  executing. */
	setitimer (ITIMER_VIRTUAL, &timer, NULL);
}
#endif
