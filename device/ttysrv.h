#ifndef	_TTYSERV_H_
#define	_TTYSERV_H_

#include <termios.h>
#include <fstream>
#include <string>

struct ttysrv {
	int ifd = 0;	// stdin
	int ofd = 1;	// stdout
	std::string path;

	enum {
		IN = 0,
		OUT = 1,
	};

	virtual void start(bool alloc_terminal);
	virtual void stop(bool alloc_terminal);
	bool ready(int io);
	int in();
	void out(int ch);
	bool hup();

	// expect
	char outbuf[1024];
	int outp;

	std::fstream is;
	std::string exp;
	std::string inp;

	// for tty input
	struct termios	t;

	void expect(const char *file);
	void expect_matched();
};

#endif	// _TTYSERV_H_
