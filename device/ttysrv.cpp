#include <sys/socket.h>
#include <sys/un.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
//#include <sched.h>
#include <set>
#include "ttysrv.h"

void ttysrv::start(bool alloc_terminal)
{
    if (alloc_terminal && !path.empty()) {
        sockaddr_un addr;
        std::string cmd = "utils/runttysrv ";

        cmd.append(path);
        ::system(cmd.c_str());

        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path));
        ifd = socket(AF_UNIX, SOCK_STREAM, 0);

	int ret;
        for (int retry = 0; retry < 10; retry++) {
            ret = connect(ifd, (const struct sockaddr *)&addr, sizeof(addr));
            if (ret >= 0)
                break;
            sleep(1);
            //sched_yield();
        }
	if (ret < 0) {
		fprintf(stderr, "Can't connect to ttysrv\n");
		exit(1);
	}

	ofd = ifd;
    } else if (tcgetattr(ifd, &t) >= 0) {
	struct termios raw;

	raw = t;
	raw.c_lflag &= ~(ECHO|ICANON);
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 0;
	tcsetattr(ifd, TCSAFLUSH, &raw);

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    }

    int flags = fcntl(ifd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(ifd, F_SETFL, flags | O_NONBLOCK);
    }
}

void ttysrv::stop(bool alloc_terminal)
{
    if (alloc_terminal && !path.empty()) {
        close(ifd);
	ifd = -1;
	ofd = -1;
    } else {
	setlinebuf(stdout);
	tcsetattr(0, TCSAFLUSH, &t);
    }
}

////////////

bool ttysrv::ready(int io)
{
	// currently output is always ready
	if (io == OUT)
		return true;

	if (inp.size()) {
		return true;
	} else {
		int nread = 0;

		ioctl(ifd, FIONREAD, &nread);
		return (nread != 0);
	}

	return false;
}

int ttysrv::in()
{
	unsigned char ch = 0;

	if (inp.size()) {
		ch = inp.front();
		inp.erase(0, 1);
	} else {
		int nread = 0;

		ioctl(ifd, FIONREAD, &nread);
		if (nread) {
			read(ifd, &ch, 1);
		}
	}

	return ch;
}

void ttysrv::out(int ch)
{
	write(ofd, &ch, 1);

	if (exp.length()) {
		if (ch == '\n') {
			outp = 0;
		} else if (outp < sizeof(outbuf)) {
			outbuf[outp++] = ch;
		}

		std::size_t exp_len = exp.length() - 1;

		if (outp == exp_len) {
			if (strncmp(exp.c_str() + 1, outbuf, exp_len) == 0) {
				expect_matched();
			}
		}
	}
}

bool ttysrv::hup()
{
	if (ready(IN))
		return false;

	struct stat st;

	fstat(ifd, &st);
	switch (st.st_mode & S_IFMT) {
	case S_IFREG:
		{
			off_t pos = lseek(ifd, 0, SEEK_CUR);
			return (pos >= st.st_size);
		}
	case S_IFIFO:
	case S_IFSOCK:
		{
			struct pollfd pfd = { ifd, POLLIN, 0 };
			poll(&pfd, 1, 0);
			if (pfd.revents & POLLHUP)
				return true;
			return false;
		}
	default:
		return false;
	}
}

void ttysrv::expect(const char *file)
{
	is.open(file, std::ios_base::in);
	expect_matched();
}

static bool is_debuggee()
{
	std::ifstream is("/proc/self/status");
	std::string s;
	while (is >> s) {
		if (s == "TracerPid:") {
			int pid;

			is >> pid;
			return pid != 0;
		}
		std::getline(is, s);
	}
	return false;
}

void ttysrv::expect_matched()
{
	const char *e;

	do {
		std::getline(is, exp);
		e = exp.c_str();
		if (*e == '>') {
			inp.append(exp);
			inp += '\n';
		} else if (*e == '*') {
			if (is_debuggee())
				asm("int3");
		} else if (*e == '<') {
			break;
		}
	} while (!is.eof());

	if (is.eof()) {
		is.close();
	}
}
