
#pragma once

#include <sys/socket.h> //includes for POSIX socket
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h> // include for string
#include <unistd.h>
#include <errno.h>
#include <string>
#include <sys/types.h>
#include <algorithm>
#include <netdb.h>

#define SOCKET_BUF_SIZE 1024
using namespace std;

class Client {

public:
	Client(string host, int port_num);
	void send_msg(string s);
	string read_msg();
	~Client();

private:
	int client_fd;
	sockaddr_in serv_addr;
	char *buffer;
};