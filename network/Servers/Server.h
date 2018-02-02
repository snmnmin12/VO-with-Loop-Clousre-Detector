
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

#define SOCKET_BUF_SIZE 20*1024

class Server {

public:
	Server(int port_num);
	void send_msg(std::string s);
	std::string read_msg();
	~Server();

private:
	int client_fd;
	int server_fd;
	sockaddr_in server_addr;
	sockaddr_in client_addr;
	char *buffer;

};