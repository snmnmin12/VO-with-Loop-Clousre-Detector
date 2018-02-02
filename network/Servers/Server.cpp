#include "Server.h"
#include <sys/time.h>
#include <sys/select.h>

Server::Server(int port_num) {
    server_fd=0;
    client_fd=0;
    memset(&client_addr, 0, sizeof(client_addr));
    memset(&server_addr, 0, sizeof (server_addr));//initialize all structs to 0

    buffer= (char*)malloc(sizeof(char)*SOCKET_BUF_SIZE);

    if((server_fd = socket(AF_INET, SOCK_STREAM, 0))<0)// initializes socket of IP type and of reliable connection
        printf("ERROR CREATING SOCKET\n");
    bool yes=true;
    setsockopt(server_fd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int)); //sets option on socket api that allows kernel to reuse sockets that are still in memory

    server_addr.sin_family = AF_INET; //AF_INET is the family of ip address, for our case an IPV4 address
    //sets server_addr to
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);// htonl is network long, format is in network byte order
    server_addr.sin_port = htons(port_num);//htons is host to network short, the format of the short is in network byte order

    bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr));//bind binds the address info to the socket
    //listen sets port to listen for client connections
    // 1 allows only one client machine to be connected on the port
    if (listen(server_fd, 1) == -1) {
        perror("listen");
        exit(1);
    }
    socklen_t client_addr_size=sizeof(client_addr);
    client_fd = accept(server_fd, (struct sockaddr*)&client_addr,&client_addr_size); //completes connection with client

    if (client_fd==-1)
        printf("ERROR IN ACCEPT\n");
}

Server::~Server() {
    close(server_fd);//close client connection
    close(client_fd);//closes open port
    free(buffer);
}


void Server::send_msg(std::string s) {
    if (s.size()>SOCKET_BUF_SIZE)
        printf("ERROR MESSAGE SIZE TOO BIG\n");
    else{
        if(send(client_fd, s.c_str(), s.size(), 0)<0)//0 means no flags
        printf("ERROR SENDING MESSAGE\n");
    }
}

std::string Server::read_msg() {

    std::string msg;

    memset(buffer, 0, sizeof(char)*SOCKET_BUF_SIZE);

    if(recv(client_fd, buffer, SOCKET_BUF_SIZE,0)<0)
        printf("ERROR READING MESSAGE\n");
    else{
        msg= std::string(buffer);
        return msg;
    }

    return std::string("error");
}