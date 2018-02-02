#include "Client.h"

Client::Client(string host, int port_num) {
    client_fd=0;
    memset(&serv_addr, 0, sizeof(serv_addr));
    buffer= (char*)malloc(sizeof(char)*SOCKET_BUF_SIZE);

    if((client_fd = socket(AF_INET, SOCK_STREAM, 0))<0)// initializes socket of IP type and of reliable connection
        perror("ERROR CREATING SOCKET\n");

    in_addr_t IP = inet_addr(host.c_str());
    struct hostent *remote; 

    if (IP == INADDR_NONE)
    {
        if ((remote = gethostbyname (host.c_str())) == NULL) {
            perror("DNS\n");
        } else {
            memcpy ((char *)&(serv_addr.sin_addr), remote->h_addr, remote->h_length);
        }
    } else {
        serv_addr.sin_addr.s_addr = IP;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port_num);   

    if (connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        perror("Connection Failed\n");
        exit(0);
    }
}

Client::~Client() {
    close(client_fd);//closes open port
    delete buffer;
}


void Client::send_msg(string s) {
    // if (s.size()>SOCKET_BUF_SIZE)
    //     cout<<"ERROR MESSAGE SIZE TOO BIG\n";
    // else{
        if(send(client_fd, s.c_str(), s.size(), 0)<0)//0 means no flags
            cout<<"ERROR SENDING MESSAGE\n";
        // else
        // cout << "SENT " << s << "\n";
    // }
}

string Client::read_msg() {
    string msg;
    if(recv(client_fd, buffer, SOCKET_BUF_SIZE,0)<0)
        cout<<"ERROR READING MESSAGE\n";
    else{
        msg= string(buffer);

        //remove newline chars, mac, linux, and windows
        // char chars[] = {'\r', '\n'};

        // for (unsigned int i = 0; i < 2; ++i) {
        //     msg.erase(std::remove(msg.begin(), msg.end(), chars[i]), msg.end());
        // }
        //msg.replace(msg.end()-2,msg.end(),"");//replaces the newline char and return char with nothing
        memset(buffer,0,SOCKET_BUF_SIZE);//clear buffer
        // cout << "RESPONSE " << msg << "\n";
        return msg;
        }
    return string("error");
}