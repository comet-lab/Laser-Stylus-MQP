#include <iostream>
#include <string>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>

#define PORT     5005
#define MAXLINE 1024  //Maximum number of bytes in a message received

class UDPServer {
private:
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;

public:
    UDPServer(const std::string& host = "", int port = PORT) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd == -1) {
            throw std::runtime_error("Failed to create socket");
        }

        std::memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);
        servaddr.sin_addr.s_addr = INADDR_ANY;
        if ( bind(sockfd, (const struct sockaddr *)&servaddr, 
            sizeof(servaddr)) < 0 )
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
    }

    void waitForRequest(char * receiveBuffer, double * message, int messageLength) {
        int n;
        socklen_t len;
        len = sizeof(cliaddr);

        // Receive data from a client
        ssize_t receivedBytes = recvfrom(sockfd, (char *)receiveBuffer, MAXLINE, 0,
                                             (struct sockaddr*)&cliaddr, &len);
        if (receivedBytes < 0) {
            std::cerr << "Failed to receive data from client." << std::endl;
        }
       
        ssize_t bytes_sent = sendto(sockfd, (double *)message, messageLength,
                                     MSG_CONFIRM, (const struct sockaddr *) &cliaddr, 
                                    len);
        if (bytes_sent == -1) {
            throw std::runtime_error("Failed to send message");
        } 
        // else{
        //     std::cout << "Sending Message to Server" << std::endl;
        // }

        // receiveBuffer[MAXLINE-1] = '\0'; //This is useful to terminate strings for printing
        // std::cout<<"Server : "<< receiveBuffer[0] <<std::endl; //verifying data received from server
        // std::cout<<"Server : "<< receiveBuffer[1] <<std::endl;
        // std::cout<<"Server : "<< receiveBuffer[2] <<std::endl;
    }

    void processData(double * receiveBuffer, double * posTarget, double * orienTarget){}

    ~UDPServer() { 
        close(sockfd);
    }
};
