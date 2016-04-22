#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include <fcntl.h>
#include<unistd.h>

void initHTTP(void);
int pollHTTP();
char readBuffer[2000], message[255];

struct sockaddr_in sa;
struct hostent *server;
int socketfd;

int main(void) {

	printf("Starting main.\n");
	initHTTP();
	
	pollHTTP();
	
	close(socketfd);
	return 0;
}


void initHTTP(void) {
	
	socketfd = socket(AF_INET, SOCK_STREAM, 0);
	printf("Socket fd: %d\n", socketfd);
	
	//server = gethostbyname("exploringbeaglebone.com");
	
	bzero((char *) &sa, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons(40000);
	//sa.sin_port = htons(80);
	inet_pton(AF_INET, "192.168.1.33", &sa.sin_addr.s_addr);
	//bcopy((char *)server->h_addr, (char *)&sa.sin_addr.s_addr, server->h_length);
	
	int response = connect(socketfd, (struct sockaddr *) &sa, sizeof(sa));
	printf("Connect response: %d\n", response);
}


int pollHTTP() {

	sprintf(message, "GET /index.html HTTP/1.1\r\nHost: 192.168.1.33:40000\r\n\r\n\0");
	printf("%s\nLength: %d\n", message, strlen(message));
	int response = write(socketfd, message, strlen(message));
	printf("Write response: %d\n", response);
	
	response = 1;
	while (response>0) {
		response = read(socketfd, readBuffer, sizeof(readBuffer));
		printf("Read response: %d\n", response);
		printf("Read: %s\n", readBuffer);
	}


	return 1;
}
