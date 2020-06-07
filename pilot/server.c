#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <netdb.h>

#include <pthread.h>

#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include "conversion.h"
#include "server.h"

extern char *dataP;
extern char *cmdP;
extern pthread_mutex_t mutexTcp;

void error(char *msg)
{
//    perror(msg);
//    exit(1);
}

void *server() {

	int sockfd, newsockfd, portno, clilen;
	char buffer[256];
	char *bufferP = buffer;
	struct sockaddr_in serv_addr, cli_addr;
	int n;

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) error("ERROR opening socket");
	  
	bzero((char *) &serv_addr, sizeof(serv_addr));
	memset((char *) &serv_addr, 0, sizeof(serv_addr));
	portno = 37377;
	
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);
	
	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) error("ERROR on binding");
	printf("Socket bound!!!\n");
	
	for(;;) {
		listen(sockfd,5);
		clilen = sizeof(cli_addr);
		newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t *) &clilen);
	
		if (newsockfd < 0) 
		error("ERROR on accept");
	
		bzero(buffer,256); // Read into buffer
		while( (n = read(newsockfd,buffer,255)) > 0)	{ 		// n=0 client disconnected
		        //printf("%s \r\n", buffer);

			if (n >= 0) {
				//	error("ERROR reading from socket");
				//	//return 0;
				//}
		
				// Split buffer in command and data part
				char token[25] = "";
				char *tokenP = token;
				tokenP = strtok(bufferP, " ");
				//printf("Here is the message: %s\r\n",buffer);
	
				// Check for quit message
/*				if (strcmpNS(tokenP, "q") == 0) {
					close(newsockfd);
					close(sockfd);
					system("poweroff");
					return 0; 
				}
*/		
				//Check for data request
				if(strcmpNS(tokenP, "$GET") == 0) {
					pthread_mutex_lock(&mutexTcp);					// Read global data thread safe
					n = write(newsockfd,dataP, strlen(dataP));			// and send to client
					pthread_mutex_unlock(&mutexTcp);

                                        //printf("Sent:: %s\r\n",dataP);
					if (n < 0) error("ERROR writing to socket");
	
				}
		
				//Check for set command
				if(strcmpNS(tokenP, "$SET") == 0) {
				
					// Find the data part
					tokenP = strtok(NULL, " ");
			
					//printf("Here is the message: %s\r\n",tokenP);

                                        if (tokenP != NULL) {	
					        pthread_mutex_lock(&mutexTcp);					// Write global data thread safe
						        strcpy(cmdP, tokenP);
					        pthread_mutex_unlock(&mutexTcp);
	                                }
				}
			}		
		}
	}
}
