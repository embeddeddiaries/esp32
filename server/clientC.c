// Client side C/C++ program to demonstrate Socket programming 
#include <stdio.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h> 
#define PORT 3333 

int main(int argc, char const *argv[]) 
{ 
	int sock = 0, valread; 
	struct sockaddr_in serv_addr; 
	//char *hello = "Hello from client"; 
	char hello = '1',option; 
	char buffer[1024] = {0}; 
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
	{ 
		printf("\n Socket creation error \n"); 
		return -1; 
	} 

	serv_addr.sin_family = AF_INET; 
	serv_addr.sin_port = htons(PORT); 
	
	// Convert IPv4 and IPv6 addresses from text to binary form 
//	if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) 
	if(inet_pton(AF_INET, "192.168.1.5", &serv_addr.sin_addr)<=0) 
	{ 
		printf("\nInvalid address/ Address not supported \n"); 
		return -1; 
	} 

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
	{ 
		printf("\nConnection Failed \n"); 
		return -1; 
	}

	do
	{
		hello = '8';
		printf("\nEnter on or off = ");
		scanf("%c",&hello); 
		send(sock , &hello , 1 , 0 ); 
		valread = read( sock , buffer, 1024); 
		printf("%s\n",buffer );
		printf("Do you want to continue = ");
		scanf("\n%c",&option); 
		getchar();
	}while(option == 'y' || option == 'Y');
	return 0; 
} 

