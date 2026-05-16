#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <cstdlib>
using namespace std;    
 int main(){
    int listening = socket(AF_INET, SOCK_STREAM,0);
    if (listening == -1){
        cerr << "Can't create a socket!\n";
        return -1;
    }
    srand(time(NULL));
    int rand_port = rand() % (65535 - 49152 + 1) + 49152;
    cout << "Port: " << rand_port << "\n";
    sockaddr_in hint;
    hint.sin_family = AF_INET; // IPv4 family
    hint.sin_port = htons(rand_port);
    inet_pton(AF_INET,"0.0.0.0",&hint.sin_addr);
    
    
    if (bind(listening, (sockaddr*) &hint, sizeof(hint)) == -1)
    {
        cerr << "Can't bind to IP/port\n";
        return -2;
    }

    if (listen(listening, SOMAXCONN) == -1)
    {
        cerr << "Can't listen\n";
        return -3;
    }

    sockaddr_in client;
    socklen_t clientSize = sizeof(client);
    char host[NI_MAXHOST];
    char svc[NI_MAXSERV];

    int clientSocket = accept(listening, (sockaddr*)&client, &clientSize);
    if (clientSocket == -1)
    {
        cerr << "Problem with client connecting.\n";
        return -4;
    }
    close(listening);

    memset(host,0,NI_MAXHOST);
    memset(svc,0,NI_MAXSERV);

    int result = getnameinfo((sockaddr*)&client, clientSize, host,NI_MAXHOST,svc,NI_MAXSERV,0);
    if (!result)
    {
        cout << host << " connected on " << svc << "\n"; 
    }
    else
    {
        inet_ntop(AF_INET,&client.sin_addr,host,NI_MAXHOST);
        cout << host << " connected on " << ntohs(client.sin_port) << "\n";
    }

    char buf[4096];

    while (true)
    {
        memset(buf, 0,4096);

        int bytesRecv = recv(clientSocket,buf,4096,0);
        if (bytesRecv ==  -1)
        {
            cerr << "There was a connection issue\n";
            break;
        }
        if (bytesRecv == 0)
        {
            cerr << "Client disconnected.\n";
            break;
        }

        cout << "Recieved: " << string(buf,0,bytesRecv) << "\n";

        send(clientSocket, buf , bytesRecv + 1, 0);
    }
    close(clientSocket);
 }