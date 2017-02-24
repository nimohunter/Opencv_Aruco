

#ifndef _Test_thread_b_h_
#define _Test_thread_b_h_


#include "CThread.h"
#include "CMsgQueue.h"
#include "PnPThread.h"
#include "transdata.h"


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

class SocketThread:public CThread
{

public:
    SocketThread(const char *m_name);
    ~SocketThread();

    virtual void mainLoop();

    void setMsgQueue(CMsgQueue *q);

private:
    CMsgQueue *p_msg_queue;;

    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];
    char *ServerIp = "192.168.189.1";


};




#endif
