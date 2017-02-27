#include "SocketThread.h"
#include "COperatingSystemFactory.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <iostream>
#include <NimoPoseDetect/utils.h>


SocketThread::SocketThread(const char *m_name):
    CThread(m_name)
{

    //add your code here




}



SocketThread::~SocketThread()
{

}

void SocketThread::mainLoop()
{
    portno = 8042;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        printf("ERROR opening socket");
        return ;
    }
    server = gethostbyname(ServerIp);
    if (server == NULL) {
        printf("ERROR, no such host\n");
        exit(0);
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);

    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    {
        printf("ERROR connecting");
        return;
    }
    bzero(buffer,256);

//    char themessage[] = "{\"msg\":\"Pulse\"}";
//    int len = strlen(themessage);

//    int   a   =   len;
//    buffer[0]   =   (char)(a   &   0xff);
//    buffer[1]  =   (char)((a   >>   8)   &   0xff);
//    buffer[2]   =   (char)((a   >>   16)   &   0xff);
//    buffer[3]   =   (char)((a   >>   24)   &   0xff);

    int count = 0;

    unsigned int code;
    void *p_msg;
    char *send_msg;
    TransData *p_transdata;

    long lastSendTime = Utils::getCurrentTime();
    while (true)
    {
//        n = write(sockfd,buffer,4);
//        write(sockfd,themessage,strlen(themessage));
//        if (n < 0) {
//            printf("send ERROR\n");
//            //error("ERROR writing to socket");
//            break;
//        }
//        printf("send  4\n");
//        p_opration_system->sleepSec(1);
//        printf("send  5\n");
        p_msg_queue->recvMsg(code, p_msg);
        p_transdata = (TransData *)p_msg;

        send_msg =  p_transdata->getTransData();
        std::cout << "##out  [" << code << "] " <<  send_msg << std::endl;
        long nowTime = Utils::getCurrentTime();
        std::cout << "##out interval time = " << nowTime - lastSendTime << std::endl;
        lastSendTime = nowTime;

        int len = strlen(send_msg);

        buffer[0]   =   (char)(len   &   0xff);
        buffer[1]  =   (char)((len   >>   8)   &   0xff);
        buffer[2]   =   (char)((len  >>   16)   &   0xff);
        buffer[3]   =   (char)((len  >>   24)   &   0xff);

        write(sockfd,buffer,4);
        write(sockfd,send_msg,strlen(send_msg));


//        printf("<<<<<<<%s  is Running....recv data from message queue: code is : [%d]   data is : [%s] \n",p_thread_name,code,(char *)p_msg);
    }
}



void SocketThread::setMsgQueue(CMsgQueue *q)
{
    p_msg_queue = q;
}
