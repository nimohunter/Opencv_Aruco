#include "nimothred.h"

#include <cstdio>
#include <unistd.h>

NimoThread::NimoThread(const char *m_name):
    CThread(m_name)
{

    //add your code here
}

NimoThread::~NimoThread()
{

}

void NimoThread::mainLoop()
{
    printf("%s :init\n",p_thread_name);
    while (true)
    {
        printf("%s :hello world\n",p_thread_name);
        sleep(1);
    }
}
