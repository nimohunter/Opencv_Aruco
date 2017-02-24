
#ifndef _TestThread_h_
#define _TestThread_h_


#include "CThread.h"
#include "CMsgQueue.h"
#include "transdata.h"

class PnPThread:public CThread
{

public:
    PnPThread(const char *m_name);
    ~PnPThread();

    virtual void mainLoop();


    void setMsgQueue(CMsgQueue *q);

private:
    CMsgQueue *p_msg_queue;




};





#endif
