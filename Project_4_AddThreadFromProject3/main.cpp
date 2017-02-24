#include "NimoCMultiThread/PnPThread.h"
#include "NimoCMultiThread/SocketThread.h"
#include "NimoCMultiThread/CMsgQueue.h"
#include "NimoCMultiThread/COperatingSystemFactory.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>


int main()
{
    CMsgQueue *q=COperatingSystemFactory::newMsgQueue("PoseDataMessage");

    PnPThread *a=new PnPThread("A");
    SocketThread *b=new SocketThread("B");

    a->setMsgQueue(q);
    b->setMsgQueue(q);

    a->run();
    b->run();

    sleep(40);


}
