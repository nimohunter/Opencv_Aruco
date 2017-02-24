#ifndef NIMOTHRED_H
#define NIMOTHRED_H

#include "CThread.h"


class NimoThread : public CThread
{
public:
    NimoThread(const char *m_name);
    ~NimoThread();
    virtual void mainLoop();
};

#endif // NIMOTHRED_H
