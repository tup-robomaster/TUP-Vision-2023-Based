
#ifndef __EVENT_H__
#define __EVENT_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus


typedef struct
{
    void*            pEvent;
    unsigned int     Flag;
}
MV_EVENT;

/*extern int MV_EventCheck();*/
extern int MV_EventInit( MV_EVENT* aEvent );
extern int MV_EventFree( MV_EVENT* aEvent );
extern int MV_EventWait( MV_EVENT* aEvent );
extern int MV_EventWaitWithTimeout( MV_EVENT* aEvent, unsigned long long aTimeout );
extern int MV_EventRaise( MV_EVENT* aEvent );
extern int MV_EventReset( MV_EVENT* aEvent );

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __EVENT_H__
