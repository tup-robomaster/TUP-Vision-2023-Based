
#ifndef     __MV_PTHREAD_H__
#define     __MV_PTHREAD_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef struct
{
    void*       pTask;
}MV_TASK;

extern int MV_KThreadRun(MV_TASK* aTask, 
                         int (*threadfn)(void *data),
                         void *data,
                         const char name[]);

extern int MV_KThreadBind(MV_TASK* aTask, unsigned int cpu);

extern int MV_KTkthreadStop(MV_TASK* aTask);

extern int MV_KThreadShouldStop(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  // __MV_PTHREAD_H__
