#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "MvCamera.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void static __stdcall ImageCallBack(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    void ImageCallBackInner(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInf);

private:
    void ShowErrorMsg(QString csMessage, int nErrorNum);
private slots:

    void on_bnEnum_clicked();

    void on_bnOpen_clicked();

    void on_bnClose_clicked();

    void on_bnContinuesMode_clicked();

    void on_bnTriggerMode_clicked();

    void on_bnStart_clicked();

    void on_bnStop_clicked();

    void on_cbSoftTrigger_clicked();

    void on_bnTriggerExec_clicked();

    void on_bnGetParam_clicked();

    void on_bnSetParam_clicked();

private:
    Ui::MainWindow *ui;

    void *m_hWnd;

    MV_CC_DEVICE_INFO_LIST  m_stDevList;
    CMvCamera*              m_pcMyCamera;
    bool                    m_bGrabbing;   // 是否开始抓图
};

#endif // MAINWINDOW_H
