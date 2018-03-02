#ifndef __CAMERA_CALIBRATOR__
#define __CAMERA_CALIBRATOR__

#include <QWidget>
#include <QLineEdit>
#include <vector>
#include "opencv2/core.hpp"
#include "mvg/stereo_compute.h"


namespace Ui{
  class CameraCalibrator;
}

class CameraCalibrator : public QWidget{
  Q_OBJECT

  public:
    explicit CameraCalibrator(QWidget *parent = 0);
    ~CameraCalibrator();

  private slots:
    void SingleCamSetup();
    void SetCalPhotoDir();
    void RunCalibration(const bool only_rectification = false);
    void RunRectifiction();
    void ShowCalImages();
    void NextCalImage();
    void PrevCalImage();
    void UpdateFindTargetOptions(QString);
    void SetCalImg(int);
    void LoadStereoParameters();
    void UseIntGuessStateChange();
    void IntFromFileStateChange();
    void MarkAsLemon();

  private:
    Ui::CameraCalibrator *ui;
    QString m_default_cal_img_dir;
    cv::Size m_image_size;
    bool m_use_opencv_rectification;

    stereoCalData_t m_cal_data;
    std::vector<cv::Mat> m_cal_img_vec, m_rect_cal_img_vec;
    
    QLabel *m_label;
    QLineEdit *m_disp_img_line_edit;
    cv::Mat m_disp_cv_img;
    QImage m_disp_qt_img;
    int m_disp_img_idx;

    int GetCalibrationFlags();
    int GetFindTargetFlags(const std::string targetType);
    void SetCalImg();
    void SetFindTargetOptions(const bool, const bool, const bool, const bool, const bool);
};

#endif //__CAMERA_CALIBRATOR__

