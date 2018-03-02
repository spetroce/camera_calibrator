#include "camera_calibrator.h"
#include "ui_camera_calibrator.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <QFileDialog>
#include <QStyleFactory>
#include <iostream>
#include <string>
#include <vector>
#include "mio/altro/io.h"
#include "mio/qt/cv_mat_to_qimage.h"

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xresource.h>


void get_ppi(float &x_pix_per_length, float &y_pix_per_length, bool in_inch = false){
  char *display_name = NULL;
  int screen_number = 0; //screen index
  Display *display = XOpenDisplay(display_name);

  x_pix_per_length = static_cast<float>( DisplayWidth(display, screen_number) ) /
                     static_cast<float>( DisplayWidthMM(display, screen_number) ),
  y_pix_per_length = static_cast<float>( DisplayHeight(display, screen_number) ) /
                     static_cast<float>( DisplayHeightMM(display, screen_number) );
  if(in_inch){
    x_pix_per_length /= 25.4f;
    y_pix_per_length /= 25.4f;
  }

  XCloseDisplay(display);
}

void DrawPoints(const std::vector<cv::Point2f> &point_vec, cv::Mat &dst, int radius = 2,  
                 cv::Scalar color = cv::Scalar::all(0), int thickness = -1){
  for(size_t i = 0; i < point_vec.size(); i++)
    cv::circle(dst, point_vec[i], radius, color, thickness);
}

void DisplayCalTarget(const std::vector<cv::Point3f> &point_vec){
  float ppmm_x, ppmm_y;
  get_ppi(ppmm_x, ppmm_y);
  const float ppmm = (ppmm_x + ppmm_y) / 2;
  printf("ppmm: %f\n", ppmm);

  std::vector<cv::Point2f> vecPoints2f( point_vec.size() );
  for(size_t i = 0; i < vecPoints2f.size(); i++) //' + (10*ppmm)' to space target 1cm from border
    vecPoints2f[i] = cv::Point2f( point_vec[i].x*ppmm + (10*ppmm), point_vec[i].y*ppmm + (10*ppmm) );

  cv::Size2f max; //search for target dimensions as a sanity check (as opposed to using target dim/spacing)
  for(size_t i = 0; i < vecPoints2f.size(); i++){
    if(vecPoints2f[i].y > max.height) max.height = vecPoints2f[i].y;
    if(vecPoints2f[i].x > max.width) max.width = vecPoints2f[i].x;
  }

  if(true){ //print coordinates
    const size_t w = static_cast<size_t>( cvRound(max.width) );
    for(size_t i = 0; i < point_vec.size(); i++){
      if( !(i % w) )
        std::cout << std::endl;
      std::cout << point_vec[i] << "  ";
    }
    printf("\n");
    for(size_t i = 0; i < vecPoints2f.size(); i++){
      if( !(i % w) )
        std::cout << std::endl;
      std::cout << vecPoints2f[i] << "  ";
    }
  }

  cv::Mat calTarget( max.height + (10*ppmm), max.width + (10*ppmm), CV_8UC3, cv::Scalar::all(255) );
  DrawPoints(vecPoints2f, calTarget);
  //cv::imshow("calibration target", calTarget);
}


CameraCalibrator::CameraCalibrator(QWidget *parent) : QWidget(parent), ui(new Ui::CameraCalibrator), 
    m_default_cal_img_dir("/home"), 
    m_use_opencv_rectification(true){
  ui->setupUi(this);
  m_label = NULL;

  connect( ui->pushButton_setCalPhotoDir, SIGNAL( clicked() ), this, SLOT( SetCalPhotoDir() ) );
  connect( ui->pushButton_runCalibration, SIGNAL( clicked() ), this, SLOT( RunCalibration() ) );
  connect( ui->pushButton_runRectification, SIGNAL( clicked() ), this, SLOT( RunRectifiction() ) );
  connect( ui->pushButton_viewCalImg, SIGNAL( clicked() ), this, SLOT( ShowCalImages() ) );
  connect( ui->pushButton_prevCalImg, SIGNAL (clicked() ), this, SLOT( PrevCalImage() ) );
  connect( ui->pushButton_nextCalImg, SIGNAL( clicked() ), this, SLOT( NextCalImage() ) );
  connect( ui->comboBox_targetType, SIGNAL( currentIndexChanged(QString) ), this, SLOT( UpdateFindTargetOptions(QString) ) );
  connect( ui->checkBox_viewRectified, SIGNAL( stateChanged(int) ), this, SLOT( SetCalImg(int) ) );
  connect( ui->pushButton_loadParameters, SIGNAL( clicked() ), this, SLOT( LoadStereoParameters() ) );
  connect( ui->checkBox_singleCamera, SIGNAL( stateChanged(int) ), this, SLOT( SingleCamSetup() ) );
  connect( ui->checkBox_useIntrinsicGuess, SIGNAL( stateChanged(int) ), this, SLOT( UseIntGuessStateChange() ) );
  connect( ui->checkBox_intrinsicFromFile, SIGNAL( stateChanged(int) ), this, SLOT( IntFromFileStateChange() ) );
  connect( ui->pushButton_lemon, SIGNAL( clicked() ), this, SLOT( MarkAsLemon() ) );

  SetupMat2QImage();
}


CameraCalibrator::~CameraCalibrator(){
  delete ui;
}


void CameraCalibrator::UseIntGuessStateChange(){
  ui->checkBox_intrinsicFromFile->setEnabled( ui->checkBox_useIntrinsicGuess->isChecked() );
}


void CameraCalibrator::IntFromFileStateChange(){
  ui->qwidget_inputIntrinsic->setEnabled( ui->checkBox_intrinsicFromFile->isChecked() );
}


void CameraCalibrator::SingleCamSetup(){
  const bool stereo_mode = !( ui->checkBox_singleCamera->isChecked() );
  ui->qwidget_rectSettings->setEnabled(stereo_mode);
  ui->label_camPrefix2->setEnabled(stereo_mode);
  ui->lineEdit_camPrefix2->setEnabled(stereo_mode);
  ui->label_extrinsicFileName->setEnabled(stereo_mode);
  ui->lineEdit_extrinsicFileName->setEnabled(stereo_mode);
  ui->pushButton_runRectification->setEnabled(stereo_mode);

  ui->checkBox_preCalibrate->setEnabled(stereo_mode);
  ui->checkBox_useIntrinsicGuess->setEnabled(stereo_mode);
  ui->checkBox_intrinsicFromFile->setEnabled(stereo_mode);
  //these options are only supported by cv::stereoCompute()
  ui->checkBox_fixIntrinsic->setEnabled(stereo_mode);
  ui->checkBox_fixFocalLength->setEnabled(stereo_mode);
  ui->checkBox_sameFocalLength->setEnabled(stereo_mode);
}


int CreateImageList(std::string dir_path, const std::string file_name_out, std::string file_ext, 
                    const std::vector<std::string> &file_prefix){
  mio::FormatFilePath(dir_path);
  mio::FormatFileExt(file_ext);
  const size_t file_ext_size = file_ext.size();

  //get directory listing
  std::vector<std::string> dir_list_vec;
  mio::GetDirList(dir_path, dir_list_vec, true);

  //create img_file_name_vec
  const size_t num_prefix = file_prefix.size();
  std::vector< std::vector<std::string> > img_file_name_vec(num_prefix);
  for(auto &dir_item : dir_list_vec) //check the entire directory listing
    for(size_t j = 0; j < num_prefix; ++j) //insert image file name into respective camera vector
      if( (dir_item.compare(0, file_prefix[j].size(), file_prefix[j]) == 0) && //check the file file prefix
          (dir_item.compare(dir_item.size() - file_ext_size, file_ext_size, file_ext) == 0) ) //check the file extension
        img_file_name_vec[j].push_back(dir_item);

  //check that each camera list is the same size
  const size_t num_img_per_prefix = img_file_name_vec[0].size();
  for(size_t i = 1; i < num_prefix; ++i)
    EXP_CHK_M(num_img_per_prefix == img_file_name_vec[i].size(), return(-1), "Different number of images between prefixes");

  if( mio::FileExists(dir_path + "/" + file_name_out + ".xml") )
    printf("CreateImageList(): output file already exists, replacing it\n");

  cv::FileStorage fs(dir_path + "/" + file_name_out + ".xml", cv::FileStorage::WRITE);
  fs << "images" << "[";
  for(size_t i = 0; i < num_img_per_prefix; ++i)
    for(size_t j = 0; j < num_prefix; ++j)
      fs << img_file_name_vec[j][i];
  fs << "]";
}


bool ReadStringList(const std::string fileNameFull, std::vector<std::string> &img_file_name_vec){
  std::cout << "ReadStringList() fileNameFull: " << fileNameFull << std::endl;
  img_file_name_vec.resize(0);
  cv::FileStorage fs(fileNameFull, cv::FileStorage::READ);
  if(!fs.isOpened() )
    return false;
  cv::FileNode n = fs.getFirstTopLevelNode();
  if(n.type() != cv::FileNode::SEQ)
    return false;
  cv::FileNodeIterator it = n.begin(), it_end = n.end();
  for(; it != it_end; ++it)
    img_file_name_vec.push_back( static_cast<std::string>(*it) );
  return true;
}


int CameraCalibrator::GetCalibrationFlags(){
  int flags = 0;

  if( ui->checkBox_fixIntrinsic->isChecked() )
    flags |= cv::CALIB_FIX_INTRINSIC;               //cv::stereoCalibrate() only
  if( ui->checkBox_useIntrinsicGuess->isChecked() )
    flags |= cv::CALIB_USE_INTRINSIC_GUESS;
  if( ui->checkBox_fixPrincipalPoint->isChecked() )
    flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
  if( ui->checkBox_fixFocalLength->isChecked() )
    flags |= cv::CALIB_FIX_FOCAL_LENGTH;            //cv::stereoCalibrate() only
  if( ui->checkBox_fixAspectRatio->isChecked() )
    flags |= cv::CALIB_FIX_ASPECT_RATIO;
  if( ui->checkBox_sameFocalLength->isChecked() )
    flags |= cv::CALIB_SAME_FOCAL_LENGTH;           //cv::stereoCalibrate() only
  if( ui->checkBox_zeroTangentDist->isChecked() )
    flags |= cv::CALIB_ZERO_TANGENT_DIST;
  if( ui->checkBox_fixK1->isChecked() )
    flags |= cv::CALIB_FIX_K1;
  if( ui->checkBox_fixK2->isChecked() )
    flags |= cv::CALIB_FIX_K2;
  if( ui->checkBox_fixK3->isChecked() )
    flags |= cv::CALIB_FIX_K3;
  if( ui->checkBox_fixK4->isChecked() )
    flags |= cv::CALIB_FIX_K4;
  if( ui->checkBox_fixK5->isChecked() )
    flags |= cv::CALIB_FIX_K5;
  if( ui->checkBox_fixK6->isChecked() )
    flags |= cv::CALIB_FIX_K6;
  if( ui->checkBox_rationalModel->isChecked() )
    flags |= cv::CALIB_RATIONAL_MODEL;
  if( ui->checkBox_thinPrism->isChecked() )
    flags |= cv::CALIB_THIN_PRISM_MODEL;
  if( ui->checkBox_fixThinPrism->isChecked() )
    flags |= cv::CALIB_FIX_S1_S2_S3_S4;
  if( ui->checkBox_tiltedModel->isChecked() )
    flags |= cv::CALIB_TILTED_MODEL;
  if( ui->checkBox_fixTauxTauy->isChecked() )
    flags |= cv::CALIB_FIX_TAUX_TAUY;

  return flags;
}


int CameraCalibrator::GetFindTargetFlags(const std::string target_type_str){
  unsigned int flags = 0;

  if(target_type_str == "chess"){
    if( ui->checkBox_adaptiveThresh->isChecked() )
      flags |= cv::CALIB_CB_ADAPTIVE_THRESH;
    if( ui->checkBox_normImage->isChecked() )
      flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
    if( ui->checkBox_filterQuads->isChecked() )
      flags |= cv::CALIB_CB_FILTER_QUADS;
    if( ui->checkBox_fastCheck->isChecked() )
      flags |= cv::CALIB_CB_FAST_CHECK;
  }
  else if(target_type_str == "circle" || target_type_str == "a-circle"){
    flags |= target_type_str == "circle" ? cv::CALIB_CB_SYMMETRIC_GRID : cv::CALIB_CB_ASYMMETRIC_GRID;
    if( ui->checkBox_clustering->isChecked() )
      flags |= cv::CALIB_CB_CLUSTERING;
  }
  else{
    printf( "CameraCalibrator::GetFindTargetFlags() - target_type_str %s is not supported, aborting\n", target_type_str.c_str() );
    return(0);
  }

  return flags;
}


void CameraCalibrator::SetFindTargetOptions(const bool a, const bool b, const bool c, const bool d, const bool e){
  ui->checkBox_adaptiveThresh->setEnabled(a);
  ui->checkBox_normImage->setEnabled(b);
  ui->checkBox_filterQuads->setEnabled(c);
  ui->checkBox_fastCheck->setEnabled(d);
  ui->checkBox_clustering->setEnabled(e);
}


//enables and disables target options depending on target type
void CameraCalibrator::UpdateFindTargetOptions(const QString new_target_type){
  std::string target_type_str = new_target_type.toStdString();
  if(target_type_str == "chess")
    SetFindTargetOptions(true,  true,  true,  true,  false);
  else if(target_type_str == "circle" || target_type_str == "a-circle")
    SetFindTargetOptions(false, false, false, false, true);
  else
    printf( "CameraCalibrator::GetFindTargetFlags() - target_type_str %s is not supported, aborting\n", target_type_str.c_str() );
}


void CameraCalibrator::SetCalPhotoDir(){
  QString saveDir = QFileDialog::getExistingDirectory(this, tr("Open Directory"), 
    QString(m_default_cal_img_dir), QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  ui->lineEdit_calPhotoDir->setText(saveDir);
  m_default_cal_img_dir = saveDir;
}


void CameraCalibrator::RunCalibration(const bool only_rectification){
  if(m_label)
    ShowCalImages();
  const bool stereo_mode = !( ui->checkBox_singleCamera->isChecked() );

  std::string cal_photo_dir = ui->lineEdit_calPhotoDir->text().toStdString();
  mio::FormatFilePath(cal_photo_dir);

  std::vector<std::string> file_prefix, img_file_name_vec;
  camCalTarget_t cal_target;
  if(!only_rectification){
    file_prefix.push_back( ui->lineEdit_camPrefix1->text().toStdString() );
    if(stereo_mode)
      file_prefix.push_back( ui->lineEdit_camPrefix2->text().toStdString() );
    CreateImageList(cal_photo_dir, "imgList", std::string( ui->comboBox_imgExt->currentText().toStdString() ), file_prefix);
    ReadStringList(cal_photo_dir + "/" + "imgList.xml", img_file_name_vec);
    EXP_CHK(img_file_name_vec.size() > 2, return)

    cal_target = camCalTarget_t( ui->comboBox_targetType->currentText().toStdString(),
                                 cv::Size( ui->spinBox_targetWidth->value(), ui->spinBox_targetHeight->value() ),
                                 ui->doubleSpinBox_spacing->value() );
    //DisplayCalTarget(point_vec); //assumes target spacing is in centimeters

    const int num_img_per_cam = ExtractCalTargetPoints(cal_photo_dir, cal_target, img_file_name_vec, m_cal_data,
                                                       GetFindTargetFlags(cal_target.type_str), stereo_mode ? 2 : 1);
    if(num_img_per_cam < 2)
      return;
    std::cout << m_cal_data.GetNumImgPerCam() << " images per camera successfully detected.\n";

    LoadCalImages(cal_photo_dir, cal_target, m_cal_data, m_cal_img_vec, stereo_mode ? 2 : 1);
  }

  if(stereo_mode){
    if(!only_rectification){
      if( ui->checkBox_useIntrinsicGuess->isChecked() &&
          ui->checkBox_intrinsicFromFile->isChecked() ){
        LoadCameraCalData(cal_photo_dir, ui->lineEdit_inputIntrinsic1->text().toStdString(),
                          ui->lineEdit_inputIntrinsic2->text().toStdString(), m_cal_data);
          std::cout << "Loaded intrinsic parameters from file...\n";
          mio::Print(m_cal_data.K[0], "M1");
          mio::Print(m_cal_data.K[1], "M2");
          mio::Print(m_cal_data.D[0], "D1");
          mio::Print(m_cal_data.D[1], "D2");
      }

      std::vector<double> repro_err_vec;
      double repro_err_1, repro_err_2;
      StereoCalibrate( cal_target,
                       m_cal_data,
                       repro_err_vec, repro_err_1, repro_err_2,
                       GetCalibrationFlags(), ui->checkBox_preCalibrate->isChecked() );

      ui->lineEdit_rmsError->setText( QString::number(repro_err_1) );
      ui->lineEdit_reprojectionError->setText( QString::number(repro_err_2) );
      std::cout << "reprojection error per pair of images:" << std::endl;
      for(size_t i = 0; i < m_cal_data.GetNumImgPerCam(); i++)
        std::cout << m_cal_data.good_img_file_names[0][i] << ", " << 
                     m_cal_data.good_img_file_names[1][i] << ": " << repro_err_vec[i] << std::endl;
    }

    m_cal_data.ClearRectData();
    ComputeRectification(cal_photo_dir, m_cal_data, !ui->checkBox_rectHartley->isChecked(),
                         ui->checkBox_rectZeroDisparity->isChecked() ? cv::CALIB_ZERO_DISPARITY : -1,
                         ui->checkBox_rectAlpha->isChecked() ? ui->doubleSpinBox_rectAlpha->value() : -1);

    RectifyImages(cal_photo_dir, m_cal_data, m_rect_cal_img_vec);

    SaveStereoCalData(cal_photo_dir, ui->lineEdit_intrinsicFileName->text().toStdString(),
                      ui->lineEdit_extrinsicFileName->text().toStdString(), m_cal_data);
  }
  else{ //single camera mode
    CalibrateCamera( cal_target, m_cal_data, 0, GetCalibrationFlags() );
    SaveCameraCalData(cal_photo_dir, ui->lineEdit_intrinsicFileName->text().toStdString(), m_cal_data);
  }

  m_cal_data.Print(!stereo_mode);
}


void CameraCalibrator::RunRectifiction(){
  RunCalibration(true);
}


void CameraCalibrator::LoadStereoParameters(){
  std::string cal_photo_dir = ui->lineEdit_calPhotoDir->text().toStdString();
  mio::FormatFilePath(cal_photo_dir);
  LoadStereoCalData(cal_photo_dir, ui->lineEdit_intrinsicFileName->text().toStdString(),
                    ui->lineEdit_extrinsicFileName->text().toStdString(), m_cal_data);
  camCalTarget_t cal_target( ui->comboBox_targetType->currentText().toStdString(),
                             cv::Size( ui->spinBox_targetWidth->value(), ui->spinBox_targetHeight->value() ),
                             ui->doubleSpinBox_spacing->value() );
  LoadCalImages(cal_photo_dir, cal_target, m_cal_data, m_cal_img_vec, ui->checkBox_singleCamera->isChecked() ? 1 : 2);
  RectifyImages(cal_photo_dir, m_cal_data, m_rect_cal_img_vec);
  m_cal_data.Print( !( ui->checkBox_singleCamera->isChecked() ) );
}


void CameraCalibrator::ShowCalImages(){
  if(!m_label){
    if(m_cal_img_vec.size() > 0){
      m_label = new QLabel;
      ui->verticalLayout_label->addWidget(m_label);
      m_disp_img_line_edit = new QLineEdit;
      ui->verticalLayout_label->addWidget(m_disp_img_line_edit);
      m_disp_img_idx = 1;
      PrevCalImage();
      if( !( ui->checkBox_singleCamera->isChecked() ) )
        ui->checkBox_viewRectified->setEnabled(true);
      ui->pushButton_nextCalImg->setEnabled(true);
      ui->pushButton_prevCalImg->setEnabled(true);
    }
  }
  else{
    ui->verticalLayout_label->removeWidget(m_disp_img_line_edit);
    ui->verticalLayout_label->removeWidget(m_label);
    delete m_label;
    m_label = NULL;
    delete m_disp_img_line_edit;
    m_disp_img_line_edit = NULL;
    ui->pushButton_nextCalImg->setEnabled(false);
    ui->pushButton_prevCalImg->setEnabled(false);
    ui->checkBox_viewRectified->setEnabled(false);
    QCoreApplication::processEvents();
  }
}


void CameraCalibrator::SetCalImg(int state){
  if(m_label)
    SetCalImg();
}


void CameraCalibrator::SetCalImg(){
  try{
    OpenCVMat2QImage(ui->checkBox_viewRectified->isChecked() ? m_rect_cal_img_vec[m_disp_img_idx] : 
                     m_cal_img_vec[m_disp_img_idx], m_disp_qt_img, true);
    m_label->setPixmap( QPixmap::fromImage(m_disp_qt_img) );
    std::string imgNames = m_cal_data.good_img_file_names[0][m_disp_img_idx] + ", " +
                           m_cal_data.good_img_file_names[1][m_disp_img_idx];
    m_disp_img_line_edit->setText( imgNames.c_str() );
  }
  catch(cv::Exception &e){
    printf( "CameraCalibrator::SetCalImg() - caught error - %s\n", e.what() );
  }
}


void CameraCalibrator::NextCalImage(){
  if( m_label && m_cal_img_vec.size() > 0 && m_disp_img_idx + 1 < m_cal_img_vec.size() ){
    m_disp_img_idx++;
    SetCalImg();
  }
}


void CameraCalibrator::PrevCalImage(){
  if(m_label && m_cal_img_vec.size() > 0 && m_disp_img_idx > 0){
    m_disp_img_idx--;
    SetCalImg();
  }
}


void DrawCross(cv::Mat &img){
  if( !( img.empty() ) ){
    cv::line(img, cv::Point(0, 0), cv::Point(img.cols, img.rows), cv::Scalar::all(255), 5);
    cv::line(img, cv::Point(0, img.rows), cv::Point(img.cols, 0), cv::Scalar::all(255), 5);
  }
}

void CameraCalibrator::MarkAsLemon(){
  EXP_CHK(m_label && m_cal_img_vec.size() > 0 && m_disp_img_idx >= 0, return)
  const size_t num_camera = ui->checkBox_singleCamera->isChecked() ? 1 : 2;
  const std::string lemon_prefix = "lemon_";
  for(size_t i = 0; i < num_camera; ++i){
    const std::string file_full = m_cal_data.good_img_file_names[i][m_disp_img_idx];
    if(file_full.compare(0, lemon_prefix.size(), lemon_prefix) != 0){
      m_cal_data.good_img_file_names[i][m_disp_img_idx].insert(0, lemon_prefix);
      const std::string cal_photo_dir = ui->lineEdit_calPhotoDir->text().toStdString(),
                        old_name = cal_photo_dir + "/" + file_full,
                        new_name = cal_photo_dir + "/" + m_cal_data.good_img_file_names[i][m_disp_img_idx];
      rename( old_name.c_str(), new_name.c_str() );
      if(m_cal_img_vec.size() > m_disp_img_idx)
        DrawCross( m_cal_img_vec[m_disp_img_idx] );
      if(m_rect_cal_img_vec.size() > m_disp_img_idx)
        DrawCross( m_rect_cal_img_vec[m_disp_img_idx] );
      SetCalImg();
    }
  }
}


int main(int argc, char *argv[]){
  QApplication a(argc, argv);
  QApplication::setStyle(QStyleFactory::create("Fusion"));
  CameraCalibrator w;
  w.setWindowIcon( QIcon(":/opencv_logo.png") );
  w.show();

  return a.exec();
}

