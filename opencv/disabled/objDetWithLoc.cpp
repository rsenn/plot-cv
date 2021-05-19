#include <iostream>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// using namespace cv;
using namespace std;

int
main() {
  cv::VideoCapture video(0); // web camden video almak için nesne oluşturuldu

  if(!video.isOpened()) {                // kamera açılamazsa
    cout << "Web cam acilamadi" << endl; // ekrana bilgi basıldı
    return -1;                           // uygulama sonlandırılıyor
  }

  // yeşil renk algılaması için gerekli HSV Min Ve Max değerleri oluşturuldu
  int Hmin = 31;
  int Hmax = 94;

  int Smin = 14;
  int Smax = 150;

  int Vmin = 194;
  int Vmax = 255;

  // algılanan nesnenin x ve y lokasyonlarını tutacak değişkenler
  int eskix = -1;
  int eskiy = -1;

  while(true) {        // sürekli tarama yapmak için sonsuz döngü
    cv::Mat yeniFrame; // kameradan alınan framelerin yükleneceği nesne

    bool okumaYapiliyormu = video.read(yeniFrame); // web camden video almak için nesne oluşturuldu

    if(!okumaYapiliyormu) {              // kameradan frame alınamamışsa
      cout << "frame alinamadi" << endl; // kullanıcıya bilgi veriliyor
      break;                             //çıkılıyor
    }

    cv::Mat hsvFrame; // HSV görüntünün yükleneceği nesne

    cv::cvtColor(yeniFrame, hsvFrame, COLOR_BGR2HSV); // RGB cv::formatındaki frame HSV'ye dönüştürüldü

    cv::Mat islenenFrame; // işlenecek frame'in yükleneceği nesne

    // HSV'ye dönüştürülen frame Hmin-max,Smin-max,Vmin-max değer aralıklarına getirilerek isres
    // nesnesine yüklendi
    cv::inRange(hsvFrame, cv::Scalar(Hmin, Smin, Vmin), cv::Scalar(Hmax, Smax, Vmax), islenenFrame);

    // HSV değerlerine göre asıl almak istenilen görüntünün tamamına ulaşmak için görüntü üzerinde
    // 5x5'lik matrisle nokta belirginleştirme işlemi yapılıyor
    cv::erode(islenenFrame, islenenFrame, cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(islenenFrame, islenenFrame, cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)));

    // 5x5'lik matrisle taranarak gürültüler siliniyor
    cv::dilate(islenenFrame, islenenFrame, cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::erode(islenenFrame, islenenFrame, cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)));

    // ekranda algılanan görüntünün lokasyon değerlerini almak için konum nesnesi oluşturuldu
    cv::Moments konum = cv::moments(islenenFrame);
    double yeksen = konum.m01; // görüntünün y konumu alındı
    double xeksen = konum.m10; // görüntünün x konumu alındı
    double alan = konum.m00;   // görüntünün alanı alındı

    if(alan > 10000) { // görüntünün alanı eşik değerinden büyükse
      // algılanan alanın tam orta noktasının lokasyonları hesaplanıyor
      int yenix = xeksen / alan;
      int yeniy = yeksen / alan;

      if(eskix >= 0 && eskiy >= 0 && yenix >= 0 && yeniy >= 0) { // lokasyonlar sıfırdan büyükse
        stringstream convert, convert2;                          // ekrana basma için gerekli tür dönüşümleri yapılıyor
        convert << yenix;
        string x = convert.str();
        convert2 << yeniy;
        string y = convert2.str();

        // görüntünün orta noktasına hesaplanan lokasyonlar yazdırılıyor
        cv::putText(
            yeniFrame, x + "," + y, cvPoint(yenix, yeniy), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 0), 1, cv::LINE_AA);
      }

      // yeni lokasyonlar güncelleniyor
      eskix = yenix;
      eskiy = yeniy;
    }

    cv::putText(
        yeniFrame, " MAHSERIN 3 ATLISI ", cvPoint(2, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 0, 0), 1, cv::LINE_AA);

    cv::imshow("gomuluRenkAlgilama", yeniFrame);

    if(cv::waitKey(30) == 27) { // escye basıldığında program sonlandırılıyor
      cout << "kullanici cikis yapti" << endl;
      break;
    }
  }
  return 0;
}