
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

struct myclass {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
} poredjenjeTacaka;

struct myclass2 {
    bool operator() (double a, double b) { return (a < b);}
} poredjenjeDouble;

Point2f computeIntersect(cv::Vec4i a, 
                             cv::Vec4i b)
{
        int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3], x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];
        float denom;

        if (float d = ((float)(x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)))
        {
                cv::Point2f pt;
                pt.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
                pt.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;
                return pt;
        }
        else
                return cv::Point2f(-1, -1);
}

 void sortiranjeCoskova(std::vector<cv::Point2f>& coskovi, cv::Point2f centar)  
 {  


   std::vector<cv::Point2f> vrh, dno;  
   for (int i = 0; i < coskovi.size(); i++)  
   {  
     if (coskovi[i].y < centar.y)  
       vrh.push_back(coskovi[i]);  
     else  
       dno.push_back(coskovi[i]);  
   }  
   
  sort(vrh.begin(),vrh.end(),poredjenjeTacaka);  
  sort(dno.begin(),dno.end(),poredjenjeTacaka); 

   cv::Point2f gl = vrh[0];  
   cv::Point2f gd = vrh[vrh.size()-1];  
   cv::Point2f dl = dno[0];  
   cv::Point2f dd = dno[dno.size()-1]; 
   coskovi.clear(); 

   coskovi.push_back(gl);  
   coskovi.push_back(gd);  
   coskovi.push_back(dd);  
   coskovi.push_back(dl);  
   cout<<"gornji lijevi"<<gl<<" gornji desni"<<gd<<" donji desni"<<dd<<" donji lijevi"<<endl;
 }  

int main(int argc, char* argv[]){

	const int brPitanja=5;
	const int brOdg=5;
	//inicijalno svi odgovori imaju vrijednost -1
		int tacni[brPitanja][brOdg];
		for (int m=0; m<brPitanja; m++){
		  for (int n=0; n<brOdg; n++){
			  tacni[m][n]=-1;
		  }
		}
		//definisanje tacnih odgovora
	tacni[0][2]=1;
	tacni[1][3]=1;
	tacni[2][0]=1;
	tacni[2][1]=1;
	tacni[3][1]=1;
	tacni[4][3]=1;
	tacni[4][4]=1;

	//ucitavanje slike
	 const char* filename = argc >= 2 ? argv[1] : "test1.jpg";

 Mat img = imread(filename, 0);
 if(img.empty())
 {
     
     cout << "can not open " << filename << endl;
     return -1;
 }
  
  cv::GaussianBlur(img,img,cv::Size(3,3),0); //blur
   imshow("primjer0",img);
  adaptiveThreshold(img, img,255,CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,75,10);  
  imshow("primjer0.1",img);
  cv::bitwise_not(img, img); //umjesto crno-bijelo ide bijelo-crno   
imshow("primjer1",img);

  cv::Mat slika2;  
  cvtColor(img,slika2, CV_GRAY2RGB);

    vector<Vec4i> linije;  
  HoughLinesP(img, linije, 1, CV_PI/180, 80, 80, 10); 
  cout<<linije.size()<<" linija"<<endl;
  for( size_t i = 0; i < linije.size(); i++ )  
  {  
   Vec4i l = linije[i];  
   line( slika2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 3, CV_AA);   
  }  

  imshow("primjer2",slika2);
  
    std::vector<cv::Point2f> coskovi;
  for (int i = 0; i < linije.size(); i++)
  {
      for (int j = i+1; j < linije.size(); j++)
      {
          cv::Point2f pt = computeIntersect(linije[i], linije[j]);
          if (pt.x >= 0 && pt.y >= 0 && pt.x < img.cols && pt.y < img.rows)
              coskovi.push_back(pt);		  
      }	 
  }



  cout<<coskovi.size();
  cv::Point2f centar(0,0);  
  for (int i = 0; i < coskovi.size(); i++)  
  centar += coskovi[i];  
  centar *= (1./ coskovi.size());
 
   sortiranjeCoskova(coskovi, centar);

     for (int cor=0; cor<coskovi.size(); cor++){
  // Draw corner points
	cv::circle(slika2, coskovi[0], 3, CV_RGB(255,0,0), 2);
	cv::circle(slika2, coskovi[1], 3, CV_RGB(255,255,0), 2);
	cv::circle(slika2, coskovi[2], 3, CV_RGB(0,0,255), 2);
	cv::circle(slika2, coskovi[3], 3, CV_RGB(255,0,255), 2);
  }
       imshow("primjer2.5",slika2);

     Rect r = boundingRect(coskovi); 
  cout<<r<<endl;
  cv::Mat quad = cv::Mat::zeros(r.height, r.width, CV_8UC3);  
  // Coskovi slike nad kojom vrsimo transformaciju 
  std::vector<cv::Point2f> quad_pts;  
  quad_pts.push_back(cv::Point2f(0, 0));  
  quad_pts.push_back(cv::Point2f(quad.cols, 0));  
  quad_pts.push_back(cv::Point2f(quad.cols, quad.rows));  
  quad_pts.push_back(cv::Point2f(0, quad.rows));



      cv::Mat slika3;  
  cvtColor(img,slika3, CV_GRAY2RGB);

    // matrica transformacije
  cv::Mat transmtx = cv::getPerspectiveTransform(coskovi, quad_pts);  //prvi param. coskovi stare slike,drugi param. coskovi nove slike
  // primjena transformacije  
  cv::warpPerspective(slika3, quad, transmtx, quad.size());  

  imshow("primjer3",quad);

    Mat cimg;
  
  cvtColor(quad,cimg, CV_BGR2GRAY);
  vector<Vec3f> kruznice;  
  HoughCircles(cimg, kruznice, CV_HOUGH_GRADIENT, 1, img.rows/7, 50, 40, 0, 0 );  
  for( size_t i = 0; i < kruznice.size(); i++ ){  
    Point centar(cvRound(kruznice[i][0]), cvRound(kruznice[i][1]));
    // centar kruznice  
    circle( quad, centar, 3, Scalar(0,0,255), -1, 8, 0 );
  }

    imshow("primjer4",quad);

    double prosR = 0;
  vector<double> red;
  vector<double> kol;
  vector<double> poeni;
  //Find rows and columns of circles for interpolation
  for(int i=0;i<kruznice.size();i++){  
    bool pronadjen = false;  
    int r = cvRound(kruznice[i][2]);
    prosR += r;
    int x = cvRound(kruznice[i][0]);  
    int y = cvRound(kruznice[i][1]);
    for(int j=0;j<red.size();j++){
      double y2 = red[j];
      if(y - r < y2 && y + r > y2){
        pronadjen = true;
        break;
      }
    }
    if(!pronadjen){
      red.push_back(y);
    }
    pronadjen = false;
    for(int j=0;j<kol.size();j++){
      double x2 = kol[j];
      if(x - r < x2 && x + r > x2){
        pronadjen = true;
        break;
      }
    }
    if(!pronadjen){  
      kol.push_back(x);
    }
  }
    prosR /= kruznice.size();

  sort(red.begin(),red.end(),poredjenjeDouble);
  sort(kol.begin(),kol.end(),poredjenjeDouble);
vector<int> indeksi;
    for(int i=0;i<red.size();i++){
    double max = 0;  
    double y = red[i];
  
    for(int j=0;j<kol.size();j++){
      double x = kol[j];
      Point c(x,y); 
	  
      //Use an actual circle if it exists
      for(int k=0;k<kruznice.size();k++){
        double x2 = kruznice[k][0];
        double y2 = kruznice[k][1];
        if(abs(y2-y)<prosR && abs(x2-x)<prosR){
          x = x2;
          y = y2;
        }
      }

      // circle outline  
      circle( quad, c, prosR, Scalar(255,0,0), 3, 8, 0 );  
      Rect rect(x-prosR,y-prosR,2*prosR,2*prosR);  
      Mat submat = cimg(rect);  //biramo kvadrat oko kruga, koji nam je od interesa
	   imshow("primjer6",submat);
	     waitKey();
      double p =(double)countNonZero(submat)/(submat.size().width*submat.size().height);  
      if(p>=0.33 ){  
          
		indeksi.push_back(j);
      } 
	  else
		  indeksi.push_back(-1);
	}
	 printf("%d:",i+1);
  for (int j=0; j<red.size(); j++) {
		    if(indeksi[j]>=0) printf("%c ",'A'+j);
      
  } 
  int koef=1;
  int rezultat=0;
  int tacnih=0;
  double kolicnik=0;
 
  for (int od=0; od<red.size(); od++) {

	  if (indeksi[od]>=0 && tacni[i][od]==1) {
	 if (koef==1)
		  rezultat++;
	 else {rezultat=0; }
	  ++tacnih;}
	  else if (indeksi[od]>=0 && tacni[i][od]==-1){
	  koef=0;
	  rezultat=0;
	  }
	  else if (indeksi[od]==-1 && tacni[i][od]==1){
	  
	  ++tacnih;}

cout<<rezultat<<" "<<tacnih<<endl;

  }
  cout<<rezultat;
 if (rezultat==0 && tacnih==0) kolicnik=0; 
else kolicnik= rezultat/(double)tacnih;
poeni.push_back(kolicnik); 
  
  indeksi.clear();
	cout<<endl;

	}
double zbir=0;
	for (int pit=0; pit<poeni.size(); pit++ )
{
zbir+=poeni[pit];

}
	int ocjena=(double)zbir/(double) poeni.size()*100;
printf("Ostvaren broj poena je %d",ocjena);
	  imshow("primjer5",quad);
  waitKey();
  return 0;
}
