#include "aruco.h"
#include <eigen3/Eigen/Dense>
#include "cvdrawingutils.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <stdexcept>

using namespace std;
using namespace cv;
using namespace aruco;
using Eigen::MatrixXf;

MarkerDetector MDetector1,MDetector2;
VideoCapture TheVideoCapturer1,TheVideoCapturer2;
vector<Marker> TheMarkers1,TheMarkers2;
Mat TheInputImage1,TheInputImageGrey1, TheInputImageCopy1, TheInputImage2,TheInputImageGrey2, TheInputImageCopy2;
CameraParameters TheCameraParameters;


void cvTackBarEvents(int pos, void*);
string dictionaryString;
int iDetectMode=0,iMinMarkerSize=0,iCorrectionRate=0,iShowAllCandidates=0,iEnclosed=0,iThreshold,iCornerMode,iDictionaryIndex=0;

int waitTime = 0;
bool showMennu=false,bPrintHelp=false,isVideo=false;
class CmdLineParser{int argc;char** argv;public:CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}   bool operator[](string param)    {int idx = -1;  for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;return (idx != -1);}    string operator()(string param, string defvalue = "-1")    {int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;if (idx == -1)return defvalue;else return (argv[idx + 1]);}};
struct   TimerAvrg{std::vector<double> times;size_t curr=0,n; std::chrono::high_resolution_clock::time_point begin,end;   TimerAvrg(int _n=30){n=_n;times.reserve(n);   }inline void start(){begin= std::chrono::high_resolution_clock::now();    }inline void stop(){end= std::chrono::high_resolution_clock::now();double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;if ( times.size()<n) times.push_back(duration);else{ times[curr]=duration; curr++;if (curr>=times.size()) curr=0;}}double getAvrg(){double sum=0;for(auto t:times) sum+=t;return sum/double(times.size());}};

TimerAvrg Fps;
cv::Mat resize(const cv::Mat& in, int width)
{
    if (in.size().width <= width)
        return in;
    float yf = float(width) / float(in.size().width);
    cv::Mat im2;
    cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
    return im2;
}
void setParamsFromGlobalVariables(aruco::MarkerDetector &md){


    md.setDetectionMode((DetectionMode)iDetectMode,float(iMinMarkerSize)/1000.);
    md.getParameters().setCornerRefinementMethod( (aruco::CornerRefinementMethod) iCornerMode);

    md.getParameters().detectEnclosedMarkers(iEnclosed);
    md.getParameters().ThresHold=iThreshold;
    md.setDictionary((aruco::Dictionary::DICT_TYPES) iDictionaryIndex,float(iCorrectionRate)/10. );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
}

void createMenu(){
    cv::createTrackbar("Dictionary", "menu", &iDictionaryIndex, 13, cvTackBarEvents);
   cv::createTrackbar("DetectMode", "menu", &iDetectMode, 2, cvTackBarEvents);
   cv::createTrackbar("CornerMode", "menu", &iCornerMode, 2, cvTackBarEvents);

   cv::createTrackbar("MinMarkerSize", "menu", &iMinMarkerSize, 1000, cvTackBarEvents);
   cv::createTrackbar("Threshold", "menu", &iThreshold, 40, cvTackBarEvents);
   cv::createTrackbar("ErrorRate", "menu", &iCorrectionRate, 10, cvTackBarEvents);
   cv::createTrackbar("Enclosed", "menu", &iEnclosed, 1, cvTackBarEvents);
   cv::createTrackbar("ShowAll", "menu", &iShowAllCandidates, 1, cvTackBarEvents);
   iThreshold=MDetector1.getParameters().ThresHold;
   iCornerMode= MDetector1.getParameters().cornerRefinementM;
   iThreshold=MDetector2.getParameters().ThresHold;
   iCornerMode= MDetector2.getParameters().cornerRefinementM;

}

void putText(cv::Mat &im,string text,cv::Point p,float size){
    float fact=float(im.cols)/float(640);
    if (fact<1) fact=1;

    cv::putText(im,text,p,FONT_HERSHEY_SIMPLEX, size,cv::Scalar(0,0,0),3*fact);
    cv::putText(im,text,p,FONT_HERSHEY_SIMPLEX, size,cv::Scalar(125,255,255),1*fact);

}
void printHelp(cv::Mat &im)
{
    float fs=float(im.cols)/float(1200);

    putText(im,"'m': show/hide menu",cv::Point(10,fs*60),fs*0.5f);
    putText(im,"'s': start/stop video capture",cv::Point(10,fs*80),fs*0.5f);
    putText(im,"'w': write image to file",cv::Point(10,fs*100),fs*0.5f);
    putText(im,"'t': do a speed test",cv::Point(10,fs*120),fs*0.5f);
    putText(im,"'f': saves current configuration to file 'arucoConfig.yml'",cv::Point(10,fs*140),fs*0.5f);
}

void printInfo(cv::Mat &im){
    float fs=float(im.cols)/float(1200);
    putText(im,"fps="+to_string(1./Fps.getAvrg()),cv::Point(10,fs*20),fs*0.5f);
    putText(im,"'h': show/hide help",cv::Point(10,fs*40),fs*0.5f);
    if(bPrintHelp) printHelp(im);
}

void printMenuInfo(){
        cv::Mat image(200,400,CV_8UC3);
        image=cv::Scalar::all(255);
        string str="Dictionary="+aruco::Dictionary::getTypeString((aruco::Dictionary::DICT_TYPES) iDictionaryIndex) ;

        cv::putText(image,str,cv::Size(10,20),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);

        str="Detection Mode="+MarkerDetector::Params::toString(MDetector1.getParameters().detectMode);
        cv::putText(image,str,cv::Size(10,40),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
        str="Corner Mode="+MarkerDetector::Params::toString(MDetector1.getParameters().cornerRefinementM);;
        cv::putText(image,str,cv::Size(10,60),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
        cv::imshow("menu",image);


        cv::putText(image,str,cv::Size(10,20),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
        str="Detection Mode="+MarkerDetector::Params::toString(MDetector2.getParameters().detectMode);
        cv::putText(image,str,cv::Size(10,40),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
        str="Corner Mode="+MarkerDetector::Params::toString(MDetector2.getParameters().cornerRefinementM);;
        cv::putText(image,str,cv::Size(10,60),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
        cv::imshow("menu",image);
}

cv::Mat resizeImage(cv::Mat &in,float resizeFactor){
    if (fabs(1-resizeFactor)<1e-3 )return in;
    float nc=float(in.cols)*resizeFactor;
    float nr=float(in.rows)*resizeFactor;
    cv::Mat imres;
    cv::resize(in,imres,cv::Size(nc,nr));
    cout<<"Imagesize="<<imres.size()<<endl;
    return imres;
}


int range(MatrixXf a, MatrixXf b)
{
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
        {
            if (((a(i,j)-b(i,j))>2)||((b(i,j)-a(i,j))>2))
                return 0;
        }
    return 1;
}
int main(int argc, char** argv)
{
    try
    {
        CmdLineParser cml(argc, argv);
        if (argc<3 || cml["-h"])
        {
            cerr << "Invalid number of arguments" << endl;
            cerr << "Usage: (in1.avi|live[:camera_index(e.g 0 or 1 or 2)]) (in2.avi|live[:camera_index(e.g 0 or 1 or 2)]) [-c camera_params.yml] [-s  marker_size_in_meters] [-d "
                    "dictionary:ALL_DICTS by default] [-h]"
                 << endl;
            cerr << "\tDictionaries: ";
            for (auto dict : aruco::Dictionary::getDicTypes())
                cerr << dict << " ";
            cerr << endl;
            cerr << "\t Instead of these, you can directly indicate the path to a file with your own generated "
                    "dictionary"
                 << endl;
            return false;
        }

        int i,j,flag=0;
        MatrixXf Transform(4,4),Transform1(4,4);
        ofstream myfile ("Transformation.txt");

        string TheInputVideo1 = argv[1];
        string TheInputVideo2 = argv[2];

        // read camera parameters if passed
        if (cml["-c"])
            TheCameraParameters.readFromXMLFile(cml("-c"));

        float TheMarkerSize = std::stof(cml("-s", "-1"));
        //resize factor
        float resizeFactor=stof(cml("-rf","1"));


        ///////////  PARSE ARGUMENTS
        
        ///////////  OPEN VIDEO
        // read from camera or from  file
        if (TheInputVideo1.find("live") != string::npos)
        {
            int vIdx = 0;
            // check if the :idx is here
            char cad[100];
            if (TheInputVideo1.find(":") != string::npos)
            {
                std::replace(TheInputVideo1.begin(), TheInputVideo1.end(), ':', ' ');
                sscanf(TheInputVideo1.c_str(), "%s %d", cad, &vIdx);
            }
            cout << "Opening camera index " << vIdx << endl;
            TheVideoCapturer1.open(vIdx);
            waitTime = 10;
            isVideo=true;
        }
        if (TheInputVideo2.find("live") != string::npos)
        {
            int vIdx = 0;
            // check if the :idx is here
            char cad[100];
            if (TheInputVideo2.find(":") != string::npos)
            {
                std::replace(TheInputVideo2.begin(), TheInputVideo2.end(), ':', ' ');
                sscanf(TheInputVideo2.c_str(), "%s %d", cad, &vIdx);
            }
            cout << "Opening camera index " << vIdx << endl;
            TheVideoCapturer2.open(vIdx);
            waitTime = 10;
            isVideo=true;
        }
        else{    
            TheVideoCapturer1.open(TheInputVideo1);
            if ( TheVideoCapturer1.get(CV_CAP_PROP_FRAME_COUNT)>=2) isVideo=true;
      
            // check video is open
            if (!TheVideoCapturer1.isOpened())
                throw std::runtime_error("Could not open video 1");

            TheVideoCapturer2.open(TheInputVideo2);
            if ( TheVideoCapturer2.get(CV_CAP_PROP_FRAME_COUNT)>=2) isVideo=true;
      
            // check video is open
            if (!TheVideoCapturer2.isOpened())
                throw std::runtime_error("Could not open video 2");
        }


        ///// CONFIGURE DATA
        // read first image to get the dimensions
        TheVideoCapturer1 >> TheInputImage1;
        if (TheCameraParameters.isValid())
        {
            TheCameraParameters.resize(TheInputImage1.size());
            cout<<"VAlid!"<<endl;
        }
        dictionaryString=cml("-d", "ALL_DICTS");
        iDictionaryIndex=(uint64_t)aruco::Dictionary::getTypeFromString(dictionaryString);
         MDetector1.setDictionary(dictionaryString,float(iCorrectionRate)/10. );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
         iThreshold=MDetector1.getParameters().ThresHold;
         iCornerMode= MDetector1.getParameters().cornerRefinementM;


        cv::namedWindow("in1",cv::WINDOW_NORMAL);
        cv::resizeWindow("in1",640,480);

        setParamsFromGlobalVariables(MDetector1);

        {
        float w1=std::min(int(1920),int(TheInputImage1.cols));
        float f1=w1/float(TheInputImage1.cols);
        resizeWindow("in1",w1,float(TheInputImage1.rows)*f1);
        }


        ///// CONFIGURE DATA
        // read first image to get the dimensions
        TheVideoCapturer2 >> TheInputImage2;
        if (TheCameraParameters.isValid())
            TheCameraParameters.resize(TheInputImage2.size());
        dictionaryString=cml("-d", "ALL_DICTS");
        iDictionaryIndex=(uint64_t)aruco::Dictionary::getTypeFromString(dictionaryString);

         MDetector2.setDictionary(dictionaryString,float(iCorrectionRate)/10. );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
         iThreshold=MDetector2.getParameters().ThresHold;
         iCornerMode= MDetector2.getParameters().cornerRefinementM;

        cv::namedWindow("in2",cv::WINDOW_NORMAL);
        cv::resizeWindow("in2",640,480);

        setParamsFromGlobalVariables(MDetector2);

        {
        float w2=std::min(int(1920),int(TheInputImage2.cols));
        float f2=w2/float(TheInputImage2.cols);
        resizeWindow("in2",w2,float(TheInputImage2.rows)*f2);
        }        


        // go!
        char key = 0;
        int index = 0,indexSave=0;
        // capture until press ESC or until the end of the video

         do
        {

             TheVideoCapturer1.retrieve(TheInputImage1);
             TheInputImage1=resizeImage(TheInputImage1,resizeFactor);

             TheVideoCapturer2.retrieve(TheInputImage2);
             TheInputImage2=resizeImage(TheInputImage2,resizeFactor);
              // copy image
            Fps.start();
            TheMarkers1 = MDetector1.detect(TheInputImage1, TheCameraParameters, 0.092);
            TheMarkers2 = MDetector2.detect(TheInputImage2, TheCameraParameters, 0.092);//THE THIRD PARAMETER IS VERY IMPORTANT, THE MARKER SIZE.
            Fps.stop();
           
            TheInputImage1.copyTo(TheInputImageCopy1);
            TheInputImage2.copyTo(TheInputImageCopy2);


            if (iShowAllCandidates){
                auto candidates=MDetector1.getCandidates();
                for(auto cand:candidates)
                    Marker(cand,-1).draw(TheInputImageCopy1, Scalar(255, 0, 255));
            }

            for (unsigned int i = 0; i < TheMarkers1.size(); i++)
            {
                //cout << TheMarkers1[i] << endl;
                TheMarkers1[i].draw(TheInputImageCopy1, Scalar(0, 0, 255),2,true);
            }

            // draw a 3d cube in each marker if there is 3d info
            if (TheCameraParameters.isValid() && TheMarkerSize > 0)
                for (unsigned int i = 0; i < TheMarkers1.size(); i++)
                {
                    CvDrawingUtils::draw3dCube(TheInputImageCopy1, TheMarkers1[i], TheCameraParameters);
                    CvDrawingUtils::draw3dAxis(TheInputImageCopy1, TheMarkers1[i], TheCameraParameters);
                }



            if (iShowAllCandidates){
                auto candidates=MDetector2.getCandidates();
                for(auto cand:candidates)
                    Marker(cand,-1).draw(TheInputImageCopy2, Scalar(255, 0, 255));
            }

            for (unsigned int i = 0; i < TheMarkers2.size(); i++)
            {
                //cout << TheMarkers2[i] << endl;
                TheMarkers2[i].draw(TheInputImageCopy2, Scalar(0, 0, 255),2,true);
            }

            // draw a 3d cube in each marker if there is 3d info
            if (TheCameraParameters.isValid() && TheMarkerSize > 0)
                for (unsigned int i = 0; i < TheMarkers2.size(); i++)
                {
                    CvDrawingUtils::draw3dCube(TheInputImageCopy2, TheMarkers2[i], TheCameraParameters);
                    CvDrawingUtils::draw3dAxis(TheInputImageCopy2, TheMarkers2[i], TheCameraParameters);
                }



            // DONE! Easy, right?
            // show input with augmented information and  the thresholded image
            printInfo(TheInputImageCopy2);
            printInfo(TheInputImageCopy2);
            
            if(showMennu)printMenuInfo();
            cv::imshow("thres1", resize(MDetector1.getThresholdedImage(), 1024));
            cv::imshow("thres2", resize(MDetector2.getThresholdedImage(), 1024));
            cv::imshow("in1", TheInputImageCopy1);
            cv::imshow("in2", TheInputImageCopy2);



            double positionL[TheMarkers1.size()][3];   //stores the position from left camera //45 degree wala
            double orientationL[TheMarkers2.size()][4];  //stores the quaternion
            double positionR[TheMarkers1.size()][3];   //stores the position from right camera
            double orientationR[TheMarkers2.size()][4];  //stores the quaternion


            Eigen::Vector3d L1,L2,L3,L4,R1,R2,R3,R4;
            MatrixXf A(4,4),B1(4,1),B2(4,1),B3(4,1),C1(4,1),C2(4,1),C3(4,1);
            if((TheMarkers2.size()>3)&&(TheMarkers1.size()>3))
            {
                 int a[4],b[4],counter=0;//a and b for keeping the track that same tag is not counted in again
                for(i=0;i<4;i++)
                {
                    a[i]=0;
                    b[i]=0;
                }
                for(i=0;(i<TheMarkers1.size())&&(TheMarkers1[i].id!=a[0])&&(TheMarkers1[i].id!=a[1])&&(TheMarkers1[i].id!=a[2])&&(TheMarkers1[i].id!=a[3]);i++)
                {
                    a[i]=TheMarkers1[i].id;
                    TheMarkers1[i].OgreGetPoseParameters(positionL[i],orientationL[i]);// gets the pose of the marker
                    double* ptrL = &positionL[i][0];   //pointer to the first position
                    std::vector<double> v_positionL(ptrL , ptrL + 3);   //double array to vector of doubles
                    double* v_ptrL= &v_positionL[0];
                    Eigen::Map<Eigen::Vector3d> translationL(v_ptrL , 3);   //mapping from vevtor of doubles to eigen Vector3d
                    A(i,0)=translationL[0];
                    A(i,1)=translationL[1];
                    A(i,2)=translationL[2];
                    A(i,3)=1;
                }    
                for(j=0;(j<TheMarkers2.size())&&(TheMarkers2[j].id!=b[0])&&(TheMarkers2[j].id!=b[1])&&(TheMarkers2[j].id!=b[2])&&(TheMarkers2[j].id!=b[3]);j++)
                {
                    b[j]=TheMarkers2[j].id;
                    TheMarkers2[j].OgreGetPoseParameters(positionR[j],orientationR[j]);
                    double* ptrR = &positionR[j][0]; //pointer to the first position
                    std::vector<double> v_positionR(ptrR , ptrR + 3);   //double array to vector of doubles
                    double* v_ptrR = &v_positionR[0];
                    Eigen::Map<Eigen::Vector3d> translationR(v_ptrR , 3);   //mapping from vevtor of doubles to eigen Vector3d
                    C1(j,0)=translationR[0];
                    C2(j,0)=translationR[1];
                    C3(j,0)=translationR[2];                 
                }
                for(i=0;i<TheMarkers1.size();i++)
                    for(j=0;j<TheMarkers2.size();j++)
                        if(TheMarkers1[i].id==TheMarkers2[j].id)
                            counter++;


                if(counter>3)// to check if we do have atleast 4 markers from both the tags
                    {
                        int x=0,y=0,z=0,c=0;
                        for(i=0;i<4;i++)
                        {
                            x=x+A(i,0);
                        }
                         for(i=0;i<4;i++)
                        {
                            y=y+A(i,1);
                        }
                         for(i=0;i<4;i++)
                        {
                            z=z+A(i,2);
                        }
                        x=x/4;
                        y=y/4;
                        z=z/4;
                        for(i=0;i<4;i++)
                        {
                            A(i,0)=A(i,0)-x;
                        }
                         for(i=0;i<4;i++)
                        {
                            A(i,1)=A(i,1)-y;
                        }
                         for(i=0;i<4;i++)
                        {
                            A(i,2)=A(i,2)-z;
                        }
                        for(i=0;i<4;i++)
                        {
                            c=c+C1(i,0);
                        }
                        c=c/4;
                        for(i=0;i<4;i++)
                        {
                            C1(i,0)=C1(i,0)-c;
                        }
                        c=0;
                        for(i=0;i<4;i++)
                        {
                           c=c+C2(i,0);
                        }
                        c=c/4;
                        for(i=0;i<4;i++)
                        {
                            C2(i,0)=C2(i,0)-c;
                        }
                        c=0;
                        for(i=0;i<4;i++)
                        {
                           c=c+C3(i,0);
                        }
                        c=c/4;
                        for(i=0;i<4;i++)
                        {
                            C3(i,0)=C3(i,0)-c;
                        }
                        c=0;
                        for(i=0;i<4;i++)
                        {
                           c=c+C3(i,0);
                        }
                        c=c/4;
                        for(i=0;i<4;i++)
                        {
                            C3(i,0)=C3(i,0)-c;
                        }
                        c=0;
                        if(A.determinant()!=0)
                        {
                            B1=(A.inverse())*C1;
                            B2=(A.inverse())*C2;
                            B3=(A.inverse())*C3;
                                        
                            for(i=0;i<4;i++)
                            {
                                Transform1(0,i)=B1(i,0);
                            }
                            for(i=0;i<4;i++)
                            {
                                Transform1(1,i)=B2(i,0);
                            }
                            for(i=0;i<4;i++)
                            {
                                Transform1(2,i)=B3(i,0);
                            }
                            for(i=0;i<3;i++)
                            {
                                Transform1(3,i)=0;
                            }
                            Transform1(3,3)=1;
                            if(flag==0)
                            {
                                Transform=Transform1;
                                flag=1;
                            }
                            else
                            {
                                if(range(Transform,Transform1)==1)
                                {
                                    ofstream myfile1 ("Transformation_only_matrix.txt");
                                    flag++;
                                    Transform=(Transform*(flag-1)+Transform1)/flag;
                                    myfile << flag << endl << Transform <<endl;
                                    myfile1 << Transform <<endl;
                                    cout << flag << endl << "Transformation Matrix:" << endl << Transform << endl;
                                }
                            }
                        }
                    }
            }   
            

            key = cv::waitKey(waitTime);  // wait for key to be pressed
            if (key == 's')
                waitTime = waitTime == 0 ? 10 : 0;
            if (key == 'w'){//writes current input image
                string number=std::to_string(indexSave++);
                while(number.size()!=3)number="0"+number;
                string imname1="arucoimage"+number+".png";
                cv::imwrite(imname1,TheInputImageCopy1);
                cout<<"saved "<<imname1<<endl;
                imname1="orgimage"+number+".png";
                cv::imwrite(imname1,TheInputImage1);
                cout<<"saved "<<imname1<<endl;
                imname1="thresimage"+number+".png";
                cv::imwrite(imname1,MDetector1.getThresholdedImage());
                string imname2="arucoimage"+number+".png";
                cv::imwrite(imname2,TheInputImageCopy2);
                cout<<"saved "<<imname2<<endl;
                imname2="orgimage"+number+".png";
                cv::imwrite(imname2,TheInputImage2);
                cout<<"saved "<<imname2<<endl;
                imname2="thresimage"+number+".png";
                cv::imwrite(imname2,MDetector2.getThresholdedImage());

            }
             if (key=='m') {
                 if (showMennu)                     cv::destroyWindow("menu");
                 else {
                     cv::namedWindow("menu",cv::WINDOW_NORMAL);
                     cv::resizeWindow("menu",640,480);
                     createMenu();
                     printMenuInfo();
                 }
                showMennu=!showMennu;
            }
            if (key=='h')bPrintHelp=!bPrintHelp;

            if (key=='t'){//run a deeper speed test

                for(int t=0;t<30;t++){
                    // Detection of markers in the image passed
                    Fps.start();
                    TheMarkers1 = MDetector1.detect(TheInputImage1, TheCameraParameters, TheMarkerSize);
                    TheMarkers2 = MDetector2.detect(TheInputImage2, TheCameraParameters, TheMarkerSize);
                    Fps.stop();
                    // chekc the speed by calculating the mean speed of all iterations
                }
                printInfo(TheInputImageCopy1);
                printInfo(TheInputImageCopy2);
            }
            if(key=='f'){
                cerr<<"Configuration saved to arucoConfig.yml"<<endl;
                MDetector1.saveParamsToFile("arucoConfig.yml");
                MDetector2.saveParamsToFile("arucoConfig.yml");
            }

            index++;  // number of images captured

            if (isVideo)
                if ( (TheVideoCapturer1.grab()==false)||(TheVideoCapturer2.grab()==false)) key=27;
        } while (key != 27 );
    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}


void cvTackBarEvents(int pos, void*)
{
    (void)(pos);


    setParamsFromGlobalVariables(MDetector1);

    setParamsFromGlobalVariables(MDetector2);

    // recompute
        Fps.start();
        MDetector1.detect(TheInputImage1, TheMarkers1, TheCameraParameters);
        MDetector2.detect(TheInputImage2, TheMarkers2, TheCameraParameters);
        Fps.stop();
    // chekc the speed by calculating the mean speed of all iterations
    TheInputImage1.copyTo(TheInputImageCopy1);
    TheInputImage1.copyTo(TheInputImageCopy1);
    if (iShowAllCandidates){
        auto candidates1=MDetector1.getCandidates();
        auto candidates2=MDetector2.getCandidates();
        for(auto cand:candidates1)
            Marker(cand,-1).draw(TheInputImageCopy1, Scalar(255, 0, 255),1); 
        for(auto cand:candidates2)           
            Marker(cand,-1).draw(TheInputImageCopy2, Scalar(255, 0, 255),1);
    }

    for (unsigned int i = 0; i < TheMarkers1.size(); i++){
        //cout << TheMarkers1[i] << endl;
        TheMarkers1[i].draw(TheInputImageCopy1, Scalar(0, 0, 255),2);
    }


    for (unsigned int i = 0; i < TheMarkers2.size(); i++){
        //cout << TheMarkers2[i] << endl;
        TheMarkers2[i].draw(TheInputImageCopy2, Scalar(0, 0, 255),2);
    }
    // draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid())
        for (unsigned int i = 0; i < TheMarkers1.size(); i++)
            CvDrawingUtils::draw3dCube(TheInputImageCopy1, TheMarkers1[i], TheCameraParameters);
        for (unsigned int i = 0; i < TheMarkers2.size(); i++)            
            CvDrawingUtils::draw3dCube(TheInputImageCopy2, TheMarkers2[i], TheCameraParameters);
    cv::putText(TheInputImageCopy1,"fps="+to_string(1./Fps.getAvrg() ),cv::Point(10,20),FONT_HERSHEY_SIMPLEX, 0.5f,cv::Scalar(125,255,255),2,CV_AA);
    cv::putText(TheInputImageCopy2,"fps="+to_string(1./Fps.getAvrg() ),cv::Point(10,20),FONT_HERSHEY_SIMPLEX, 0.5f,cv::Scalar(125,255,255),2,CV_AA);

    cv::imshow("in1",  TheInputImageCopy1 );
    cv::imshow("in2",  TheInputImageCopy2 );
    cv::imshow("thres1", resize(MDetector1.getThresholdedImage(), 1024));
    cv::imshow("thres2", resize(MDetector2.getThresholdedImage(), 1024));
    if(showMennu)printMenuInfo();

}
