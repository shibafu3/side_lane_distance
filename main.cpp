#ifdef __WIN32
#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"D:/home/Sources/opencv/opencv310/build/install/lib/opencv_world310d.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"D:/home/Sources/opencv/opencv310/build/install/lib/opencv_world310.lib") 
#endif

#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"D:/home/Sources/CANUSB/lib/canusbdrv64.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"D:/home/Sources/CANUSB/lib/canusbdrv64.lib")
#endif
#endif

#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
/*OpenCVライブラリ*/
#include <opencv2/opencv.hpp>
//Matとか行列モジュール
#include <opencv2/core/core.hpp>
//GUIモジュール
#include <opencv2/highgui/highgui.hpp>
//キャリブレーションモジュール
#include <opencv2/calib3d/calib3d.hpp>


using namespace std;
using namespace cv;

union MultiTypeUnion {
    float f[2];
    int_least8_t c[8];
    int_least32_t i[2];
    uint_least32_t ui[2];
};

int ConsoleGraph(double value, double scale, int offset = 0, double max = 0) {
    int bar_length = value * scale + offset;
    if ((max != 0) && (bar_length > max)) { cout << endl; return 1; }
    for (int i = 0; i < value * scale; ++i) {
        cout << "|";
    }
    cout << endl;
    return 0;
}

int main() {
    //=============================================================================
    // 設定値定義
    //-----------------------------------------------------------------------------
    int MEDIAN_WINDOW_SIZE_L = 21;
    int CANNY_THRESHOLD1_L = 50;
    int CANNY_THRESHOLD2_L = 70;
    int HOUGH_THRESHOLD_L = 120;
    int HOUGH_MIN_L = 30;
    int HOUGH_MAX_L = 500;
    int BINARY_THRESHOLD1_L = 180;
    int BINARY_THRESHOLD2_L = 255;
    Point2d REFERENCE_POINT1_L(365, 364);
    Point2d REFERENCE_POINT2_L(382, 134);

    int MEDIAN_WINDOW_SIZE_R = 21;
    int CANNY_THRESHOLD1_R = 50;
    int CANNY_THRESHOLD2_R = 70;
    int HOUGH_THRESHOLD_R = 120;
    int HOUGH_MIN_R = 30;
    int HOUGH_MAX_R = 500;
    int BINARY_THRESHOLD1_R = 180;
    int BINARY_THRESHOLD2_R = 255;
    Point2d REFERENCE_POINT1_R(365, 364);
    Point2d REFERENCE_POINT2_R(382, 134);

    int START_FRAME_COUNT = 10;
    int END_FRAME_COUNT = 200;
    //=============================================================================



    //=============================================================================
    // ファイルから設定値を読み込み
    //-----------------------------------------------------------------------------
    ifstream canny_ifs_l("C:/Users/0133752/Desktop/workspace/process_param/canny_l.txt");
    ifstream hough_ifs_l("C:/Users/0133752/Desktop/workspace/process_param/hough_l.txt");
    ifstream median_ifs_l("C:/Users/0133752/Desktop/workspace/process_param/median_l");
    ifstream line_ifs_l("C:/Users/0133752/Desktop/workspace/vehicle_param/line_l.txt");
    ifstream binary_ifs_l("C:/Users/0133752/Desktop/workspace/process_param/binary_l.txt");

    string temp_string;
    getline(canny_ifs_l, temp_string);
    CANNY_THRESHOLD1_L = stoi(temp_string);
    getline(canny_ifs_l, temp_string);
    CANNY_THRESHOLD2_L = stoi(temp_string);

    getline(median_ifs_l, temp_string);
    MEDIAN_WINDOW_SIZE_L = stoi(temp_string);

    getline(hough_ifs_l, temp_string);
    HOUGH_THRESHOLD_L = stoi(temp_string);
    getline(hough_ifs_l, temp_string);
    HOUGH_MIN_L = stoi(temp_string);
    getline(hough_ifs_l, temp_string);
    HOUGH_MAX_L = stoi(temp_string);

    getline(line_ifs_l, temp_string);
    REFERENCE_POINT1_L.x = stoi(temp_string);
    getline(line_ifs_l, temp_string);
    REFERENCE_POINT1_L.y = stoi(temp_string);
    getline(line_ifs_l, temp_string);
    REFERENCE_POINT2_L.x = stoi(temp_string);
    getline(line_ifs_l, temp_string);
    REFERENCE_POINT2_L.y = stoi(temp_string);

    getline(binary_ifs_l, temp_string);
    BINARY_THRESHOLD1_L = stoi(temp_string);
    getline(binary_ifs_l, temp_string);
    BINARY_THRESHOLD2_L = stoi(temp_string);


    ifstream canny_ifs_r("C:/Users/0133752/Desktop/workspace/process_param/canny_r.txt");
    ifstream hough_ifs_r("C:/Users/0133752/Desktop/workspace/process_param/hough_r.txt");
    ifstream median_ifs_r("C:/Users/0133752/Desktop/workspace/process_param/median_r");
    ifstream line_ifs_r("C:/Users/0133752/Desktop/workspace/vehicle_param/line_r.txt");
    ifstream binary_ifs_r("C:/Users/0133752/Desktop/workspace/process_param/binary_r.txt");


    getline(canny_ifs_r, temp_string);
    CANNY_THRESHOLD1_R = stoi(temp_string);
    getline(canny_ifs_r, temp_string);
    CANNY_THRESHOLD2_R = stoi(temp_string);

    getline(median_ifs_r, temp_string);
    MEDIAN_WINDOW_SIZE_R = stoi(temp_string);

    getline(hough_ifs_r, temp_string);
    HOUGH_THRESHOLD_R = stoi(temp_string);
    getline(hough_ifs_r, temp_string);
    HOUGH_MIN_R = stoi(temp_string);
    getline(hough_ifs_r, temp_string);
    HOUGH_MAX_R = stoi(temp_string);

    getline(line_ifs_r, temp_string);
    REFERENCE_POINT1_R.x = stoi(temp_string);
    getline(line_ifs_r, temp_string);
    REFERENCE_POINT1_R.y = stoi(temp_string);
    getline(line_ifs_r, temp_string);
    REFERENCE_POINT2_R.x = stoi(temp_string);
    getline(line_ifs_r, temp_string);
    REFERENCE_POINT2_R.y = stoi(temp_string);

    getline(binary_ifs_r, temp_string);
    BINARY_THRESHOLD1_R = stoi(temp_string);
    getline(binary_ifs_r, temp_string);
    BINARY_THRESHOLD2_R = stoi(temp_string);


    ifstream frame_ifs("C:/Users/0133752/Desktop/workspace/frame.txt");

    getline(frame_ifs, temp_string);
    START_FRAME_COUNT = stoi(temp_string);
    getline(frame_ifs, temp_string);
    END_FRAME_COUNT = stoi(temp_string);
    //=============================================================================



    //=============================================================================
    // 設定値書き出し
    //-----------------------------------------------------------------------------
    ofstream canny_ofs_l("C:/Users/0133752/Desktop/canny_l.txt");
    ofstream hough_ofs_l("C:/Users/0133752/Desktop/hough_l.txt");
    ofstream median_ofs_l("C:/Users/0133752/Desktop/median_l.txt");
    ofstream line_ofs_l("C:/Users/0133752/Desktop/line_l.txt");
    ofstream binary_ofs_l("C:/Users/0133752/Desktop/binary_l.txt");

    ofstream canny_ofs_r("C:/Users/0133752/Desktop/canny_r.txt");
    ofstream hough_ofs_r("C:/Users/0133752/Desktop/hough_r.txt");
    ofstream median_ofs_r("C:/Users/0133752/Desktop/median_r.txt");
    ofstream line_ofs_r("C:/Users/0133752/Desktop/line_r.txt");
    ofstream binary_ofs_r("C:/Users/0133752/Desktop/binary_r.txt");

    ofstream frame_ofs("C:/Users/0133752/Desktop/frame.txt");
    //=============================================================================



    //=============================================================================
    // 変数宣言
    //-----------------------------------------------------------------------------
    //求めたカメラ行列を入れるMat
    Mat camera_matrix_l;
    //求めた歪みベクトルを入れるMat
    Mat dist_coeffs_l;
    //求めたカメラ回転ベクトルを入れるvector
    vector<Mat> rvecs_l;
    //求めたカメラ並進ベクトルを入れるvector
    vector<Mat> tvecs_l;
    //補正マップ
    Mat remapx_l, remapy_l;
    // ロドリゲスから一列抜いたやつ
    Mat rtvecs_l(3, 3, CV_64FC1, Scalar::all(0));

    //求めたカメラ行列を入れるMat
    Mat camera_matrix_r;
    //求めた歪みベクトルを入れるMat
    Mat dist_coeffs_r;
    //求めたカメラ回転ベクトルを入れるvector
    vector<Mat> rvecs_r;
    //求めたカメラ並進ベクトルを入れるvector
    vector<Mat> tvecs_r;
    //補正マップ
    Mat remapx_r, remapy_r;
    // ロドリゲスから一列抜いたやつ
    Mat rtvecs_r(3, 3, CV_64FC1, Scalar::all(0));
    //=============================================================================



    //=============================================================================
    // ファイル読み込み
    //-----------------------------------------------------------------------------
    FileStorage cvfs_internal_l("C:/Users/0133752/Desktop/workspace/vehicle_param/calibrate_l.xml", FileStorage::READ);
    FileNode node_internal_l(cvfs_internal_l.fs, NULL);
    read(node_internal_l["cameraMatrix"], camera_matrix_l);
    read(node_internal_l["distCoeffs"], dist_coeffs_l);
    read(node_internal_l["remapx"], remapx_l);
    read(node_internal_l["remapy"], remapy_l);

    FileStorage cvfs_external_l("C:/Users/0133752/Desktop/workspace/vehicle_param/rtvecs_l.xml", FileStorage::READ);
    FileNode node_external_l(cvfs_external_l.fs, NULL);
    read(node_external_l["rvecs"], rvecs_l);
    read(node_external_l["tvecs"], tvecs_l);
    read(node_external_l["rtvecs"], rtvecs_l);


    FileStorage cvfs_internal_r("C:/Users/0133752/Desktop/workspace/vehicle_param/calibrate_r.xml", FileStorage::READ);
    FileNode node_internal_r(cvfs_internal_r.fs, NULL);
    read(node_internal_r["cameraMatrix"], camera_matrix_r);
    read(node_internal_r["distCoeffs"], dist_coeffs_r);
    read(node_internal_r["remapx"], remapx_r);
    read(node_internal_r["remapy"], remapy_r);

    FileStorage cvfs_external_r("C:/Users/0133752/Desktop/workspace/vehicle_param/rtvecs_r.xml", FileStorage::READ);
    FileNode node_external_r(cvfs_external_r.fs, NULL);
    read(node_external_r["rvecs"], rvecs_r);
    read(node_external_r["tvecs"], tvecs_r);
    read(node_external_r["rtvecs"], rtvecs_r);
    //=============================================================================



    //=============================================================================
    // カメラ初期化
    //-----------------------------------------------------------------------------
    Mat frame_l;
    Mat remap_image_l;
    Mat gray_image_l;
    Mat median_image_l;
    Mat binary_image_l;
    Mat differential_image_l;
    Mat view_image_l;

    VideoCapture cap_l("C:/Users/0133752/Desktop/workspace/l.avi");
    cap_l >> frame_l;

    Mat mask_image_l;
    Mat masked_image_l;
    mask_image_l = imread("C:/Users/0133752/Desktop/workspace/vehicle_param/mask_l.bmp", 0);


    Mat frame_r;
    Mat remap_image_r;
    Mat gray_image_r;
    Mat median_image_r;
    Mat binary_image_r;
    Mat differential_image_r;
    Mat view_image_r;

    VideoCapture cap_r("C:/Users/0133752/Desktop/workspace/r.avi");
    cap_r >> frame_r;

    Mat mask_image_r;
    Mat masked_image_r;
    mask_image_r = imread("C:/Users/0133752/Desktop/workspace/vehicle_param/mask_r.bmp", 0);
    //=============================================================================



    //=============================================================================
    // 白線認識インスタンス作成
    //-----------------------------------------------------------------------------
    vector<Vec4i> probabilistic_hough_lines_l;

    vector<Vec4i> probabilistic_hough_lines_r;
    //=============================================================================



    //=============================================================================
    // 距離計算用ベクトル
    //-----------------------------------------------------------------------------
    Mat image_point_l(3, 1, CV_64FC1, Scalar::all(0));
    image_point_l.at<double>(2, 0) = 1;
    //Mat world_point(3, 1, CV_64FC1, Scalar::all(0));
    //world_point.at<double>(2, 0) = 1;
    Mat left_hen_l = rtvecs_l.inv() * image_point_l;
    double S_l = 0, X_l = 0, Y_l = 0;

    image_point_l.at<double>(0, 0) = 1000;
    image_point_l.at<double>(1, 0) = 500;
    left_hen_l = rtvecs_l.inv() * image_point_l;

    if (left_hen_l.at<double>(2, 0) == 0) {
        S_l = 1.0 / left_hen_l.at<double>(2, 0);
        X_l = S_l * left_hen_l.at<double>(0, 0);
        Y_l = S_l * left_hen_l.at<double>(1, 0);
    }
    else {
        S_l = 1.0 / left_hen_l.at<double>(2, 0);
        X_l = S_l * left_hen_l.at<double>(0, 0);
        Y_l = S_l * left_hen_l.at<double>(1, 0);
    }


    Mat image_point_r(3, 1, CV_64FC1, Scalar::all(0));
    image_point_r.at<double>(2, 0) = 1;
    //Mat world_point(3, 1, CV_64FC1, Scalar::all(0));
    //world_point.at<double>(2, 0) = 1;
    Mat left_hen_r = rtvecs_r.inv() * image_point_r;
    double S_r = 0, X_r = 0, Y_r = 0;

    image_point_r.at<double>(0, 0) = 1000;
    image_point_r.at<double>(1, 0) = 500;
    left_hen_r = rtvecs_r.inv() * image_point_r;

    if (left_hen_r.at<double>(2, 0) == 0) {
        S_r = 1.0 / left_hen_r.at<double>(2, 0);
        X_r = S_r * left_hen_r.at<double>(0, 0);
        Y_r = S_r * left_hen_r.at<double>(1, 0);
    }
    else {
        S_r = 1.0 / left_hen_r.at<double>(2, 0);
        X_r = S_r * left_hen_r.at<double>(0, 0);
        Y_r = S_r * left_hen_r.at<double>(1, 0);
    }
    //=============================================================================



    //=============================================================================
    // 交点検出用変数
    //-----------------------------------------------------------------------------
    Point2d reference_point1_l = REFERENCE_POINT1_L;
    Point2d reference_point2_l = REFERENCE_POINT2_L;
    double a_l = (reference_point2_l.y - reference_point1_l.y) / (reference_point2_l.x - reference_point1_l.x);
    double b_l = -reference_point1_l.x * a_l + reference_point1_l.y;
    Point2d line_point1_l(0, 250);
    Point2d line_point2_l(1000, 330);
    double c_l = (line_point2_l.y - line_point1_l.y) / (line_point2_l.x - line_point1_l.x);
    double d_l = -line_point1_l.x * c_l + line_point1_l.y;
    Point2d kouten_l = Point2d((d_l - b_l) / (a_l - c_l), (a_l*d_l - b_l * c_l) / (a_l - c_l));
    Point2d kouten_true_l;

    Point2d reference_point1_r = REFERENCE_POINT1_R;
    Point2d reference_point2_r = REFERENCE_POINT2_R;
    double a_r = (reference_point2_r.y - reference_point1_r.y) / (reference_point2_r.x - reference_point1_r.x);
    double b_r = -reference_point1_r.x * a_r + reference_point1_r.y;
    Point2d line_point1_r(0, 250);
    Point2d line_point2_r(1000, 330);
    double c_r = (line_point2_r.y - line_point1_r.y) / (line_point2_r.x - line_point1_r.x);
    double d_r = -line_point1_r.x * c_r + line_point1_r.y;
    Point2d kouten_r = Point2d((d_r - b_r) / (a_r - c_r), (a_r*d_r - b_r * c_r) / (a_r - c_r));
    Point2d kouten_true_r;
    //=============================================================================



    //=============================================================================
    // CAN 初期化
    //-----------------------------------------------------------------------------
    MultiTypeUnion b2f_l;
    b2f_l.ui[0] = b2f_l.ui[1] = 0;

    MultiTypeUnion b2f_r;
    b2f_r.ui[0] = b2f_r.ui[1] = 0;
    //=============================================================================



    //=============================================================================
    // GPU使う用宣言
    //-----------------------------------------------------------------------------
    //=============================================================================



    //=============================================================================
    // タイム測定用変数宣言
    //-----------------------------------------------------------------------------
    //=============================================================================



    //=============================================================================
    // スタートフレーム設定
    //-----------------------------------------------------------------------------
    int start_frame_count_l = START_FRAME_COUNT;
    int end_frame_count_l = END_FRAME_COUNT;
    ++b2f_l.ui[0] = start_frame_count_l;
    cap_l.set(CV_CAP_PROP_POS_FRAMES, start_frame_count_l);

    int start_frame_count_r = START_FRAME_COUNT;
    int end_frame_count_r = END_FRAME_COUNT;
    ++b2f_r.ui[0] = start_frame_count_r;
    cap_r.set(CV_CAP_PROP_POS_FRAMES, start_frame_count_r);
    //=============================================================================



    //=============================================================================
    // GUI
    //-----------------------------------------------------------------------------
    namedWindow("trackbar", WINDOW_NORMAL);
    createTrackbar("median_l", "trackbar", &MEDIAN_WINDOW_SIZE_L, 100);
    createTrackbar("canny1_l", "trackbar", &CANNY_THRESHOLD1_L, 300);
    createTrackbar("canny2_l", "trackbar", &CANNY_THRESHOLD2_L, 300);
    createTrackbar("hough_th_l", "trackbar", &HOUGH_THRESHOLD_L, 300);
    createTrackbar("hough_min_l", "trackbar", &HOUGH_MIN_L, 300);
    createTrackbar("hough_max_l", "trackbar", &HOUGH_MAX_L, 500);
    createTrackbar("binary_threshold1_l", "trackbar", &BINARY_THRESHOLD1_L, 500);
    createTrackbar("binary_threshold2_l", "trackbar", &BINARY_THRESHOLD2_L, 500);

    createTrackbar("median_r", "trackbar", &MEDIAN_WINDOW_SIZE_R, 100);
    createTrackbar("canny1_r", "trackbar", &CANNY_THRESHOLD1_R, 300);
    createTrackbar("canny2_r", "trackbar", &CANNY_THRESHOLD2_R, 300);
    createTrackbar("hough_th_r", "trackbar", &HOUGH_THRESHOLD_R, 300);
    createTrackbar("hough_min_r", "trackbar", &HOUGH_MIN_R, 300);
    createTrackbar("hough_max_r", "trackbar", &HOUGH_MAX_R, 500);
    createTrackbar("binary_threshold1_r", "trackbar", &BINARY_THRESHOLD1_R, 500);
    createTrackbar("binary_threshold2_r", "trackbar", &BINARY_THRESHOLD2_R, 500);
    //=============================================================================

    double temp_l = 0;
    double temp_r = 0;
    int pause = 0;
    int key = 0;
    while (1) {
        //=============================================================================
        // タイム測定
        //-----------------------------------------------------------------------------
            //=============================================================================



            //=============================================================================
            // 画像取り込み 範囲切り取り
            //-----------------------------------------------------------------------------
            if (pause == 0) {
                cap_l >> frame_l; if (frame_l.empty()) { break; }
                cap_r >> frame_r; if (frame_r.empty()) { break; }
            }
            if (key == 59) {
                cap_l.set(CV_CAP_PROP_POS_FRAMES, cap_l.get(CV_CAP_PROP_POS_FRAMES) - 10);
                cap_l >> frame_l; if (frame_l.empty()) { break; }
                cap_r.set(CV_CAP_PROP_POS_FRAMES, cap_r.get(CV_CAP_PROP_POS_FRAMES) - 10);
                cap_r >> frame_r; if (frame_r.empty()) { break; }
            }
            if (key == 39) {
                cap_l.set(CV_CAP_PROP_POS_FRAMES, cap_l.get(CV_CAP_PROP_POS_FRAMES) + 10);
                cap_l >> frame_l; if (frame_l.empty()) { break; }
                cap_r.set(CV_CAP_PROP_POS_FRAMES, cap_r.get(CV_CAP_PROP_POS_FRAMES) + 10);
                cap_r >> frame_r; if (frame_r.empty()) { break; }
            }

            frame_l.copyTo(view_image_l);
            remap(frame_l, remap_image_l, remapx_l, remapy_l, INTER_LINEAR);

            frame_r.copyTo(view_image_r);
            remap(frame_r, remap_image_r, remapx_r, remapy_r, INTER_LINEAR);

            //gframe_l.upload(frame_l);
            //cuda::remap(gframe_l, gremap_image_l, xmap_l, ymap_l, INTER_LINEAR);

            //gframe_r.upload(frame_r);
            //cuda::remap(gframe_r, gremap_image_r, xmap_r, ymap_r, INTER_LINEAR);
            //=============================================================================



            //=============================================================================
            // 色変換
            //-----------------------------------------------------------------------------
            cvtColor(remap_image_l, gray_image_l, CV_RGB2GRAY);

            cvtColor(remap_image_r, gray_image_r, CV_RGB2GRAY);


            //cuda::cvtColor(gremap_image_l, ggray_image_l, CV_RGB2GRAY);

            //cuda::cvtColor(gremap_image_r, ggray_image_r, CV_RGB2GRAY);
            //=============================================================================



            //=============================================================================
            // 中間値フィルター
            //-----------------------------------------------------------------------------
            medianBlur(gray_image_l, median_image_l, 11);

            medianBlur(gray_image_r, median_image_r, 11);

            //ggray_image_l.upload(gray_image_l);
            //gmedian_l->apply(ggray_image_l, gmedian_image_l);
            //gmedian_image_l.download(median_image_l);

            //ggray_image_r.upload(gray_image_r);
            //gmedian_r->apply(ggray_image_r, gmedian_image_r);
            //gmedian_image_r.download(median_image_r);
            //=============================================================================



            //=============================================================================
            // 二値化
            //-----------------------------------------------------------------------------
            threshold(median_image_l, binary_image_l, BINARY_THRESHOLD1_L, BINARY_THRESHOLD2_L, THRESH_BINARY);

            threshold(median_image_r, binary_image_r, BINARY_THRESHOLD1_R, BINARY_THRESHOLD2_R, THRESH_BINARY);
            //=============================================================================



            //=============================================================================
            // 微分
            //-----------------------------------------------------------------------------
            Canny(binary_image_l, differential_image_l, CANNY_THRESHOLD1_L, CANNY_THRESHOLD2_L);

            Canny(binary_image_r, differential_image_r, CANNY_THRESHOLD1_R, CANNY_THRESHOLD2_R);

            //gcanny_l->detect(gmedian_image_l, gdifferential_image_l);

            //gcanny_r->detect(gmedian_image_r, gdifferential_image_r);
            //=============================================================================



            //=============================================================================
            // マスク処理
            //-----------------------------------------------------------------------------
            differential_image_l.copyTo(masked_image_l, mask_image_l);

            differential_image_r.copyTo(masked_image_r, mask_image_r);


            //gdifferential_image_l.copyTo(gmasked_image_l, gmask_image_l);

            //gdifferential_image_r.copyTo(gmasked_image_r, gmask_image_r);
            //=============================================================================



            //=============================================================================
            // ハフ変換 交点検出
            //-----------------------------------------------------------------------------
            //                                                              分解能、  閾値,最小長,最大長
            HoughLinesP(masked_image_l, probabilistic_hough_lines_l, 1.0, CV_PI / 180, HOUGH_THRESHOLD_L, HOUGH_MIN_L, HOUGH_MAX_L);
            for (int i = 0; i < probabilistic_hough_lines_l.size(); ++i) {
                // 直線の傾き計算
                if (probabilistic_hough_lines_l[i][2] - probabilistic_hough_lines_l[i][0] == 0) { continue; }
                c_l = double(probabilistic_hough_lines_l[i][3] - probabilistic_hough_lines_l[i][1]) / double(probabilistic_hough_lines_l[i][2] - probabilistic_hough_lines_l[i][0]);
                d_l = -probabilistic_hough_lines_l[i][0] * c_l + probabilistic_hough_lines_l[i][1];
                // 交点検出
                kouten_l = Point2d((d_l - b_l) / (a_l - c_l), (a_l*d_l - b_l * c_l) / (a_l - c_l));
                if ((kouten_l.y > kouten_true_l.y) && (540 > kouten_l.y)) {
                    kouten_true_l = kouten_l;
                }
                // 表示
                line(view_image_l, Point(probabilistic_hough_lines_l[i][0], probabilistic_hough_lines_l[i][1]),
                    Point(probabilistic_hough_lines_l[i][2], probabilistic_hough_lines_l[i][3]), Scalar(0, 0, 255), 1, CV_AA);
                line(view_image_l, reference_point1_l, reference_point2_l, Scalar(255, 255, 0), 1, CV_AA);
            }
            circle(view_image_l, kouten_true_l, 3, Scalar(0, 255, 0), 3);


            HoughLinesP(masked_image_r, probabilistic_hough_lines_r, 1.0, CV_PI / 180, HOUGH_THRESHOLD_L, HOUGH_MIN_L, HOUGH_MAX_L);
            for (int i = 0; i < probabilistic_hough_lines_r.size(); ++i) {
                // 直線の傾き計算
                if (probabilistic_hough_lines_r[i][2] - probabilistic_hough_lines_r[i][0] == 0) { continue; }
                c_r = double(probabilistic_hough_lines_r[i][3] - probabilistic_hough_lines_r[i][1]) / double(probabilistic_hough_lines_r[i][2] - probabilistic_hough_lines_r[i][0]);
                d_r = -probabilistic_hough_lines_r[i][0] * c_r + probabilistic_hough_lines_r[i][1];
                // 交点検出
                kouten_r = Point2d((d_r - b_r) / (a_r - c_r), (a_r*d_r - b_r * c_r) / (a_r - c_r));
                if ((kouten_r.y > kouten_true_r.y) && (540 > kouten_r.y)) {
                    kouten_true_r = kouten_r;
                }
                // 表示
                line(view_image_r, Point(probabilistic_hough_lines_r[i][0], probabilistic_hough_lines_r[i][1]),
                    Point(probabilistic_hough_lines_r[i][2], probabilistic_hough_lines_r[i][3]), Scalar(0, 0, 255), 1, CV_AA);
                line(view_image_r, reference_point1_r, reference_point2_r, Scalar(255, 255, 0), 1, CV_AA);
            }
            circle(view_image_r, kouten_true_r, 3, Scalar(0, 255, 0), 3);


            //ghough_l->detect(gmasked_image_l, ghough_result_l);
            //ghough_l->downloadResults(ghough_result_l, ghough_lines_l);
            //for (int i = 0; i < ghough_lines_l.size(); ++i) {
            //    float rho = ghough_lines_l[i][0], theta = ghough_lines_l[i][1];
            //    Point pt1, pt2;
            //    double a_l = cos(theta), b_l = sin(theta);
            //    double x0 = a_l*rho, y0 = b_l*rho;
            //    pt1.x = cvRound(x0 + 1000 * (-b_l));
            //    pt1.y = cvRound(y0 + 1000 * (a_l));
            //    pt2.x = cvRound(x0 - 1000 * (-b_l));
            //    pt2.y = cvRound(y0 - 1000 * (a_l));

            //    // 直線の傾き計算
            //    if (pt2.x - pt1.x == 0) { continue; }
            //    c_l = double(pt2.y - pt1.y) / double(pt2.x - pt1.x);
            //    d_l = -pt1.x * c_l + pt1.y;
            //    // 交点検出
            //    kouten_l = Point2d((d_l - b_l) / (a_l - c_l), (a_l*d_l - b_l*c_l) / (a_l - c_l));
            //    if ((kouten_l.y > kouten_true_l.y) && (540 > kouten_l.y)) {
            //        kouten_true_l = kouten_l;
            //    }
            //    // 表示
            //    line(frame_l, Point(pt1.x, pt1.y),
            //        Point(pt2.x, pt2.y), Scalar(0, 0, 255), 1, CV_AA);
            //    line(frame_l, reference_point1_l, reference_point2_l, Scalar(255, 255, 0), 1, CV_AA);
            //}
            //circle(frame_l, kouten_true_l, 3, Scalar(0, 255, 0), 3);


            //ghough_r->detect(gmasked_image_r, ghough_result_r);
            //ghough_r->downloadResults(ghough_result_r, ghough_lines_r);
            //for (int i = 0; i < ghough_lines_r.size(); ++i) {
            //    float rho = ghough_lines_r[i][0], theta = ghough_lines_r[i][1];
            //    Point pt1, pt2;
            //    double a_r = cos(theta), b_r = sin(theta);
            //    double x0 = a_r*rho, y0 = b_r*rho;
            //    pt1.x = cvRound(x0 + 1000 * (-b_r));
            //    pt1.y = cvRound(y0 + 1000 * (a_r));
            //    pt2.x = cvRound(x0 - 1000 * (-b_r));
            //    pt2.y = cvRound(y0 - 1000 * (a_r));

            //    // 直線の傾き計算
            //    if (pt2.x - pt1.x == 0) { continue; }
            //    c_r = double(pt2.y - pt1.y) / double(pt2.x - pt1.x);
            //    d_r = -pt1.x * c_r + pt1.y;
            //    // 交点検出
            //    kouten_r = Point2d((d_r - b_r) / (a_r - c_r), (a_r*d_r - b_r*c_r) / (a_r - c_r));
            //    if ((kouten_r.y > kouten_true_r.y) && (540 > kouten_r.y)) {
            //        kouten_true_r = kouten_r;
            //    }
            //    // 表示
            //    line(frame_r, Point(pt1.x, pt1.y),
            //        Point(pt2.x, pt2.y), Scalar(0, 0, 255), 1, CV_AA);
            //    line(frame_r, reference_point1_r, reference_point2_r, Scalar(255, 255, 0), 1, CV_AA);
            //}
            //circle(frame_r, kouten_true_r, 3, Scalar(0, 255, 0), 3);

            //=============================================================================



            //=============================================================================
            // ワールド座標距離算出
            //-----------------------------------------------------------------------------
            image_point_l.at<double>(0, 0) = kouten_true_l.x;
            image_point_l.at<double>(1, 0) = kouten_true_l.y;
            left_hen_l = rtvecs_l.inv() * image_point_l;

            if (left_hen_l.at<double>(2, 0) == 0) {
                S_l = 1.0 / left_hen_l.at<double>(2, 0);
                X_l = S_l * left_hen_l.at<double>(0, 0);
                Y_l = S_l * left_hen_l.at<double>(1, 0);
            }
            else {
                S_l = 1.0 / left_hen_l.at<double>(2, 0);
                X_l = S_l * left_hen_l.at<double>(0, 0);
                Y_l = S_l * left_hen_l.at<double>(1, 0);
            }

            image_point_r.at<double>(0, 0) = kouten_true_r.x;
            image_point_r.at<double>(1, 0) = kouten_true_r.y;
            left_hen_r = rtvecs_r.inv() * image_point_r;

            if (left_hen_r.at<double>(2, 0) == 0) {
                S_r = 1.0 / left_hen_r.at<double>(2, 0);
                X_r = S_r * left_hen_r.at<double>(0, 0);
                Y_r = S_r * left_hen_r.at<double>(1, 0);
            }
            else {
                S_r = 1.0 / left_hen_r.at<double>(2, 0);
                X_r = S_r * left_hen_r.at<double>(0, 0);
                Y_r = S_r * left_hen_r.at<double>(1, 0);
            }
            //=============================================================================



            //=============================================================================
            // 変数初期化
            //-----------------------------------------------------------------------------
            kouten_true_l = Point2d(0.0, -1000000000000.0);
            kouten_true_r = Point2d(0.0, -1000000000000.0);
            //=============================================================================



            //=============================================================================
            // 表示
            //-----------------------------------------------------------------------------
            //cout << "gazou " << kouten_true_l.x << " " << kouten_true_l.y << endl;
            cout << Y_l;
            //ConsoleGraph(Y_l, 0.1, 100, 190);
            //=============================================================================



            //=============================================================================
            // 画像表示
            //-----------------------------------------------------------------------------
            imshow("view_image_l", view_image_l);
            imshow("gray_l", gray_image_l);
            imshow("median_l", median_image_l);
            //imshow("binary_l", binary_image_l);
            imshow("differencial_l", differential_image_l);
            imshow("mask_l", mask_image_l);
            imshow("masked_l", masked_image_l);

            imshow("view_image_r", view_image_r);
            imshow("gray_r", gray_image_r);
            imshow("median_r", median_image_r);
            //imshow("binary_r", binary_image_r);
            imshow("differencial_r", differential_image_r);
            imshow("mask_r", mask_image_r);
            imshow("masked_r", masked_image_r);
            if ((key = waitKey(1)) == 27) { break; }
            else if ((key == 32) && (pause == 0)) { pause = 1; }
            else if ((key == 32) && (pause == 1)) { pause = 0; }
            //=============================================================================



            //=============================================================================
            // タイム測定
            //-----------------------------------------------------------------------------
        //=============================================================================
    }


    //=============================================================================
    // 終了処理
    //-----------------------------------------------------------------------------
    canny_ofs_l << CANNY_THRESHOLD1_L << endl;
    canny_ofs_l << CANNY_THRESHOLD2_L;
    hough_ofs_l << HOUGH_THRESHOLD_L << endl;
    hough_ofs_l << HOUGH_MIN_L << endl;
    hough_ofs_l << HOUGH_MAX_L;
    median_ofs_l << MEDIAN_WINDOW_SIZE_L;
    line_ofs_l << REFERENCE_POINT1_L.x << endl;
    line_ofs_l << REFERENCE_POINT1_L.y << endl;
    line_ofs_l << REFERENCE_POINT2_L.x << endl;
    line_ofs_l << REFERENCE_POINT2_L.y;
    binary_ofs_l << BINARY_THRESHOLD1_L << endl;
    binary_ofs_l << BINARY_THRESHOLD2_L;

    canny_ofs_r << CANNY_THRESHOLD1_R << endl;
    canny_ofs_r << CANNY_THRESHOLD2_R;
    hough_ofs_r << HOUGH_THRESHOLD_R << endl;
    hough_ofs_r << HOUGH_MIN_R << endl;
    hough_ofs_r << HOUGH_MAX_R;
    median_ofs_r << MEDIAN_WINDOW_SIZE_R;
    line_ofs_r << REFERENCE_POINT1_R.x << endl;
    line_ofs_r << REFERENCE_POINT1_R.y << endl;
    line_ofs_r << REFERENCE_POINT2_R.x << endl;
    line_ofs_r << REFERENCE_POINT2_R.y;
    binary_ofs_r << BINARY_THRESHOLD1_R << endl;
    binary_ofs_r << BINARY_THRESHOLD2_R;

    frame_ofs << START_FRAME_COUNT << endl;
    frame_ofs << END_FRAME_COUNT;

    return 0;
    //=============================================================================
}

