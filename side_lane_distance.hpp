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
#include <boost/property_tree/xml_parser.hpp>


union MultiTypeUnion {
    float f[2];
    int_least8_t c[8];
    int_least32_t i[2];
    uint_least32_t ui[2];
};

int ConsoleGraph(double value, double scale, int offset = 0, double max = 0) {
    int bar_length = value * scale + offset;
    if ((max != 0) && (bar_length > max)) { std::cout << std::endl; return 1; }
    for (int i = 0; i < value * scale; ++i) {
        std::cout << "|";
    }
    std::cout << std::endl;
    return 0;
}




class LaneDistanceDetector {
    //=============================================================================
    // パラメータ
    //-----------------------------------------------------------------------------
    int MEDIAN_WINDOW_SIZE;
    int CANNY_THRESHOLD1;
    int CANNY_THRESHOLD2;
    int HOUGH_THRESHOLD;
    int HOUGH_MIN;
    int HOUGH_MAX;
    int BINARY_THRESHOLD1;
    int BINARY_THRESHOLD2;
    cv::Point2d REFERENCE_POINT1;
    cv::Point2d REFERENCE_POINT2;
    //=============================================================================


    //=============================================================================
    // 変数宣言
    //-----------------------------------------------------------------------------
    //求めたカメラ行列を入れるMat
    cv::Mat camera_matrix;
    //求めた歪みベクトルを入れるMat
    cv::Mat dist_coeffs;
    //求めたカメラ回転ベクトルを入れるvector
    std::vector<cv::Mat> rvecs;
    //求めたカメラ並進ベクトルを入れるvector
    std::vector<cv::Mat> tvecs;
    //補正マップ
    cv::Mat remapx, remapy;
    // ARt_Z_inv
    cv::Mat ARt_Z_inv;
    //=============================================================================

    //
    cv::Mat frame;
    cv::Mat remap_image;
    cv::Mat gray_image;
    cv::Mat median_image;
    cv::Mat binary_image;
    cv::Mat differential_image;
    cv::Mat view_image;

    cv::Mat mask_image;
    cv::Mat masked_image;


    cv::VideoCapture capture;

    //=============================================================================
    // 白線認識インスタンス作成
    //-----------------------------------------------------------------------------
    std::vector<cv::Vec4i> probabilistic_hough_lines;
    //=============================================================================

    int InitDefaultParam() {
        MEDIAN_WINDOW_SIZE = 21;
        CANNY_THRESHOLD1 = 50;
        CANNY_THRESHOLD2 = 70;
        HOUGH_THRESHOLD = 120;
        HOUGH_MIN = 30;
        HOUGH_MAX = 500;
        BINARY_THRESHOLD1 = 180;
        BINARY_THRESHOLD2 = 255;
        REFERENCE_POINT1 = cv::Point2d(365.0, 364.0);
        REFERENCE_POINT2 = cv::Point2d(382.0, 134.0);
        ARt_Z_inv = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    }
    //フィルターのパラメータ読み込み
    int ReadFilterParam(std::string file_path){
        boost::property_tree::ptree pt;
        boost::property_tree::read_xml(file_path, pt);
        if (boost::optional<int> config_value = pt.get_optional<int>("root.MEDIAN_WINDOW_SIZE")) {
            MEDIAN_WINDOW_SIZE = config_value.get();
        }
        if (boost::optional<int> config_value = pt.get_optional<int>("root.CANNY_THRESHOLD1")) {
            CANNY_THRESHOLD1 = config_value.get();
        }
        if (boost::optional<int> config_value = pt.get_optional<int>("root.CANNY_THRESHOLD2")) {
            CANNY_THRESHOLD2 = config_value.get();
        }
        if (boost::optional<int> config_value = pt.get_optional<int>("root.HOUGH_THRESHOLD")) {
            HOUGH_THRESHOLD = config_value.get();
        }
        if (boost::optional<int> config_value = pt.get_optional<int>("root.HOUGH_MIN")) {
            HOUGH_MIN = config_value.get();
        }
        if (boost::optional<int> config_value = pt.get_optional<int>("root.HOUGH_MAX")) {
            HOUGH_MAX = config_value.get();
        }
        if (boost::optional<int> config_value = pt.get_optional<int>("root.BINARY_THRESHOLD1")) {
            BINARY_THRESHOLD1 = config_value.get();
        }
        if (boost::optional<int> config_value = pt.get_optional<int>("root.BINARY_THRESHOLD2")) {
            BINARY_THRESHOLD2 = config_value.get();
        }
        if (boost::optional<double> config_value = pt.get_optional<double>("root.REFERENCE_POINT1.x")) {
            REFERENCE_POINT1.x = config_value.get();
        }
        if (boost::optional<double> config_value = pt.get_optional<double>("root.REFERENCE_POINT1.y")) {
            REFERENCE_POINT1.y = config_value.get();
        }
        if (boost::optional<double> config_value = pt.get_optional<double>("root.REFERENCE_POINT2.x")) {
            REFERENCE_POINT2.x = config_value.get();
        }
        if (boost::optional<double> config_value = pt.get_optional<double>("root.REFERENCE_POINT2.y")) {
            REFERENCE_POINT2.y = config_value.get();
        }
        return 0;
    }
    //カメラパラメータ読み込み
    int ReadCameraParam(std::string internal_param_file_path, std::string external_param_file_path) {
        cv::FileStorage cvfs_internal(internal_param_file_path, cv::FileStorage::READ);
        cv::FileNode node_internal(cvfs_internal.fs, NULL);
        read(node_internal["cameraMatrix"], camera_matrix);
        read(node_internal["distCoeffs"], dist_coeffs);
        read(node_internal["remapx"], remapx);
        read(node_internal["remapy"], remapy);
        cvfs_internal.release();

        cv::FileStorage cvfs_external(external_param_file_path, cv::FileStorage::READ);
        cv::FileNode node_external(cvfs_external.fs, NULL);
        read(node_external["rvecs"], rvecs);
        read(node_external["tvecs"], tvecs);
        read(node_external["ARt_Z_inv"], ARt_Z_inv);
        cvfs_external.release();

        return 0;
    }
    int ReadMaskImage(std::string mask_image_file_path) {
        mask_image = cv::imread(mask_image_file_path, 0);
        return 0;
    }
    int InitVideoCapture(int camera_number) {
        capture = cv::VideoCapture(camera_number);
        capture >> frame;
    }
    int InitVideoCapture(std::string video_file_path) {
        capture = cv::VideoCapture(video_file_path);
        capture >> frame;
    }
    //=============================================================================
    // ワールド座標距離算出
    //
    // ****原理解説****
    // 世界座標M(X, Y, Z)と画像の座標m(u, v)の関係式はこの様になっている。
    // (fx, fyは焦点距離。cx, cyはレンズの中心の画像座標。Rは回転行列、tは並進ベクトル)
    // s*m = A[R|t]M
    //  |u| = |fx 0  cx| | r11 r12 r13 t1| |X|
    // s|v| = |0  fy cy| | r21 r22 r23 t2| |Y|
    //  |1| = |0  0  1 | | r31 r32 r33 t3| |Z|
    //                                     |1|
    //        ----------------------------
    //
    // 下線部を計算したものをARtとすると、
    //  |u| = |ARt11 ARt12 ARt13 ARt14| |X|
    // s|v| = |ARt21 ARt22 ARt23 ARt24| |Y|
    //  |1| = |ARt31 ARt32 ARt33 ARt34| |Z|
    //                                  |1|
    //
    // 世界座標のZが0とすると上記の式はこの様に考えることができる。
    //  |u| = |ARt11 ARt12 ARt14| |X|
    // s|v| = |ARt21 ARt22 ARt24| |Y|
    //  |1| = |ARt31 ARt32 ARt34| |1|
    //        -------------------
    //
    // 下線部をARt_Zとすると正方行列なので逆行列ARt_Z_invを求めることができる。
    //  |         | |u| = |X/s|
    //  |ARt_Z_inv| |v| = |Y/s|
    //  |   3x3   | |1| = |1/s|
    //
    // となり、世界座標のZが0 (つまり高さ0)という仮定のもとでなら、画像座標から世界座標のXYを求めることができる。
    //------------------------------------------------------------------------------
    int ConvertImagePoint2WorldPoint(cv::Point2d target_image_point, cv::Point2d world_point){
        double S = 0;

        cv::Mat image_point(3, 1, CV_64FC1, cv::Scalar::all(0));
        image_point.at<double>(0, 0) = target_image_point.x;
        image_point.at<double>(1, 0) = target_image_point.y;
        image_point.at<double>(2, 0) = 1.0;

        cv::Mat left_hen = ARt_Z_inv * image_point;
        if (left_hen.at<double>(2, 0) != 0) {
            S = 1.0 / left_hen.at<double>(2, 0);
            world_point.x = S * left_hen.at<double>(0, 0);
            world_point.y = S * left_hen.at<double>(1, 0);
        }
        else {
            return -1;
        }
        return 0;
    }
    double a;
    double b;
    double c;
    double d;
    cv::Point2d kouten;
    cv::Point2d kouten_true;
    cv::Point2d reference_point1;
    cv::Point2d reference_point2;
    int InitCrossLine() {
        cv::Point2d line_point1(0, 250);
        cv::Point2d line_point2(1000, 330);
        reference_point1 = REFERENCE_POINT1;
        reference_point2 = REFERENCE_POINT2;
        a = (reference_point2.y - reference_point1.y) / (reference_point2.x - reference_point1.x);
        b = -reference_point1.x * a + reference_point1.y;
        c = (line_point2.y - line_point1.y) / (line_point2.x - line_point1.x);
        d = -line_point1.x * c + line_point1.y;
        kouten = cv::Point2d((d - b) / (a - c), (a*d - b * c) / (a - c));
        kouten_true;
    }
    cv::Point2d CalcCrossPoint(cv::Point2d point1, cv::Point2d point2) {

        return cv::Point2d();
    }
public :
    LaneDistanceDetector(std::string filter_file_path, std::string internal_param_file_path, std::string externai_file_path, std::string mask_image_path, std::string video_file_path) {
        InitDefaultParam();
        ReadFilterParam(filter_file_path);
        ReadCameraParam(internal_param_file_path, externai_file_path);
        ReadMaskImage(mask_image_path);
        InitVideoCapture(video_file_path);
        ConvertImagePoint2WorldPoint(cv::Point2d(1000, 500), cv::Point2d());
    }
    LaneDistanceDetector(std::string filter_file_path, std::string internal_param_file_path, std::string externai_file_path, std::string mask_image_path, int camera_number) {
        InitDefaultParam();
        ReadFilterParam(filter_file_path);
        ReadCameraParam(internal_param_file_path, externai_file_path);
        ReadMaskImage(mask_image_path);
        InitVideoCapture(camera_number);
        ConvertImagePoint2WorldPoint(cv::Point2d(1000, 500), cv::Point2d());
    }
    int OpenParamTrackbar() {
        cv::namedWindow("trackbar", cv::WINDOW_NORMAL);
        cv::createTrackbar("median", "trackbar", &MEDIAN_WINDOW_SIZE, 100);
        cv::createTrackbar("canny1", "trackbar", &CANNY_THRESHOLD1, 300);
        cv::createTrackbar("canny2", "trackbar", &CANNY_THRESHOLD2, 300);
        cv::createTrackbar("hough_th", "trackbar", &HOUGH_THRESHOLD, 300);
        cv::createTrackbar("hough_min", "trackbar", &HOUGH_MIN, 300);
        cv::createTrackbar("hough_max", "trackbar", &HOUGH_MAX, 500);
        cv::createTrackbar("binary_threshold1", "trackbar", &BINARY_THRESHOLD1, 500);
        cv::createTrackbar("binary_threshold2", "trackbar", &BINARY_THRESHOLD2, 500);
        return 0;
    }
    int ReadFrame() {
        capture >> frame;
        return frame.empty();
    }
    int ProccessImage() {
        // 表示用に元画像コピー
        frame.copyTo(view_image);
        // 歪み補正
        cv::remap(frame, remap_image, remapx, remapy, cv::INTER_LINEAR);
        // 白黒へ変換
        cv::cvtColor(remap_image, gray_image, CV_RGB2GRAY);
        // メディアンフィルター
        cv::medianBlur(gray_image, median_image, 11);
        // 2値化
        cv::threshold(median_image, binary_image, BINARY_THRESHOLD1, BINARY_THRESHOLD2, cv::THRESH_BINARY);
        // 微分（エッジ検出）
        cv::Canny(binary_image, differential_image, CANNY_THRESHOLD1, CANNY_THRESHOLD2);
        // マスク処理
        differential_image.copyTo(masked_image, mask_image);
        // ハフ変換（直線検出）                                             分解能、        閾値,          最小長,     最大長
        cv::HoughLinesP(masked_image, probabilistic_hough_lines, 1.0, CV_PI / 180, HOUGH_THRESHOLD, HOUGH_MIN, HOUGH_MAX);
        for (int i = 0; i < probabilistic_hough_lines.size(); ++i) {
            // 直線の傾き計算
            if (probabilistic_hough_lines[i][2] - probabilistic_hough_lines[i][0] == 0) { continue; }
            c = double(probabilistic_hough_lines[i][3] - probabilistic_hough_lines[i][1]) / double(probabilistic_hough_lines[i][2] - probabilistic_hough_lines[i][0]);
            d = -probabilistic_hough_lines[i][0] * c + probabilistic_hough_lines[i][1];
            // 交点計算
            kouten = cv::Point2d((d - b) / (a - c), (a*d - b * c) / (a - c));
            // 一番車両に近い交点抽出
            if ((kouten.y > kouten_true.y) && (540 > kouten.y)) {
                kouten_true = kouten;
            }
            // 表示
            cv::line(view_image, cv::Point(probabilistic_hough_lines[i][0], probabilistic_hough_lines[i][1]),
                                 cv::Point(probabilistic_hough_lines[i][2], probabilistic_hough_lines[i][3]), cv::Scalar(0, 0, 255), 1, CV_AA);
            cv::line(view_image, reference_point1, reference_point2, cv::Scalar(255, 255, 0), 1, CV_AA);
        }
        cv::circle(view_image, kouten_true, 3, cv::Scalar(0, 255, 0), 3);
        // 画像座標から世界座標算出
        ConvertImagePoint2WorldPoint(kouten_true, cv::Point());
        // 変数初期化
        kouten_true = cv::Point2d(0.0, -1000000000000.0);

        return 0;
    }
    int ViewImage() {
        cv::imshow("view_image", view_image);
        cv::imshow("gray", gray_image);
        cv::imshow("median", median_image);
        cv::imshow("binary", binary_image);
        cv::imshow("differencial", differential_image);
        cv::imshow("mask", mask_image);
        cv::imshow("masked", masked_image);
    }
    int ViewImage(int view = 0, int gray = 0, int median = 0, int binary = 0, int differencial = 0, int mask = 0, int masked = 0) {
        if (view) { cv::imshow("view_image", view_image); }
        if (gray) { cv::imshow("gray", gray_image); }
        if (median) { cv::imshow("median", median_image); }
        if (binary) { cv::imshow("binary", binary_image); }
        if (differencial) { cv::imshow("differencial", differential_image); }
        if (mask) { cv::imshow("mask", mask_image); }
        if (masked) { cv::imshow("masked", masked_image); }
    }
    int OutputConsoleLog() {
        //cout << "gazou " << kouten_true_l.x << " " << kouten_true_l.y << endl;
        //std::cout << world_point.y;
        //ConsoleGraph(Y_l, 0.1, 100, 190);
    }

};