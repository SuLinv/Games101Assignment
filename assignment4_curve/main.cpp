#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

// 代数的方法
void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        // 伯恩斯坦多项式作为系数的线性组合
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255; // 设置颜色
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    // 如果只有两个点，跳出递归，返回两个点的插值后的新控制点
    if(control_points.size() == 2)
        return (1-t)*control_points[0] + t*control_points[1];
    std::vector<cv::Point2f> contorl_points_new; // 每次插值后的新控制点序列
    // 遍历所有序列，存入n-1个插值后的新控制点序列
    for(int i = 0;i < control_points.size() - 1;++i){
        contorl_points_new.push_back((1-t)*control_points[i] + t*control_points[i+1]);
    }
    // 递归
    return recursive_bezier(contorl_points_new, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(double t = 0; t <= 1; t += 0.001){
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255; // 设置颜色

        // 以下为削减走样
        std::vector<cv::Point2f> points(4); // 采样点周围四个像素点
        points[0] = cv::Point2f(std::floor(point.x + 0.5), std::floor(point.y + 0.5));
        points[1] = cv::Point2f(std::floor(point.x + 0.5), std::floor(point.y - 0.5));
        points[2] = cv::Point2f(std::floor(point.x - 0.5), std::floor(point.y + 0.5));
        points[3] = cv::Point2f(std::floor(point.x - 0.5), std::floor(point.y - 0.5));
    
        cv::Point2f distance = points[0] - point; //距离采样点最近的像素点坐标是+0.5后的点
        float d0 = sqrt(distance.x * distance.x + distance.y * distance.y); //取这个距离作为参照
        
        for (auto p : points)
        {
            cv::Point2f d = p - point;
            float percnet = d0 / sqrt(d.x * d.x + d.y * d.y); //计算其他像素点与最近距离的比值
            float color = window.at<cv::Vec3b>(p.y, p.x)[1];
            color = std::max(color, 255 * percnet);//取最大值效果更好
            window.at<cv::Vec3b>(p.y, p.x)[1] = color;
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
