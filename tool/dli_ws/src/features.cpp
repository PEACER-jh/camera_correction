#include "../include/features.hpp"

void computeFeatures_ORB(const cv::Mat& l, const cv::Mat& r)
{
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat dst1, dst2;
    cv::Ptr<cv::FeatureDetector>     detector   = cv::ORB::create();    //创建一个指向ORB特征检测器对象的指针
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();    //创建一个指向ORB描述符提取器对象的指针
    cv::Ptr<cv::DescriptorMatcher>   matcher    = cv::DescriptorMatcher::create("BruteForce-Hamming");  //基于Hamming距离的描述符匹配器

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //1、检测Oriented FAST角点位置
    detector->detect(l, kp1);
    detector->detect(r, kp2);
    //2、根据角点位置计算BRIEF描述子
    descriptor->compute(l, kp1, dst1);
    descriptor->compute(r, kp2, dst2);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "extract ORB cost = " << time_used.count() << " s" << std::endl;

    // 绘制角点
    cv::Mat out1, out2;
    // cv::namedWindow("ORB features 1", cv::WINDOW_FREERATIO);
    // cv::namedWindow("ORB features 2", cv::WINDOW_FREERATIO);
    cv::drawKeypoints(l, kp1, out1, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DEFAULT);
    cv::drawKeypoints(l, kp2, out2, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DEFAULT);
    // cv::imshow("ORB features 1", out1);
    // cv::imshow("ORB features 2", out2);

    std::vector<cv::DMatch> mt;
    t1 = std::chrono::steady_clock::now();
    //3、使用汉明距离对两幅图像中的BRIEF描述子进行匹配
    matcher->match(dst1, dst2, mt);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "match ORB cost   = " << time_used.count() << " s" << std::endl;

    //4、匹配点对筛选
    //计算最小和最大距离
    auto min_max = minmax_element(mt.begin(), mt.end(),
                                  [](const cv::DMatch &m1, const cv::DMatch &m2) 
                                    {return m1.distance < m2.distance;});
    double min = min_max.first->distance;
    double max = min_max.second->distance;
    // printf("    Min distance = %lf \n", min);
    // printf("    Max distance = %lf \n", max);
    //当描述子之间的距离大于两倍的最小距离时，即认为匹配有误
	//但有时最小距离会非常小，所以设置一个30经验值为下限
    int count = 0;
    std::vector<cv::DMatch> mt_y;
    for(int i = 0; i < dst1.rows; i++)
    {
        if(mt[i].distance <= MAX(2*min, 30.0))
        {
            mt_y.push_back(mt[i]);
            count++;
        }
        if(count == 20)
            break;
    }

    // 绘制匹配结果
    cv::Mat dst_match_all, dst_match_yes;
    cv::drawMatches(l, kp1, r, kp2, mt, dst_match_all);
    cv::drawMatches(l, kp1, r, kp2, mt_y, dst_match_yes);
    // cv::namedWindow("all result", cv::WINDOW_FREERATIO);
    cv::namedWindow("yes result", cv::WINDOW_FREERATIO);
    // cv::imshow("all result", dst_match_all);
    cv::imshow("yes result", dst_match_yes);

    cv::waitKey(100);
}