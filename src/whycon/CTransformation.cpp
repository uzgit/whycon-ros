#include <cstdio>
#include <cmath>

#include "whycon/CTransformation.h"

/*
 * File name: CTransformation.h
 * Date:      2014
 * Author:   Tom Krajnik, Matias Nitsche
 * Description: Transforms detected ellipse position and dimensions to arbitrary 3D or 2D coordinate frame. The method is described in Chapter 4 of the article [1].
 * Licence: if you use this class for your research, please cite [1].
 * References: [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 */

namespace whycon
{

CTransformation::CTransformation(float circle_diam) :
    transform_type_(TRANSFORM_NONE),
    circle_diameter_(circle_diam)
{
    intrinsic_mat_ = cv::Mat::eye(3,3, CV_32FC1);
    distortion_coeffs_ = cv::Mat::zeros(1,5, CV_32FC1);
}

CTransformation::~CTransformation()
{
}

void CTransformation::setTransformType(const ETransformType trans_type)
{
    if(calibrated_ || trans_type == TRANSFORM_NONE)
    {
        transform_type_ = trans_type;
    }
    else
    {
        throw std::runtime_error("Calibrated coordinate system is not avaiable. Either load it or create it.");
    }
}

ETransformType CTransformation::getTransformType()
{
    return transform_type_;
}

void CTransformation::setCircleDiameter(const float circle_diam)
{
    circle_diameter_ = circle_diam;
}

void CTransformation::updateCameraParams(const double *intri, const double* dist)
{
    for(int i = 0; i < 5; i++)
        distortion_coeffs_.at<float>(i) = dist[i];

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
            intrinsic_mat_.at<float>(i, j) = intri[3 * i + j];
    }
}

void CTransformation::updateCameraParams(cv::Mat intri, cv::Mat dist)
{
    intrinsic_mat_ = intri;
    distortion_coeffs_ = dist;
}

void CTransformation::reTransformXY(float &x, float &y, float &z)
{
    static cv::Mat coords = cv::Mat(3, 1, CV_32FC1);
    static cv::Mat result = cv::Mat(2, 1, CV_32FC1);

    coords.at<float>(0) = x;
    coords.at<float>(1) = y;
    coords.at<float>(2) = z;

    cv::projectPoints(coords, cv::Mat::zeros(3, 1, CV_32FC1), cv::Mat::zeros(3, 1, CV_32FC1), intrinsic_mat_, distortion_coeffs_, result);

    x = result.at<float>(0);
    y = result.at<float>(1);
    z = 0;
}

void CTransformation::transformXY(float &x, float &y)
{
    static cv::Mat coords = cv::Mat(2, 1, CV_32FC1);
    static cv::Mat result = cv::Mat(2, 1, CV_32FC1);

    coords.at<float>(0) = x;
    coords.at<float>(1) = y;

    cv::undistortPoints(coords, result, intrinsic_mat_, distortion_coeffs_);

    x = result.at<float>(0);
    y = result.at<float>(1);
}

void CTransformation::transform2D(STrackedObject &o)
{
    float x = hom_[0] * o.x + hom_[1] * o.y + hom_[2];
    float y = hom_[3] * o.x + hom_[4] * o.y + hom_[5];
    float z = hom_[6] * o.x + hom_[7] * o.y + hom_[8];

    o.x = x / z;
    o.y = y / z;
    o.z = 0;
}

void CTransformation::transform3D(STrackedObject &o, const int num)
{
    /* [3] = {x, y, z} */
    float a[3];
    float final[3] = {0.0, 0.0, 0.0};

    /* result[4][3] = {{x1, y1, z1}, {x2, y2, z2}, {x3, y3, z3}, {x4, y4, z4}} */
    float result[4][3];

    float str = 0;
    float str_all = 0;

    for (int k = 0; k < num; k++)
    {
        a[0] = o.x - D3transform_[k].orig[0];
        a[1] = o.y - D3transform_[k].orig[1];
        a[2] = o.z - D3transform_[k].orig[2];
        
        result[k][0] = D3transform_[k].simlar[0] * a[0] + D3transform_[k].simlar[1] * a[1] + D3transform_[k].simlar[2] * a[2];
        result[k][1] = D3transform_[k].simlar[3] * a[0] + D3transform_[k].simlar[4] * a[1] + D3transform_[k].simlar[5] * a[2];
        result[k][2] = D3transform_[k].simlar[6] * a[0] + D3transform_[k].simlar[7] * a[1] + D3transform_[k].simlar[8] * a[2];
        
        result[k][0] = (k % 2) * grid_dim_x_ + (1 - (k % 2) * 2) * result[k][0];
        result[k][1] = (k / 2) * grid_dim_y_ + (1 - (k / 2) * 2) * result[k][1];
        if (k == 0 || k == 3)
            result[k][2] = -result[k][2];

        str = 1.0 / (a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + 0.01);

        final[0] += str * result[k][0];
        final[1] += str * result[k][1];
        final[2] += str * result[k][2];
        str_all += str;
        // std::printf("UUU: %f %f %f %f\n", result[k][0], result[k][1], result[k][2], str);
    }

    o.x = final[0] / str_all;
    o.y = final[1] / str_all;
    o.z = final[2] / str_all;
}

void CTransformation::loadCalibration(const std::string &str)
{
    try
    {
        cv::FileStorage fs(str, cv::FileStorage::READ);
        if(!fs.isOpened())
            throw std::runtime_error("Could not open/load calibration file.");

        fs["dim_x"] >> grid_dim_x_;
        fs["dim_y"] >> grid_dim_y_;

        cv::Mat hom_tmp(3, 3, CV_32FC1);
        fs["hom"] >> hom_tmp;
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
                hom_[3 * i + j] = hom_tmp.at<float>(i, j);
        }

        for(int k = 0; k < 4; k++)
        {
            cv::Mat offset_tmp(3, 1, CV_32FC1);
            fs["offset_" + std::to_string(k)] >> offset_tmp;
            for(int i = 0; i < 3; i++)
                D3transform_[k].orig[i] = offset_tmp.at<float>(i);

            cv::Mat simlar_tmp(3, 3, CV_32FC1);
            fs["simlar_" + std::to_string(k)] >> simlar_tmp;
            for(int i = 0; i < 3; i++)
            {
                for(int j = 0; j < 3; j++)
                    D3transform_[k].simlar[3 * i + j] = simlar_tmp.at<float>(i, j);
            }
        }

        fs.release();
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void CTransformation::saveCalibration(const std::string &str)
{
    try
    {
        cv::FileStorage fs(str, cv::FileStorage::WRITE);
        if(!fs.isOpened())
            throw std::runtime_error("Could not open/create calibration file.");

        fs.writeComment("Dimensions");
        fs << "dim_x" << grid_dim_x_;
        fs << "dim_y" << grid_dim_y_;
        fs.writeComment("2D calibration");
        fs << "hom" << cv::Mat(3, 3, CV_32FC1, hom_);
        fs.writeComment("3D calibration");

        for (int k = 0; k < 4; k++)
        {
            fs.writeComment("D3transform " + std::to_string(k));
            fs << "offset_" + std::to_string(k) << cv::Mat(3, 1, CV_32FC1, D3transform_[k].orig);
            fs << "simlar_" + std::to_string(k) << cv::Mat(3, 3, CV_32FC1, D3transform_[k].simlar);
        }

        fs.release();
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void CTransformation::calibrate2D(const STrackedObject *in, const float g_dim_x, const float g_dim_y, const float robot_radius, const float robot_height, const float camera_height)
{
    /*specific to the pheromone system:
        compensates the fact, that the calibration patterns are displayed in a lower position
        than the robots assumes that the camera above the field centre*/
    float ix = g_dim_x / camera_height * robot_height / 2;
    float iy = g_dim_y / camera_height * robot_height / 2;
    // float ix = g_dim_x / (in[0].x + in[1].x + in[2].x + in[3].x) * 4 * off;
    // float iy = g_dim_y / (in[0].x + in[1].x + in[2].x + in[3].x) * 4 * off;

    /* r[4][2] = {{x1, y1}, {x2, y2}, {x3, y3}, {x4, y4}} */
    float r[4][2];
    r[0][0] = robot_radius + ix;
    r[0][1] = robot_radius + iy;
    r[1][0] = g_dim_x - robot_radius - ix;
    r[1][1] = robot_radius + iy;
    r[2][0] = robot_radius + ix;
    r[2][1] = g_dim_y - robot_radius - iy;
    r[3][0] = g_dim_x - robot_radius - ix;
    r[3][1] = g_dim_y - robot_radius - iy;

    /* o[4][2] = {{x1, y1}, {x2, y2}, {x3, y3}, {x4, y4}} */
    float o[4][2];
    for(int i = 0; i < 4; i++)
    {
        o[i][0] = -in[i].y / in[i].x;
        o[i][1] = -in[i].z / in[i].x;
    }

    cv::Matx<float, 8, 8> est;
    cv::Matx<float, 8, 1> vec;
    cv::Matx<float, 8, 1> res;
    for(int i = 0; i < 4; i++)
    {
        est(2 * i, 0) = -o[i][0];
        est(2 * i, 1) = -o[i][1];
        est(2 * i, 2) = -1;
        est(2 * i, 3) = 0;
        est(2 * i, 4) = 0;
        est(2 * i, 5) = 0;
        est(2 * i, 6) = r[i][0] * o[i][0];
        est(2 * i, 7) = r[i][0] * o[i][1];
        est(2 * i + 1, 0) = 0;
        est(2 * i + 1, 1) = 0;
        est(2 * i + 1, 2) = 0;
        est(2 * i + 1, 3) = -o[i][0];
        est(2 * i + 1, 4) = -o[i][1];
        est(2 * i + 1, 5) = -1;
        est(2 * i + 1, 6) = r[i][1] * o[i][0];
        est(2 * i + 1, 7) = r[i][1] * o[i][1];
        vec(2 * i, 0) = -r[i][0];
        vec(2 * i + 1, 0) = -r[i][1];
    }
    cv::solve(est, vec, res);

    for(int i = 0; i < 8; i++)
        hom_[i] = res(i);
    hom_[8] = 1;

    transform_type_ = TRANSFORM_2D;
}

void CTransformation::calibrate3D(const STrackedObject *o, const float g_dim_x, const float g_dim_y)
{
    D3transform_[0] = calibrate3D(o[0], o[1], o[2], g_dim_x, g_dim_y);
    D3transform_[1] = calibrate3D(o[1], o[0], o[3], g_dim_x, g_dim_y);
    D3transform_[2] = calibrate3D(o[2], o[3], o[0], g_dim_x, g_dim_y);
    D3transform_[3] = calibrate3D(o[3], o[2], o[1], g_dim_x, g_dim_y);
    grid_dim_x_ = g_dim_x;
    grid_dim_y_ = g_dim_y;
    transform_type_ = TRANSFORM_3D;
}

S3DTransform CTransformation::calibrate3D(const STrackedObject &o0, const STrackedObject &o1, const STrackedObject &o2, const float g_dim_x, const float g_dim_y)
{
    S3DTransform result;
    result.orig[0] = o0.x;
    result.orig[1] = o0.y;
    result.orig[2] = o0.z;

    // float scale = 1.0;
    /* v[3] = {x, y, z} */
    float v0[3] = {o1.x - o0.x, o1.y - o0.y, o1.z - o0.z};
    /*scale = sqrt(v0[0] * v0[0] + v0[1] * v0[1] + v0[2] * v0[2]);
    v0[0] /= scale;
    v0[1] /= scale;
    v0[2] /= scale;*/

    float v1[3] = {o2.x - o0.x, o2.y - o0.y, o2.z - o0.z};
    /*scale = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
    v1[0] /= scale;
    v1[1] /= scale;
    v1[2] /= scale;*/

    float v2[3] = {v0[1] * v1[2] - v1[1] * v0[2], v0[2] * v1[0] - v1[2] * v0[0], v0[0] * v1[1] - v1[0] * v0[1]};
    /*scale = sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
    v2[0] /= scale;
    v2[1] /= scale;
    v2[2] /= scale;*/

    cv::Matx33f m23D(v0[0], v1[0], v2[0],
                     v0[1], v1[1], v2[1],
                     v0[2], v1[2], v2[2]);
    m23D = m23D.inv();

    result.simlar[0] = m23D(0, 0) * g_dim_x;
    result.simlar[1] = m23D(0, 1) * g_dim_x;
    result.simlar[2] = m23D(0, 2) * g_dim_x;

    result.simlar[3] = m23D(1, 0) * g_dim_y;
    result.simlar[4] = m23D(1, 1) * g_dim_y;
    result.simlar[5] = m23D(1, 2) * g_dim_y;

    result.simlar[6] = m23D(2, 0) * g_dim_x * g_dim_y;
    result.simlar[7] = m23D(2, 1) * g_dim_x * g_dim_y;
    result.simlar[8] = m23D(2, 2) * g_dim_x * g_dim_y;

    transform_type_ = TRANSFORM_3D;
    return result;
}

SEllipseCenters CTransformation::calcEigen(const float *data)
{
    SEllipseCenters result;
    cv::Matx31f val;
    cv::Matx33f vec;
    cv::Matx33f in(data);

    cv::eigen(in, val, vec);

    // eigenvalues
    float L0 = val(0);
    float L1 = val(1);
    float L2 = val(2);

    // eigenvectors
    int V0 = 0;
    int V2 = 2;

    // partial calculation
    float c0 = std::sqrt((L0 - L1) / (L0 - L2));
    float c0x = c0 * vec(V0, 0);
    float c0y = c0 * vec(V0, 1);
    float c0z = c0 * vec(V0, 2);
    float c1 = std::sqrt((L1 - L2) / (L0 - L2));
    float c1x = c1 * vec(V2, 0);
    float c1y = c1 * vec(V2, 1);
    float c1z = c1 * vec(V2, 2);
    float c2 = circle_diameter_ / std::sqrt(-L0 * L2) / 2.0;

    // sign arrays
    static float s0[8] = {1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0};
    static float s1[8] = {1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0};
    static float s2[8] = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0};

    float n0, n1, n2, t0, t1, t2;
    int idx = 0;

    // solving for all solutions
    for(int i = 0; i < 8; i++)
    {
        n2 = s0[i] * c0z + s1[i] * c1z;
        t2 = s2[i] * c2 * (s0[i] * L2 * c0z + s1[i] * L0 * c1z);

        // restrictions on orientations
        if(n2 < 0 && t2 > 0)
        {
            n0 = s0[i] * c0x + s1[i] * c1x;
            n1 = s0[i] * c0y + s1[i] * c1y;

            t0 = s2[i] * c2 * (s0[i] * L2 * c0x + s1[i] * L0 * c1x);
            t1 = s2[i] * c2 * (s0[i] * L2 * c0y + s1[i] * L0 * c1y);

            result.n[idx][0] = n0;
            result.n[idx][1] = n1;
            result.n[idx][2] = n2;

            /* image coords -> camera coords
                   t2  ~  z -> x
                  -t0  ~ -x -> y
                  -t1  ~ -y -> z */
            result.t[idx][0] = t2;
            result.t[idx][1] = -t0;
            result.t[idx][2] = -t1;

            reTransformXY(t0, t1, t2);
            result.u[idx] = t0;
            result.v[idx] = t1;

            idx++;

            // std::printf("calcEigen %d %.3f %.3f %.3f %.3f %.3f %.3f\n", idx, t0, t1, t2, n0, n1, n2);
        }
    }

    return result;
}

SEllipseCenters CTransformation::calcSolutions(const SSegment segment)
{
    float x, y, x1, x2, y1, y2, major, minor, v0, v1;

    /* transformation to the canonical camera coordinates, see 4.1 of [1] */
    x = segment.x;
    y = segment.y;
    transformXY(x, y);

    // major axis
    // vertices in image coords
    x1 = segment.x + segment.v0 * segment.m0 * 2.0;
    x2 = segment.x - segment.v0 * segment.m0 * 2.0;
    y1 = segment.y + segment.v1 * segment.m0 * 2.0;
    y2 = segment.y - segment.v1 * segment.m0 * 2.0;
    // vertices in canonical camera coords
    transformXY(x1, y1);
    transformXY(x2, y2);
    // semiaxes length
    major = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) / 2.0;
    v0 = (x2 - x1) / major / 2.0;
    v1 = (y2 - y1) / major / 2.0;
    // std::printf("AAA: %f %f\n", std::sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)) - sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2)), major);

    // the minor axis
    // vertices in image coords
    x1 = segment.x + segment.v1 * segment.m1 * 2.0;
    x2 = segment.x - segment.v1 * segment.m1 * 2.0;
    y1 = segment.y - segment.v0 * segment.m1 * 2.0;
    y2 = segment.y + segment.v0 * segment.m1 * 2.0;
    // vertices in canonical camera coords
    transformXY(x1, y1);
    transformXY(x2, y2);
    // minor axis length
    minor = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) / 2.0;
    // std::printf("BBB: %f %f\n", std::sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)) - sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2)), minor);

    /* construct the ellipse characteristic equation, see 4.2 of [1], prepare coefs for Eq */
    float a, b, c, d, e, f;
    a = v0 * v0 / (major * major) + v1 * v1 / (minor * minor);
    b = v0 * v1 * (1.0 / (major * major) - 1.0 / (minor * minor));
    c = v0 * v0 / (minor * minor) + v1 * v1 / (major * major);
    d = (-x * a - b * y);
    e = (-y * c - b * x);
    f = (a * x * x + c * y * y + 2.0 * b * x * y - 1.0);
    float data[9] = {a, b, d, b, c, e, d, e, f};  // matrix conic coefficients, see 4.2 of [1]

    return calcEigen(data);
}

void CTransformation::transformCoordinates(STrackedObject &obj)
{
    /* transformation to camera-centric or user-defined coordinate frames */
    switch(transform_type_)
    {
        // 3D->2D homography, see 4.4.2 of [1]
        case TRANSFORM_2D:
        {
            /*TODO note #09*/
            //for debug only
            //obj.x = x;
            //obj.y = y;
            transform2D(obj);
            break;
        }
        // camera-centric coordinate frame, see 4.3 and 4.4 of [1]
        default:
        case TRANSFORM_NONE:
        {
            break;
        }
        // user-defined 3D coordinate system, see 4.4.1 of [1]
        case TRANSFORM_3D:
        {
            transform3D(obj);
            break;
        }
    }
}

void CTransformation::calcOrientation(STrackedObject &obj)
{
    calcQuaternion(obj);
    calcEulerFromQuat(obj);
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
// https://en.wikipedia.org/wiki/Quaternion#Hamilton_product
void CTransformation::calcQuaternion(STrackedObject &obj)
{
    cv::Vec3f initial_norm(0.0, 0.0, 1.0);
    cv::Vec3f final_norm(obj.n0, obj.n1, obj.n2);
    cv::Vec3f axis_vec = final_norm.cross(initial_norm);
    cv::normalize(axis_vec, axis_vec);

    float rot_angle = -std::acos(final_norm.dot(initial_norm));
    //std::printf("rot_angle %.3f\n", rot_angle);
    float s = std::sin(rot_angle / 2.0);
    float qx1 = axis_vec[0] * s;
    float qy1 = axis_vec[1] * s;
    float qz1 = axis_vec[2] * s;
    float qw1 = std::cos(rot_angle / 2.0);

    /* float scale = std::sqrt(qx1*qx1 + qy1*qy1 + qz1*qz1 + qw1*qw1);
    std::printf("quat1 norm %.3f\n", scale);
    if(std::fabs(scale - 1) > 1e-6){
        qx1 /= scale;
        qy1 /= scale;
        qz1 /= scale;
        qw1 /= scale;
    }*/

    obj.qx = qx1;
    obj.qy = qy1;
    obj.qz = qz1;
    obj.qw = qw1;

    std::printf("angle %f\n", obj.angle);
    s = std::sin(obj.angle / 2.0);
    float qx2 = final_norm[0] * s;
    float qy2 = final_norm[1] * s;
    float qz2 = final_norm[2] * s;
    float qw2 = std::cos(obj.angle / 2.0);

    /* scale = std::sqrt(qx2*qx2 + qy2*qy2 + qz2*qz2 + qw2*qw2);
    std::printf("quat2 norm %f\n", scale);
    if(std::fabs(scale - 1) > 1e-6){
        qx2 /= scale;
        qy2 /= scale;
        qz2 /= scale;
        qw2 /= scale;
    }*/

    /* Hamilton product: at first applying q1 and then rotation q2 */
    obj.qx = qw2 * qx1 + qx2 * qw1 + qy2 * qz1 - qz2 * qy1;
    obj.qy = qw2 * qy1 - qx2 * qz1 + qy2 * qw1 + qz2 * qx1;
    obj.qz = qw2 * qz1 + qx2 * qy1 - qy2 * qx1 + qz2 * qw1;
    obj.qw = qw2 * qw1 - qx2 * qx1 - qy2 * qy1 - qz2 * qz1;

    /* scale = std::sqrt(obj.qx*obj.qx + obj.qy*obj.qy + obj.qz*obj.qz + obj.qw*obj.qw);
    std::printf("quatFin norm %f\n", scale);
    if(std::fabs(scale - 1) > 1e-6){
        obj.qx /= scale;
        obj.qy /= scale;
        obj.qz /= scale;
        obj.qw /= scale;
    }*/
}

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void CTransformation::calcEulerFromQuat(STrackedObject &obj)
{
    // roll (x-axis rotation)
    obj.roll = std::atan2(2.0 * (obj.qw * obj.qx + obj.qy * obj.qz), 1.0 - 2.0 * (obj.qx * obj.qx + obj.qy * obj.qy));

    // pitch (y-axis rotation)
    float sinp = 2.0 * (obj.qw * obj.qy - obj.qz * obj.qx);
    if (std::fabs(sinp) >= 1)
        obj.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        obj.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    obj.yaw = std::atan2(2.0 * (obj.qw * obj.qz + obj.qx * obj.qy), 1.0 - 2.0 * (obj.qy * obj.qy + obj.qz * obj.qz));

    // std::printf("roll %.3f pitch %.3f yaw %.3f\n", obj.roll, obj.pitch, obj.yaw);
}

}