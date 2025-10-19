//
//
//// CLinf_HDC_Extract_Normals.cpp
//// 运行环境：VS2022(C++20) / PCL 1.14.1 / OpenCV 4.10
//// 功能：从BMP切片构建体数据，执行 CLinf-HDC 点云提取，计算并保存法向（原始与下采样均含法向）。
//// 说明：不使用 try/catch；对边界和数值病态做显式保护；中文注释详述关键步骤。
//
//#include <opencv2/opencv.hpp>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/filters/voxel_grid.h>
//
//#include <filesystem>
//#include <string>
//#include <vector>
//#include <array>
//#include <iostream>
//#include <fstream>
//#include <cmath>
//#include <numeric>
//#include <limits>
//
//using std::size_t;
//namespace fs = std::filesystem;
//
//// ---------------------- 基本向量与体数据结构 ----------------------
//struct Vec3 {
//    double x, y, z;
//    Vec3() : x(0), y(0), z(0) {}
//    Vec3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
//    inline Vec3 operator+(const Vec3& b) const { return Vec3(x + b.x, y + b.y, z + b.z); }
//    inline Vec3 operator-(const Vec3& b) const { return Vec3(x - b.x, y - b.y, z - b.z); }
//    inline Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
//    inline Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }
//    inline Vec3& operator+=(const Vec3& b) { x += b.x; y += b.y; z += b.z; return *this; }
//};
//static inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
//static inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
//static inline Vec3   normalize(const Vec3& a) { double n = norm(a); return (n > 0) ? (a / n) : Vec3(0, 0, 0); }
//
//struct EdgeIntersect {
//    Vec3 pos;   // 亚体素交点  p_e
//    Vec3 nrm;   // 法线（由梯度插值得到并单位化）
//    double b;   // 平面常数 b = n·p
//};
//
//struct Volume {
//    int W = 0, H = 0, D = 0;              // 体素维度
//    std::vector<float> data;        // 标量体
//    inline size_t idx(int x, int y, int z) const { return (size_t)z * H * W + (size_t)y * W + (size_t)x; }
//    inline bool inside(int x, int y, int z) const { return (x >= 0 && x < W && y >= 0 && y < H && z >= 0 && z < D); }
//    inline float val(int x, int y, int z) const { return data[idx(x, y, z)]; }
//};
//
//// ---------------------- 工具 ----------------------
//static void ensure_dir(const std::string& path) { fs::path p(path); if (!fs::exists(p)) fs::create_directories(p); }
//
//// ---------------------- Otsu 阈值 ----------------------
//static double otsu_threshold_from_hist(const std::vector<uint64_t>& hist, uint32_t levels) {
//    const uint64_t total = std::accumulate(hist.begin(), hist.end(), uint64_t(0));
//    if (total == 0) return 0.0;
//    double sum = 0.0; for (uint32_t i = 0; i < levels; ++i) sum += double(i) * double(hist[i]);
//    double sumB = 0.0; uint64_t wB = 0; double varMax = -1.0; uint32_t best = 0;
//    for (uint32_t t = 0; t < levels; ++t) {
//        wB += hist[t]; if (wB == 0) continue;
//        uint64_t wF = total - wB; if (wF == 0) break;
//        sumB += double(t) * double(hist[t]);
//        double mB = sumB / double(wB), mF = (sum - sumB) / double(wF);
//        double varBetween = double(wB) * double(wF) * (mB - mF) * (mB - mF);
//        if (varBetween > varMax) { varMax = varBetween; best = t; }
//    }
//    return double(best);
//}
//
//// ---------------------- 三维梯度（有限差分） ----------------------
//static Vec3 gradient_at(const Volume& vol, int x, int y, int z) {
//    const int xm1 = (x > 0) ? x - 1 : x, xp1 = (x < vol.W - 1) ? x + 1 : x;
//    const int ym1 = (y > 0) ? y - 1 : y, yp1 = (y < vol.H - 1) ? y + 1 : y;
//    const int zm1 = (z > 0) ? z - 1 : z, zp1 = (z < vol.D - 1) ? z + 1 : z;
//    const double fx = (x > 0 && x < vol.W - 1) ? (vol.val(xp1, y, z) - vol.val(xm1, y, z)) * 0.5 : (vol.val(xp1, y, z) - vol.val(x, y, z));
//    const double fy = (y > 0 && y < vol.H - 1) ? (vol.val(x, yp1, z) - vol.val(x, ym1, z)) * 0.5 : (vol.val(x, yp1, z) - vol.val(x, y, z));
//    const double fz = (z > 0 && z < vol.D - 1) ? (vol.val(x, y, zp1) - vol.val(x, y, zm1)) * 0.5 : (vol.val(x, y, zp1) - vol.val(x, y, z));
//    return Vec3(fx, fy, fz);
//}
//
//// 三线性插值梯度
//static Vec3 trilinear_interp_grad(
//    const Vec3 g000, const Vec3 g100, const Vec3 g110, const Vec3 g010,
//    const Vec3 g001, const Vec3 g101, const Vec3 g111, const Vec3 g011,
//    double fx, double fy, double fz)
//{
//    auto lerp = [&](const Vec3& a, const Vec3& b, double t) { return a * (1.0 - t) + b * t; };
//    Vec3 g00 = lerp(g000, g100, fx), g10 = lerp(g010, g110, fx), g01 = lerp(g001, g101, fx), g11 = lerp(g011, g111, fx);
//    Vec3 g0 = lerp(g00, g10, fy), g1 = lerp(g01, g11, fy);
//    return lerp(g0, g1, fz);
//}
//
//// 在任意点 (px,py,pz) 处由梯度场计算法向（在其所在单元做三线性插值）
//static Vec3 normal_at_point(const Volume& vol, const std::vector<Vec3>& grad, double px, double py, double pz) {
//    const int bx = std::clamp(int(std::floor(px)), 0, vol.W - 2);
//    const int by = std::clamp(int(std::floor(py)), 0, vol.H - 2);
//    const int bz = std::clamp(int(std::floor(pz)), 0, vol.D - 2);
//    const double rx = px - bx, ry = py - by, rz = pz - bz;
//
//    Vec3 g000 = grad[vol.idx(bx + 0, by + 0, bz + 0)];
//    Vec3 g100 = grad[vol.idx(bx + 1, by + 0, bz + 0)];
//    Vec3 g110 = grad[vol.idx(bx + 1, by + 1, bz + 0)];
//    Vec3 g010 = grad[vol.idx(bx + 0, by + 1, bz + 0)];
//    Vec3 g001 = grad[vol.idx(bx + 0, by + 0, bz + 1)];
//    Vec3 g101 = grad[vol.idx(bx + 1, by + 0, bz + 1)];
//    Vec3 g111 = grad[vol.idx(bx + 1, by + 1, bz + 1)];
//    Vec3 g011 = grad[vol.idx(bx + 0, by + 1, bz + 1)];
//
//    return normalize(trilinear_interp_grad(g000, g100, g110, g010, g001, g101, g111, g011, rx, ry, rz));
//}
//
//// ---------------------- 立方体几何定义 ----------------------
//static const int cornerOffset[8][3] = { {0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1} };
//static const int edgeCorners[12][2] = { {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7} };
//
//// ---------------------- Hermite 根定位（牛顿+二分） ----------------------
//static double hermite_root_find(double f0, double f1, double d0, double d1, double iso, int maxIter = 20) {
//    const double y0 = f0 - iso, y1 = f1 - iso;
//    if (std::abs(y0) < 1e-12) return 0.0;
//    if (std::abs(y1) < 1e-12) return 1.0;
//    if (y0 * y1 > 0.0) { // 退化回退线性
//        const double denom = (f1 - f0); if (std::abs(denom) < 1e-12) return 0.5;
//        double s = (iso - f0) / denom; if (s < 0.0) s = 0.0; if (s > 1.0) s = 1.0; return s;
//    }
//    auto H = [&](double s)->double {
//        double s2 = s * s, s3 = s2 * s;
//        double h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s, h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;
//        return h00 * f0 + h10 * d0 + h01 * f1 + h11 * d1;
//        };
//    auto dH = [&](double s)->double {
//        double s2 = s * s;
//        double dh00 = 6 * s2 - 6 * s, dh10 = 3 * s2 - 4 * s + 1, dh01 = -6 * s2 + 6 * s, dh11 = 3 * s2 - 2 * s;
//        return dh00 * f0 + dh10 * d0 + dh01 * f1 + dh11 * d1;
//        };
//    double s = (iso - f0) / (f1 - f0); if (!(s >= 0.0 && s <= 1.0)) s = 0.5;
//    double a = 0.0, b = 1.0; double fa = H(a) - iso, fb = H(b) - iso;
//    if (fa * fb > 0.0) { const double denom = (f1 - f0); if (std::abs(denom) < 1e-12) return 0.5; double sl = (iso - f0) / denom; return std::min(1.0, std::max(0.0, sl)); }
//
//    for (int it = 0; it < maxIter; ++it) {
//        double y = H(s) - iso; if (std::abs(y) < 1e-8) break;
//        double dy = dH(s); double s_new = (std::abs(dy) > 1e-12) ? (s - y / dy) : 0.5 * (a + b);
//        if (s_new<a || s_new>b) s_new = 0.5 * (a + b);
//        double y_new = H(s_new) - iso;
//        if (fa * y_new <= 0.0) { b = s_new; fb = y_new; }
//        else { a = s_new; fa = y_new; }
//        s = s_new; if (std::abs(b - a) < 1e-8) break;
//    }
//    return std::min(1.0, std::max(0.0, 0.5 * (a + b)));
//}
//
//// ---------------------- L∞ 顶点（POCS + t 二分） ----------------------
//static bool pocs_feasible(const std::vector<Vec3>& normals, const std::vector<double>& bs, double t, Vec3& x, int maxOuter = 60) {
//    const size_t m = normals.size(); if (m == 0) return false;
//    for (int it = 0; it < maxOuter; ++it) {
//        double maxViolation = 0.0;
//        for (size_t j = 0; j < m; ++j) {
//            const Vec3& n = normals[j]; const double b = bs[j];
//            const double r = n.x * x.x + n.y * x.y + n.z * x.z - b;
//            double v = 0.0; if (r > t) v = r - t; else if (r < -t) v = r + t; // 投影到边界
//            if (v != 0.0) { x.x -= v * n.x; x.y -= v * n.y; x.z -= v * n.z; }
//            double av = std::abs(v); if (av > maxViolation) maxViolation = av;
//        }
//        if (maxViolation < 1e-7) return true;
//    }
//    return false;
//}
//
//static bool linf_dual_vertex(const std::vector<EdgeIntersect>& es, Vec3& outX, double& outT) {
//    std::vector<Vec3> normals; normals.reserve(es.size());
//    std::vector<double> bs;    bs.reserve(es.size());
//    for (const auto& e : es) {
//        Vec3 n = e.nrm; double n2 = dot(n, n); if (n2 < 1e-12) continue; n = n / std::sqrt(n2);
//        normals.push_back(n); bs.push_back(dot(n, e.pos));
//    }
//    const size_t m = normals.size(); if (m < 3) return false;
//    Vec3 x0(0, 0, 0); for (const auto& e : es) x0 += e.pos; x0 = x0 / double(es.size());
//    auto max_res = [&](const Vec3& x) { double t = 0.0; for (size_t i = 0; i < m; ++i) { double r = dot(normals[i], x) - bs[i]; t = std::max(t, std::abs(r)); } return t; };
//    double t_low = 0.0, t_high = std::max(1e-6, max_res(x0));
//    Vec3 x = x0;
//    for (int k = 0; k < 20; ++k) { Vec3 xt = x0; if (pocs_feasible(normals, bs, t_high, xt)) { x = xt; break; } t_high *= 2.0; if (t_high > 1e6) break; }
//    Vec3 bestX = x; double bestT = t_high;
//    for (int it = 0; it < 40; ++it) {
//        double t_mid = 0.5 * (t_low + t_high); Vec3 xt = x;
//        bool ok = pocs_feasible(normals, bs, t_mid, xt);
//        if (ok) { bestX = xt; bestT = t_mid; t_high = t_mid; }
//        else { t_low = t_mid; }
//        if ((t_high - t_low) < 1e-6) break;
//    }
//    outX = bestX; outT = bestT; return true;
//}
//
//// ---------------------- 保存PLY（带法向） ----------------------
//static void renormalize_normals(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) {
//    for (auto& pt : cloud->points) {
//        const double nx = pt.normal_x, ny = pt.normal_y, nz = pt.normal_z;
//        const double n2 = nx * nx + ny * ny + nz * nz;
//        if (n2 > 0) { const double inv = 1.0 / std::sqrt(n2); pt.normal_x = float(nx * inv); pt.normal_y = float(ny * inv); pt.normal_z = float(nz * inv); }
//    }
//}
//
//static bool save_clouds_ply_with_normals(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_raw,
//    const std::string& out_dir,
//    double voxelLeaf)
//{
//    ensure_dir(out_dir);
//
//    const std::string raw_path = (fs::path(out_dir) / "pointcloud_raw_CLinf-HDC_withNormals.ply").string();
//    if (pcl::io::savePLYFileBinary(raw_path, *cloud_raw) != 0) {
//        std::cout << "[错误] 保存原始点云失败: " << raw_path << "\n"; return false;
//    }
//    std::cout << "[信息] 已保存原始点云(含法向): " << raw_path << " 点数=" << cloud_raw->size() << "\n";
//
//    // 下采样（法向也参与平均），再单位化
//    pcl::VoxelGrid<pcl::PointNormal> vox;
//    vox.setInputCloud(cloud_raw);
//    vox.setLeafSize(float(voxelLeaf), float(voxelLeaf), float(voxelLeaf));
//    vox.setDownsampleAllData(true); // 让法向/其他字段也参与平均
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointNormal>());
//    vox.filter(*cloud_ds);
//    renormalize_normals(cloud_ds);
//
//    const std::string ds_path = (fs::path(out_dir) / ("pointcloud_downsampled_voxel" + std::to_string(int(voxelLeaf)) + "_CLinf-HDC_withNormals.ply")).string();
//    if (pcl::io::savePLYFileBinary(ds_path, *cloud_ds) != 0) {
//        std::cout << "[错误] 保存下采样点云失败: " << ds_path << "\n"; return false;
//    }
//    std::cout << "[信息] 已保存下采样点云(含法向): " << ds_path << " 点数=" << cloud_ds->size() << "\n";
//    return true;
//}
//
//// ---------------------- 主流程 ----------------------
//int main() {
//    // ---------- 参数（保持与你原来一致，可按需修改） ----------
//    const int startIdx = 223;
//    const int endIdx = 1620;
//    const std::string in_pattern = "D:\\\\PointCloudExtraction\\\\SLICE_512\\\\%04d.bmp";
//    const std::string out_dir = "D:\\\\PointCloudExtraction\\\\teapot\\\\teapot_CLinf-HDC\\\\";
//
//    // 是否把“边交点”并入原始点云（通常点数会大很多；如要控点数可设为 false）
//    const bool includeEdgePointsInRawCloud = false;
//
//    // 体素下采样叶子尺寸（单位=体素坐标）；可增大到 3/4/5 进一步减少点数
//    const double voxelLeaf = 2.0;
//    // --------------------------------------------------------
//
//    std::cout << "[信息] 读取图像序列并构建体数据...\n";
//
//    // 读取首张图了解尺寸/位深
//    char buf0[512] = { 0 };
//    std::snprintf(buf0, sizeof(buf0), in_pattern.c_str(), startIdx);
//    cv::Mat first = cv::imread(buf0, cv::IMREAD_UNCHANGED);
//    if (first.empty()) { std::cout << "[错误] 无法读取首张图像: " << buf0 << "\n"; return 1; }
//    if (first.channels() > 1) cv::cvtColor(first, first, cv::COLOR_BGR2GRAY);
//
//    const int W = first.cols, H = first.rows, D = endIdx - startIdx + 1;
//    Volume vol; vol.W = W; vol.H = H; vol.D = D; vol.data.resize(size_t(W) * H * D, 0.0f);
//
//    const bool is16u = (first.depth() == CV_16U);
//    const uint32_t levels = is16u ? 65536u : 256u;
//    std::vector<uint64_t> hist(levels, 0ull);
//
//    // 读取所有切片并填充体素/直方图
//    for (int s = startIdx; s <= endIdx; ++s) {
//        char path[512] = { 0 }; std::snprintf(path, sizeof(path), in_pattern.c_str(), s);
//        cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
//        if (img.empty()) { std::cout << "[错误] 读取失败: " << path << "\n"; return 2; }
//        if (img.channels() > 1) cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
//        if (img.cols != W || img.rows != H) { std::cout << "[错误] 图像尺寸不一致: " << path << "\n"; return 3; }
//
//        const int z = s - startIdx;
//        if (img.depth() == CV_8U) {
//            for (int y = 0; y < H; ++y) {
//                const uint8_t* p = img.ptr<uint8_t>(y);
//                for (int x = 0; x < W; ++x) { uint8_t v = p[x]; hist[uint32_t(v)]++; vol.data[vol.idx(x, y, z)] = float(v); }
//            }
//        }
//        else if (img.depth() == CV_16U) {
//            for (int y = 0; y < H; ++y) {
//                const uint16_t* p = img.ptr<uint16_t>(y);
//                for (int x = 0; x < W; ++x) { uint16_t v = p[x]; hist[uint32_t(v)]++; vol.data[vol.idx(x, y, z)] = float(v); }
//            }
//        }
//        else {
//            cv::Mat tmp16; img.convertTo(tmp16, CV_16U);
//            for (int y = 0; y < H; ++y) {
//                const uint16_t* p = tmp16.ptr<uint16_t>(y);
//                for (int x = 0; x < W; ++x) { uint16_t v = p[x]; hist[uint32_t(v)]++; vol.data[vol.idx(x, y, z)] = float(v); }
//            }
//        }
//        if ((s - startIdx) % 20 == 0) std::cout << "  [进度] 已读取切片 z=" << z << "/" << (D - 1) << "\n";
//    }
//
//    const double iso = otsu_threshold_from_hist(hist, levels);
//    std::cout << "[信息] Otsu 等值 = " << iso << "\n";
//
//    // 计算每个节点的梯度（法向来源）
//    std::cout << "[信息] 计算三维梯度场...\n";
//    std::vector<Vec3> grad; grad.resize(vol.data.size());
//    for (int z = 0; z < D; ++z) {
//        for (int y = 0; y < H; ++y) {
//            for (int x = 0; x < W; ++x) {
//                grad[vol.idx(x, y, z)] = gradient_at(vol, x, y, z);
//            }
//        }
//        if (z % 20 == 0) std::cout << "  [进度] 梯度 z=" << z << "/" << (D - 1) << "\n";
//    }
//
//    // 输出点云：PointNormal
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointNormal>());
//    cloud_raw->reserve(size_t(W) * H * D / 4);
//
//    // 遍历所有立方体单元
//    std::cout << "[信息] 执行 CLinf-HDC 点云提取（含法向）...\n";
//    int processedCells = 0;
//
//    for (int k = 0; k < D - 1; ++k) {
//        for (int j = 0; j < H - 1; ++j) {
//            for (int i = 0; i < W - 1; ++i) {
//                // 8角点
//                float f[8];
//                for (int c = 0; c < 8; ++c) {
//                    int cx = i + cornerOffset[c][0], cy = j + cornerOffset[c][1], cz = k + cornerOffset[c][2];
//                    f[c] = vol.val(cx, cy, cz);
//                }
//                float fmin = f[0], fmax = f[0];
//                for (int c = 1; c < 8; ++c) { fmin = std::min(fmin, f[c]); fmax = std::max(fmax, f[c]); }
//                if (!(iso >= fmin && iso <= fmax)) continue; // 非活跃单元跳过
//
//                // 收集穿越边
//                std::vector<EdgeIntersect> E; E.reserve(6);
//                for (int e = 0; e < 12; ++e) {
//                    const int c0 = edgeCorners[e][0], c1 = edgeCorners[e][1];
//                    const float v0 = f[c0], v1 = f[c1];
//                    const bool cross = ((v0 - iso) * (v1 - iso) <= 0.0f) && (v0 != v1);
//                    if (!cross) continue;
//
//                    const int x0 = i + cornerOffset[c0][0], y0 = j + cornerOffset[c0][1], z0 = k + cornerOffset[c0][2];
//                    const int x1 = i + cornerOffset[c1][0], y1 = j + cornerOffset[c1][1], z1 = k + cornerOffset[c1][2];
//
//                    // 端点梯度与沿边方向导数
//                    const Vec3 g0 = grad[vol.idx(x0, y0, z0)];
//                    const Vec3 g1 = grad[vol.idx(x1, y1, z1)];
//                    Vec3 dir = normalize(Vec3(double(x1 - x0), double(y1 - y0), double(z1 - z0)));
//                    const double d0 = dot(g0, dir), d1 = dot(g1, dir);
//
//                    // Hermite 根
//                    const double s = hermite_root_find(double(v0), double(v1), d0, d1, iso);
//
//                    // 交点坐标
//                    const double fx = double(x0) + (double(x1) - double(x0)) * s;
//                    const double fy = double(y0) + (double(y1) - double(y0)) * s;
//                    const double fz = double(z0) + (double(z1) - double(z0)) * s;
//
//                    // 交点法向：用梯度场三线性插值
//                    Vec3 n = normal_at_point(vol, grad, fx, fy, fz);
//
//                    EdgeIntersect EI; EI.pos = Vec3(fx, fy, fz); EI.nrm = n; EI.b = dot(n, EI.pos);
//                    E.push_back(EI);
//
//                    // （可选）把边交点加入原始点云
//                    if (includeEdgePointsInRawCloud) {
//                        pcl::PointNormal pn;
//                        pn.x = float(fx); pn.y = float(fy); pn.z = float(fz);
//                        pn.normal_x = float(n.x); pn.normal_y = float(n.y); pn.normal_z = float(n.z);
//                        pn.curvature = 0.0f;
//                        cloud_raw->push_back(pn);
//                    }
//                }
//
//                if (E.size() < 3) continue;
//
//                // L∞ 顶点
//                Vec3 x_star; double t_star = 0.0;
//                if (!linf_dual_vertex(E, x_star, t_star)) continue;
//
//                // 顶点法向：三线性插值；若退化则用 E 法向的加权平均回退
//                Vec3 n_v = normal_at_point(vol, grad, x_star.x, x_star.y, x_star.z);
//                if (norm(n_v) <= 0) {
//                    Vec3 acc(0, 0, 0);
//                    for (const auto& ei : E) acc += ei.nrm;
//                    n_v = normalize(acc);
//                }
//
//                pcl::PointNormal pv;
//                pv.x = float(x_star.x); pv.y = float(x_star.y); pv.z = float(x_star.z);
//                pv.normal_x = float(n_v.x); pv.normal_y = float(n_v.y); pv.normal_z = float(n_v.z);
//                pv.curvature = 0.0f;
//                cloud_raw->push_back(pv);
//
//                ++processedCells;
//            }
//        }
//        if (k % 5 == 0) {
//            std::cout << "[进度] 单元层 z=" << k << "/" << (D - 2)
//                << "，累计处理单元数=" << processedCells
//                << "，当前点数=" << cloud_raw->size() << "\n";
//        }
//    }
//
//    // 写出（含法向）
//    std::cout << "[信息] 提取完成，写出点云(含法向)...\n";
//    const bool ok = save_clouds_ply_with_normals(cloud_raw, out_dir, voxelLeaf);
//    if (!ok) { std::cout << "[错误] 写出PLY失败。\n"; return 4; }
//
//    std::cout << "[完成] 所有流程结束。\n";
//    return 0;
//}
//
//





