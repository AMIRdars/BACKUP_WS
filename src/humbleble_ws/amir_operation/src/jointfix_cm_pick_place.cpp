/*
 * jointfix_cm_pick_place.cpp
 *
 * jointfix_pick_place.cpp をベースに pick_place_humble_10.cpp の
 * Collision Margin (CM) / Danger Field (DF) 評価・CSV 記録・RViz 可視化を統合。
 *
 * 主な特徴:
 *  - 複数物体の連続 Pick & Place（jointfix の動作ロジック）
 *  - GripperCommand アクションクライアント＋段階的把持・接触検知
 *  - Joint_4/5 拘束付き位置指定移動
 *  - 各移動ステップで K=3 経路候補を計画し CM 最大のものを選択・実行
 *  - CM / DF / 楕円体データを /tmp/ 以下の CSV に保存
 *  - RViz に操作性楕円体・危険方向矢印を可視化
 */

// ─── 基本 ───────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>
#include <string>
#include <fstream>
#include <limits>
#include <iostream>
#include <algorithm>
#include <atomic>
#include <ctime>
#include <filesystem>

// ─── MoveIt ─────────────────────────────────────────────────────────────────
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// ─── ROS メッセージ ──────────────────────────────────────────────────────────
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ─── 数値計算 ────────────────────────────────────────────────────────────────
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using MoveGroupInterface     = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
using GripperCommand         = control_msgs::action::GripperCommand;
using GripperClient          = rclcpp_action::Client<GripperCommand>;

// ═══════════════════════════════════════════════════════════════════════════
// 設定パラメータ（jointfix_pick_place.cpp から）
// ═══════════════════════════════════════════════════════════════════════════

const double FIXED_JOINT_5_VALUE = 0.0;
const double APPROACH_HEIGHT     = 0.15;  // [m]

struct Pose3D { double x, y, z; };

const std::vector<Pose3D> OBJ_POSITIONS = {
    {0.5, -0.20, 0.18},
    {0.5, -0.05, 0.18},
    {0.5,  0.10, 0.18},
    // {0.5,  0.25, 0.08},
};
const std::vector<Pose3D> PLACE_POSITIONS = {
    {-0.20, 0.40, 0.23},
    {-0.05, 0.40, 0.23},
    { -0.05, 0.50, 0.23},
    // { 0.25, 0.40, 0.15},
};

// 物体モデル（CM 解析・衝突オブジェクトで使用）
const double OBJ_RADIUS = 0.03;  // [m]
const double OBJ_HEIGHT = 0.10;   // [m]

// ─── place_divider 設定 ──────────────────────────────────────────────────────
// spawn_cm_scene.launch.py の同名定数と値を揃えること
const double DIV_THICK  = 0.0;   // x方向の厚み [m]
const double DIV_HEIGHT = 0.14;   // z方向の高さ [m]
const double DIV_DEPTH  = 0.25;   // y方向の奥行き [m]

// 仕切りの中心座標リスト（base_footprint 基準）
// {cx, cy, cz} を直接指定。要素を増減するだけで枚数を変えられる。
struct DividerPos { double cx, cy, cz; };
const std::vector<DividerPos> DIVIDER_POSITIONS = {
    // {-0.125, 0.40, 0.17},  // 仕切り0
};

// グリッパー
const double GRIPPER_OPEN             = -1.0;
const double GRIPPER_CLOSE            =  0.1;
const int    GRIPPER_CLOSE_STEPS      = 25;
const int    GRIPPER_CLOSE_STEP_MS    = 100;
const double GRIPPER_STALL_THRESHOLD  = 0.01;
const int    GRIPPER_DETECT_START_STEP = 5;
const double GRIPPER_PRELOAD          = 0.10;
const double GRIPPER_MAX_POS          = 0.261799;

// ── アタッチ時に接触を許可するリンク（把持中の自己衝突誤検知を防ぐ）──────────
const std::vector<std::string> TOUCH_LINKS = {
    "gripper_base_1",
    "finger_left_1",
    "finger_right_1",
    "inner_link_left_1",
    "inner_link_right_1",
    "outer_link_left_1",
    "outer_link_right_1",
    "tcp_link",
    "d435_link",
    "d435_bottom_screw_frame",
};

// ═══════════════════════════════════════════════════════════════════════════
// CM / DF 解析パラメータ
// ═══════════════════════════════════════════════════════════════════════════

const int    PLAN_CANDIDATES  = 3;       // 計画候補数
const std::string JAC_LINK    = "tcp_link";
const std::string LOG_ROOT    = "/home/das-note-5080/pick_place_logs/";

static std::string g_condition_id = "jointfix_cm";
static std::string g_run_dir;          // 実行ごとに作成されるタイムスタンプフォルダ

struct DFParams {
    double R      = OBJ_RADIUS;
    double eps    = 0.001;
    double k1     = 1.0;
    double k2     = 1.0;
    double k3     = 1.0;
    double gamma  = 1.0;
    double sigma0 = 0.05;
    double Delta  = 50.0;
    double v_eps  = 1e-6;
};
static DFParams g_prm;

struct RepPoint {
    std::string       link;
    Eigen::Vector3d   offset;
    std::string       name;
};

static const std::vector<RepPoint> REP_POINTS = {
    {"outer_link_left_1",  Eigen::Vector3d::Zero(), "outer_link_left_1"},
    {"outer_link_right_1", Eigen::Vector3d::Zero(), "outer_link_right_1"},
    {"finger_right_1",     Eigen::Vector3d::Zero(), "finger_right_1"},
    {"finger_left_1",      Eigen::Vector3d::Zero(), "finger_left_1"},
    {"tcp_link", Eigen::Vector3d(0, 0, +OBJ_HEIGHT / 2.0), "grasp_obj_top"},
    {"tcp_link", Eigen::Vector3d(0, 0, -OBJ_HEIGHT / 2.0), "grasp_obj_bottom"},
};

struct Obstacle { Eigen::Vector3d center; };
static std::vector<Obstacle> g_obstacles;

struct CMLogRow {
    int    plan_success = 0;
    int    exec_success = -1;
    double plan_time_s  = 0.0;
    double M     = 0.0;
    double m_min = 1.0;
    double d_min = std::numeric_limits<double>::infinity();
    double T     = 0.0;
    int    n_points = 0;
};

struct DFLogRow {
    int    plan_success = 0;
    int    exec_success = -1;
    std::string fail_mode = "NA";
    double dmin  = std::numeric_limits<double>::infinity();
    double amax  = 0.0;
    double sigmin = std::numeric_limits<double>::infinity();
    double DFmax = 0.0;
    double Lrisk = 0.0;
    double Rrisk = 0.0;
};

// ═══════════════════════════════════════════════════════════════════════════
// ユーティリティ
// ═══════════════════════════════════════════════════════════════════════════

static std::string csv_escape(const std::string &s) {
    if (s.find(',') == std::string::npos && s.find('"') == std::string::npos)
        return s;
    std::string o = "\"";
    for (char c : s) o += (c == '"' ? "\"\"" : std::string(1, c));
    return o + "\"";
}

static Eigen::Vector3d getPointW(const moveit::core::RobotState &st, const RepPoint &rp) {
    const Eigen::Isometry3d &T = st.getGlobalLinkTransform(rp.link);
    return T.translation() + T.rotation() * rp.offset;
}

static void nearestObstacle(const Eigen::Vector3d &p,
                             const std::vector<Obstacle> &obs,
                             double R,
                             int &j_star, double &d_clear, Eigen::Vector3d &o_star)
{
    j_star  = -1;
    d_clear = std::numeric_limits<double>::infinity();
    o_star  = Eigen::Vector3d::Zero();
    for (int j = 0; j < (int)obs.size(); ++j) {
        double d = (p - obs[j].center).norm() - R;
        if (d < d_clear) { d_clear = d; j_star = j; o_star = obs[j].center; }
    }
}

static double minSingularValueJacobian(const moveit::core::RobotState &st,
                                        const moveit::core::JointModelGroup *jmg,
                                        const std::string &ee_link)
{
    Eigen::MatrixXd J;
    st.getJacobian(jmg, st.getLinkModel(ee_link), Eigen::Vector3d::Zero(), J);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.singularValues().size() > 0 ? svd.singularValues().minCoeff() : 0.0;
}


// ═══════════════════════════════════════════════════════════════════════════
// CSV 追記
// ═══════════════════════════════════════════════════════════════════════════

static void appendCmTrajCsv(const std::string &path,
                             const std::string &cond,
                             int trial,
                             const std::string &stage,
                             const CMLogRow &r)
{
    bool new_file = !std::ifstream(path).good();
    std::ofstream ofs(path, std::ios::app);
    if (new_file)
        ofs << "condition_id,trial,stage,plan_success,plan_time_s,exec_success,"
               "M,m_min,d_min,T,n_points\n";
    ofs << csv_escape(cond) << "," << trial << "," << csv_escape(stage) << ","
        << r.plan_success << "," << r.plan_time_s << "," << r.exec_success << ","
        << r.M << "," << r.m_min << "," << r.d_min << "," << r.T << "," << r.n_points << "\n";
}

static void appendCmTimeSeriesCsv(const std::string &path,
                                   const std::string &cond,
                                   int trial,
                                   const std::string &stage,
                                   int idx, double t,
                                   double m_k, double d_k, double r_k,
                                   double rho, double alpha,
                                   const Eigen::Vector3d &ghat,
                                   const Eigen::Vector3d &ulong,
                                   const std::string &link_star)
{
    bool new_file = !std::ifstream(path).good();
    std::ofstream ofs(path, std::ios::app);
    if (new_file)
        ofs << "condition_id,trial,stage,idx,t,m_k,d_k,r_k,rho,alpha,"
               "ghx,ghy,ghz,ulx,uly,ulz,link_star\n";
    ofs << csv_escape(cond) << "," << trial << "," << csv_escape(stage) << ","
        << idx << "," << t << "," << m_k << "," << d_k << "," << r_k << ","
        << rho << "," << alpha << ","
        << ghat.x() << "," << ghat.y() << "," << ghat.z() << ","
        << ulong.x() << "," << ulong.y() << "," << ulong.z() << ","
        << csv_escape(link_star) << "\n";
}

static void appendDfSummaryCsv(const std::string &path,
                                const std::string &cond,
                                int trial,
                                const std::string &stage,
                                const DFLogRow &r)
{
    bool new_file = !std::ifstream(path).good();
    std::ofstream ofs(path, std::ios::app);
    if (new_file)
        ofs << "condition_id,trial,stage,plan_success,exec_success,fail_mode,"
               "dmin,amax,sigmin,DFmax,Lrisk,Rrisk\n";
    ofs << csv_escape(cond) << "," << trial << "," << csv_escape(stage) << ","
        << r.plan_success << "," << r.exec_success << "," << csv_escape(r.fail_mode) << ","
        << r.dmin << "," << r.amax << "," << r.sigmin << ","
        << r.DFmax << "," << r.Lrisk << "," << r.Rrisk << "\n";
}

// ═══════════════════════════════════════════════════════════════════════════
// CM 解析（軌道全体）
// ═══════════════════════════════════════════════════════════════════════════

static bool calcCmAtState(const moveit::core::RobotState &st,
                           const moveit::core::JointModelGroup *jmg,
                           const Eigen::Vector3d &p,
                           const Eigen::Vector3d &obs_center,
                           double R, double lambda,
                           double &out_m, double &out_d, double &out_r,
                           Eigen::Vector3d &out_ghat, Eigen::Vector3d &out_ostar)
{
    Eigen::Vector3d pc = obs_center - p;
    double pc_norm = pc.norm();
    if (pc_norm < 1e-9) return false;

    Eigen::Vector3d o = obs_center - R * (pc / pc_norm);
    out_ostar = o;
    Eigen::Vector3d g = o - p;
    double d = g.norm();

    if (d <= 1e-6) { out_m = 0.0; out_d = 0.0; out_r = 0.0; out_ghat = {}; return true; }

    out_ghat = g / d;
    Eigen::MatrixXd J;
    st.getJacobian(jmg, st.getLinkModel(JAC_LINK), Eigen::Vector3d::Zero(), J);
    Eigen::MatrixXd Jpos = J.topRows(3);
    Eigen::Matrix3d A    = Jpos * Jpos.transpose();
    Eigen::Matrix3d Ainv = (A + lambda * Eigen::Matrix3d::Identity()).inverse();
    double denom = out_ghat.transpose() * Ainv * out_ghat;
    denom = std::max(1e-12, denom);
    double r = (1.0 / std::sqrt(denom)) * 0.115;  // スケール係数

    out_m = std::clamp(1.0 - (r / d), 0.0, 1.0);
    out_d = d;
    out_r = r;
    return true;
}

static CMLogRow analyzeCollisionMargin(
    const MoveGroupInterface &arm,
    const trajectory_msgs::msg::JointTrajectory &jt,
    const std::string &ts_csv,
    const std::string &cond, int trial, const std::string &stage,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub)
{
    CMLogRow out;
    if (jt.points.size() < 2) return out;
    if (g_obstacles.empty()) { out.m_min = 1.0; out.M = 1.0; return out; }

    auto robot_model = arm.getRobotModel();
    const auto *jmg  = robot_model->getJointModelGroup(arm.getName());
    if (!jmg) return out;

    moveit::core::RobotState st(robot_model);
    st.setToDefaultValues();
    const double lambda = 1e-4;
    double num = 0.0, den = 0.0;
    out.m_min = 1.0;
    out.d_min = std::numeric_limits<double>::infinity();

    for (size_t k = 0; k + 1 < jt.points.size(); ++k) {
        st.setVariablePositions(jt.joint_names, jt.points[k].positions);
        st.update();
        double t  = rclcpp::Duration(jt.points[k].time_from_start).seconds();
        double t2 = rclcpp::Duration(jt.points[k + 1].time_from_start).seconds();
        double dt = std::max(0.0, t2 - t);

        double mk = 1.0, dk = std::numeric_limits<double>::infinity(), rk = 0.0;
        std::string link_star = "NA";
        Eigen::Vector3d ghat_star = Eigen::Vector3d::Zero();
        Eigen::Vector3d p_star    = Eigen::Vector3d::Zero();

        for (const auto &rp : REP_POINTS) {
            Eigen::Vector3d p = getPointW(st, rp);
            int j_star; double d_clear; Eigen::Vector3d c_star;
            nearestObstacle(p, g_obstacles, OBJ_RADIUS, j_star, d_clear, c_star);
            if (j_star < 0) continue;

            double m_i, d_i, r_i;
            Eigen::Vector3d ghat_i, ostar_i;
            if (!calcCmAtState(st, jmg, p, c_star, OBJ_RADIUS, lambda,
                               m_i, d_i, r_i, ghat_i, ostar_i)) continue;
            if (m_i < mk) {
                mk = m_i; dk = d_i; rk = r_i;
                link_star = rp.name; ghat_star = ghat_i; p_star = p;
            }
        }

        // 楕円体（位置ヤコビアン SVD）
        Eigen::MatrixXd J;
        st.getJacobian(jmg, st.getLinkModel(JAC_LINK), Eigen::Vector3d::Zero(), J);
        Eigen::MatrixXd Jpos = J.topRows(3);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jpos, Eigen::ComputeFullU | Eigen::ComputeThinV);
        auto s = svd.singularValues();
        Eigen::Matrix3d U  = svd.matrixU();
        Eigen::Vector3d ulong = U.col(0);

        double alpha = std::abs(ulong.dot(ghat_star));
        double rho   = (dk > 1e-9) ? (rk / dk) : 1e9;

        out.m_min = std::min(out.m_min, mk);
        out.d_min = std::min(out.d_min, dk);
        num += mk * dt;
        den += dt;

        appendCmTimeSeriesCsv(ts_csv, cond, trial, stage, (int)k, t,
                               mk, dk, rk, rho, alpha,
                               ghat_star, ulong, link_star);

        // RViz マーカー（best のみ）
        if (marker_pub && link_star != "NA") {
            Eigen::Vector3d tcp = st.getGlobalLinkTransform(JAC_LINK).translation();
            visualization_msgs::msg::MarkerArray arr;
            auto makeArrow = [&](int id, const Eigen::Vector3d &dir, double len,
                                  float r, float g, float b) {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "world";
                m.header.stamp    = rclcpp::Time(0);
                m.ns = "cm_ellipsoid"; m.id = id;
                m.type   = visualization_msgs::msg::Marker::ARROW;
                m.action = visualization_msgs::msg::Marker::ADD;
                geometry_msgs::msg::Point ps, pe;
                ps.x = tcp.x(); ps.y = tcp.y(); ps.z = tcp.z();
                Eigen::Vector3d e = tcp + len * dir.normalized();
                pe.x = e.x(); pe.y = e.y(); pe.z = e.z();
                m.points = {ps, pe};
                m.scale.x = 0.01; m.scale.y = 0.02; m.scale.z = 0.02;
                m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.9f;
                m.lifetime = rclcpp::Duration(0, 0);
                return m;
            };
            const double kv = 0.20;
            arr.markers.push_back(makeArrow(1, U.col(0), kv * s(0), 1, 0, 0));
            arr.markers.push_back(makeArrow(2, U.col(1), kv * (s.size()>1?s(1):0), 0, 1, 0));
            arr.markers.push_back(makeArrow(3, U.col(2), kv * (s.size()>2?s(2):0), 0, 0, 1));
            if (ghat_star.norm() > 1e-9)
                arr.markers.push_back(makeArrow(10, ghat_star, 0.20, 1, 1, 0));
            marker_pub->publish(arr);
        }
    }

    out.T       = rclcpp::Duration(jt.points.back().time_from_start).seconds();
    out.n_points = (int)jt.points.size();
    out.M       = (den > 0.0) ? (num / den) : 0.0;
    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// DF 解析（軌道全体サマリ）
// ═══════════════════════════════════════════════════════════════════════════

static DFLogRow analyzeDangerField(
    const MoveGroupInterface &arm,
    const trajectory_msgs::msg::JointTrajectory &jt)
{
    DFLogRow out;
    if (jt.points.size() < 2 || g_obstacles.empty()) return out;
    auto robot_model = arm.getRobotModel();
    const auto *jmg  = robot_model->getJointModelGroup(arm.getName());
    if (!jmg) return out;

    for (size_t k = 0; k + 1 < jt.points.size(); ++k) {
        moveit::core::RobotState st(robot_model), st2(robot_model);
        st.setToDefaultValues();  st2.setToDefaultValues();
        st.setVariablePositions(jt.joint_names, jt.points[k].positions);     st.update();
        st2.setVariablePositions(jt.joint_names, jt.points[k+1].positions);  st2.update();

        double t  = rclcpp::Duration(jt.points[k].time_from_start).seconds();
        double t2 = rclcpp::Duration(jt.points[k+1].time_from_start).seconds();
        double dt = std::max(1e-6, t2 - t);

        double sigma = minSingularValueJacobian(st, jmg, JAC_LINK);
        double DFsig = g_prm.k3 * std::max(0.0, (g_prm.sigma0 - sigma) / g_prm.sigma0);

        double DF_max = 0.0, d_min_all = std::numeric_limits<double>::infinity();
        double a_max = 0.0, ds_max = 0.0;

        for (const auto &rp : REP_POINTS) {
            Eigen::Vector3d p  = getPointW(st,  rp);
            Eigen::Vector3d p2 = getPointW(st2, rp);
            double ds = (p2 - p).norm();
            ds_max = std::max(ds_max, ds);

            int j_star; double d_clear; Eigen::Vector3d o_star;
            nearestObstacle(p, g_obstacles, g_prm.R, j_star, d_clear, o_star);
            d_min_all = std::min(d_min_all, d_clear);

            double dprime = d_clear + g_prm.eps;
            double DFd    = g_prm.k1 / dprime;
            Eigen::Vector3d v = (p2 - p) / dt;
            double vnorm = v.norm();
            double cosphi = 0.0;
            if (vnorm > g_prm.v_eps) {
                Eigen::Vector3d g_vec = o_star - p;
                double gn = g_vec.norm();
                if (gn > 1e-9) cosphi = (v / vnorm).dot(g_vec / gn);
            }
            double a_k   = std::max(0.0, cosphi);
            double DFa   = g_prm.k2 * vnorm * std::max(0.0, g_prm.gamma + cosphi) / (dprime * dprime);
            a_max  = std::max(a_max,  a_k);
            DF_max = std::max(DF_max, DFd + DFa);
        }

        double DF = DF_max + DFsig;
        if (d_min_all < out.dmin)   out.dmin   = d_min_all;
        if (a_max      > out.amax)   out.amax   = a_max;
        if (sigma      < out.sigmin) out.sigmin = sigma;
        if (DF         > out.DFmax)  out.DFmax  = DF;
        if (DF > g_prm.Delta) out.Lrisk += ds_max;
        out.Rrisk += DF * ds_max;
    }
    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// CM スコアリング
// ═══════════════════════════════════════════════════════════════════════════

static bool betterCM(const CMLogRow &a, const CMLogRow &b) {
    if (a.M     != b.M)     return a.M     > b.M;
    if (a.m_min != b.m_min) return a.m_min > b.m_min;
    if (a.d_min != b.d_min) return a.d_min > b.d_min;
    return a.n_points < b.n_points;
}

// ═══════════════════════════════════════════════════════════════════════════
// グリッパー（jointfix_pick_place.cpp から）
// ═══════════════════════════════════════════════════════════════════════════

static void controlGripper(rclcpp::Node::SharedPtr node,
                            GripperClient::SharedPtr client,
                            double position)
{
    (void)node;
    if (!client->wait_for_action_server(5s)) return;
    auto goal = GripperCommand::Goal();
    goal.command.position   = position;
    goal.command.max_effort = 50.0;
    auto gh = client->async_send_goal(goal).get();
    if (gh) client->async_get_result(gh).get();
}

static void closeGripperGradually(rclcpp::Node::SharedPtr node,
                                   GripperClient::SharedPtr client,
                                   double target_pos = GRIPPER_CLOSE)
{
    using JointState = sensor_msgs::msg::JointState;
    if (!client->wait_for_action_server(5s)) return;

    std::atomic<double> g_pos{GRIPPER_OPEN};
    auto js_sub = node->create_subscription<JointState>(
        "/joint_states", rclcpp::QoS(10),
        [&g_pos](JointState::ConstSharedPtr msg) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == "Gripper" && i < msg->position.size()) {
                    g_pos.store(msg->position[i]); break;
                }
            }
        });

    rclcpp::sleep_for(200ms);
    double start_pos = g_pos.load();
    double range     = target_pos - start_pos;
    bool   contacted = false;
    double contact_pos = target_pos;
    double prev_actual = start_pos;

    RCLCPP_INFO(node->get_logger(),
        "Gripper 段階的閉じ (%.3f → %.3f)", start_pos, target_pos);

    for (int step = 1; step <= GRIPPER_CLOSE_STEPS && rclcpp::ok(); ++step) {
        double cmd_pos = start_pos + range * (double)step / GRIPPER_CLOSE_STEPS;
        auto goal = GripperCommand::Goal();
        goal.command.position   = cmd_pos;
        goal.command.max_effort = 50.0;
        client->async_send_goal(goal);
        rclcpp::sleep_for(std::chrono::milliseconds(GRIPPER_CLOSE_STEP_MS));

        double actual   = g_pos.load();
        double movement = std::abs(actual - prev_actual);
        if (step >= GRIPPER_DETECT_START_STEP && movement < GRIPPER_STALL_THRESHOLD) {
            contact_pos = actual; contacted = true;
            RCLCPP_INFO(node->get_logger(),
                "Gripper 接触検知 step=%d actual=%.3f", step, actual);
            break;
        }
        prev_actual = actual;
    }

    double hold_target = contacted
        ? std::min(contact_pos + GRIPPER_PRELOAD, GRIPPER_MAX_POS)
        : target_pos;
    auto hold = GripperCommand::Goal();
    hold.command.position   = hold_target;
    hold.command.max_effort = 100.0;
    client->async_send_goal(hold).get();
    rclcpp::sleep_for(500ms);
}

// ═══════════════════════════════════════════════════════════════════════════
// 移動関数（K 候補 + CM 選択付き）
// ═══════════════════════════════════════════════════════════════════════════

// use_constraints=true  → Joint_4/5 拘束付き（アプローチ・降下・退避）
// use_constraints=false → 拘束なし（把持後の移動）
static bool moveLogged(
    rclcpp::Node::SharedPtr node,
    MoveGroupInterface &arm,
    int trial_id,
    const std::string &stage,
    double x, double y, double z,
    double speed_scaling,
    bool use_constraints,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub)
{
    arm.setStartStateToCurrentState();

    if (use_constraints) {
        moveit_msgs::msg::Constraints c;
        moveit_msgs::msg::JointConstraint jc4;
        jc4.joint_name = "Joint_4"; jc4.position = 0.0;
        jc4.tolerance_above = 0.5; jc4.tolerance_below = 0.5; jc4.weight = 1.0;
        c.joint_constraints.push_back(jc4);
        moveit_msgs::msg::JointConstraint jc5;
        jc5.joint_name = "Joint_5"; jc5.position = FIXED_JOINT_5_VALUE;
        jc5.tolerance_above = 0.05; jc5.tolerance_below = 0.05; jc5.weight = 1.0;
        c.joint_constraints.push_back(jc5);
        arm.setPathConstraints(c);
    }

    arm.setPositionTarget(x, y, z);
    arm.setGoalPositionTolerance(0.01);
    arm.setMaxVelocityScalingFactor(speed_scaling);
    arm.setMaxAccelerationScalingFactor(speed_scaling * 0.5);

    // ---- K 候補を計画して CM でベストを選ぶ --------------------------------
    struct Cand {
        MoveGroupInterface::Plan plan;
        CMLogRow cm;
        double plan_time_s = 0.0;
        bool ok = false;
    };
    std::vector<Cand> cands(PLAN_CANDIDATES);

    for (int i = 0; i < PLAN_CANDIDATES; ++i) {
        arm.setStartStateToCurrentState();
        auto t0 = std::chrono::steady_clock::now();
        auto code = arm.plan(cands[i].plan);
        cands[i].plan_time_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        cands[i].ok = (code == moveit::core::MoveItErrorCode::SUCCESS);
        if (!cands[i].ok) {
            RCLCPP_WARN(node->get_logger(),
                "[%s] cand%d FAIL code=%d", stage.c_str(), i, code.val);
            continue;
        }

        // CM 解析
        std::string ts_path = g_run_dir +"cm_ts_" + stage + "_cand" +
                              std::to_string(i) + ".csv";
        cands[i].cm = analyzeCollisionMargin(arm,
            cands[i].plan.trajectory_.joint_trajectory,
            ts_path, g_condition_id, trial_id, stage,
            nullptr);  // 候補選定中は RViz 出さない
        cands[i].cm.plan_success = 1;
        cands[i].cm.plan_time_s  = cands[i].plan_time_s;

        RCLCPP_INFO(node->get_logger(),
            "[%s] cand%d M=%.4f m_min=%.4f d_min=%.4f",
            stage.c_str(), i, cands[i].cm.M, cands[i].cm.m_min, cands[i].cm.d_min);
    }

    // ---- ベスト選択 ---------------------------------------------------------
    int best = -1;
    for (int i = 0; i < PLAN_CANDIDATES; ++i) {
        if (!cands[i].ok) continue;
        if (best < 0 || betterCM(cands[i].cm, cands[best].cm)) best = i;
    }

    arm.clearPathConstraints();

    DFLogRow df_row;
    df_row.plan_success = (best >= 0) ? 1 : 0;

    if (best < 0) {
        RCLCPP_ERROR(node->get_logger(),
            "[%s] 全候補の計画に失敗。", stage.c_str());
        df_row.exec_success = -1; df_row.fail_mode = "plan_fail_all";
        appendDfSummaryCsv(g_run_dir +"df_summary.csv",
                           g_condition_id, trial_id, stage, df_row);
        CMLogRow cm_fail; cm_fail.plan_success = 0;
        appendCmTrajCsv(g_run_dir +"cm_traj.csv",
                        g_condition_id, trial_id, stage, cm_fail);
        return false;
    }

    RCLCPP_INFO(node->get_logger(),
        "[%s] best=cand%d  M=%.4f m_min=%.4f d_min=%.4f",
        stage.c_str(), best, cands[best].cm.M, cands[best].cm.m_min, cands[best].cm.d_min);

    // ベストのみ RViz 可視化（再解析）
    std::string best_ts = g_run_dir +"cm_ts_" + stage + "_best.csv";
    analyzeCollisionMargin(arm,
        cands[best].plan.trajectory_.joint_trajectory,
        best_ts, g_condition_id, trial_id, stage, marker_pub);

    // DF 解析
    df_row = analyzeDangerField(arm, cands[best].plan.trajectory_.joint_trajectory);
    df_row.plan_success = 1;

    // 実行
    auto ex = arm.execute(cands[best].plan);
    bool success = (ex == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(),
        "[%s] execute code=%d", stage.c_str(), ex.val);

    // ログ記録
    df_row.exec_success = success ? 1 : 0;
    df_row.fail_mode    = success ? "ok" : "exec_fail";
    appendDfSummaryCsv(g_run_dir +"df_summary.csv",
                       g_condition_id, trial_id, stage, df_row);

    CMLogRow &cm_best = cands[best].cm;
    cm_best.exec_success = success ? 1 : 0;
    appendCmTrajCsv(g_run_dir +"cm_traj.csv",
                    g_condition_id, trial_id, stage, cm_best);

    return success;
}

// ═══════════════════════════════════════════════════════════════════════════
// 衝突オブジェクト管理
// ═══════════════════════════════════════════════════════════════════════════

static PlanningSceneInterface *g_psi = nullptr;

// テーブル表面の CM 解析点（起動時1回構築、サイクル間で保持）
static std::vector<Obstacle> g_table_obstacles;

// 配置済み物体の CM 解析点（配置成功のたびに追加、サイクル間で保持）
static std::vector<Obstacle> g_placed_obstacles;

// ピックテーブル・プレーステーブルを Planning Scene に登録し、
// g_table_obstacles を構築する（main() で addObjectsToScene() より前に呼ぶ）
//
// 座標は base_footprint 基準。
// ピック台: OBJ_POSITIONS.z=0.08, OBJ_HEIGHT=0.10 → 台上面 z=0.03m
// プレース台: PLACE_POSITIONS.z=0.15 → 台上面 z=0.10m
static void addTableObstacles() {
    if (!g_psi) return;
    g_table_obstacles.clear();
    std::vector<moveit_msgs::msg::CollisionObject> objs;

    const double table_thick = 0.10;  // 台の厚み [m]

    // ── ピック台（table_pick）: pick物体群の下、x=0.5付近 ────────────────
    // const double pick_top_z  = OBJ_POSITIONS[0].z - OBJ_HEIGHT / 2.0;  // ≒ 0.03m
    const double pick_top_z  = 0.1;  // ≒ 0.03m
    const double pick_cx     = 0.50;
    const double pick_cy     = 0.025;   // OBJ y範囲 -0.20〜0.25 の中心
    const double pick_size_x = 0.25;
    const double pick_size_y = 0.65;

    {
        moveit_msgs::msg::CollisionObject obj;
        obj.id              = "table_pick";
        obj.header.frame_id = "base_footprint";
        obj.operation       = obj.ADD;

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions.resize(3);
        prim.dimensions[0] = pick_size_x;
        prim.dimensions[1] = pick_size_y;
        prim.dimensions[2] = table_thick;

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x    = pick_cx;
        pose.position.y    = pick_cy;
        pose.position.z    = pick_top_z - table_thick / 2.0;

        obj.primitives.push_back(prim);
        obj.primitive_poses.push_back(pose);
        objs.push_back(obj);
    }

    // CM解析用: ピック台上面グリッド（3×5点）
    for (int ix = 0; ix < 3; ++ix) {
        for (int iy = 0; iy < 5; ++iy) {
            double px = (pick_cx - pick_size_x / 2.0) + ix * (pick_size_x / 2.0);
            double py = (pick_cy - pick_size_y / 2.0) + iy * (pick_size_y / 4.0);
            g_table_obstacles.push_back({Eigen::Vector3d(px, py, pick_top_z)});
        }
    }

    // ── プレース台（table_place）: place物体群の下、y=0.40付近 ──────────
    // table_thick(0.10) だとアームリンクと干渉しやすいため、薄い天板のみにする
    // const double place_top_z        = PLACE_POSITIONS[0].z - OBJ_HEIGHT / 2.0;  // ≒ 0.10m
    const double place_top_z = 0.10;

    // {
    //     moveit_msgs::msg::CollisionObject obj;
    //     obj.id              = "table_place";
    //     obj.header.frame_id = "base_footprint";
    //     obj.operation       = obj.ADD;

    //     shape_msgs::msg::SolidPrimitive prim;
    //     prim.type = prim.BOX;
    //     prim.dimensions.resize(3);
    //     prim.dimensions[0] = place_size_x;
    //     prim.dimensions[1] = place_size_y;
    //     prim.dimensions[2] = place_slab_thick;

    //     geometry_msgs::msg::Pose pose;
    //     pose.orientation.w = 1.0;
    //     pose.position.x    = place_cx;
    //     pose.position.y    = place_cy;
    //     // 上面を place_top_z に合わせる
    //     pose.position.z    = place_top_z - place_slab_thick / 2.0;

    //     obj.primitives.push_back(prim);
    //     obj.primitive_poses.push_back(pose);
    //     objs.push_back(obj);
    // }

    // // CM解析用: プレース台上面グリッド（5×3点）
    // for (int ix = 0; ix < 5; ++ix) {
    //     for (int iy = 0; iy < 3; ++iy) {
    //         double px = (place_cx - place_size_x / 2.0) + ix * (place_size_x / 4.0);
    //         double py = (place_cy - place_size_y / 2.0) + iy * (place_size_y / 2.0);
    //         g_table_obstacles.push_back({Eigen::Vector3d(px, py, place_top_z)});
    //     }
    // }

    // ── place台上の仕切り壁 ────────────────────────────────────────────────────
    // DIVIDER_POSITIONS に中心座標を直接指定する
    for (size_t k = 0; k < DIVIDER_POSITIONS.size(); ++k) {
        const double div_cx = DIVIDER_POSITIONS[k].cx;
        const double div_cy = DIVIDER_POSITIONS[k].cy;
        const double div_cz = DIVIDER_POSITIONS[k].cz;

        moveit_msgs::msg::CollisionObject div;
        div.id              = "place_divider_" + std::to_string(k);
        div.header.frame_id = "base_footprint";
        div.operation       = div.ADD;

        shape_msgs::msg::SolidPrimitive dprim;
        dprim.type = dprim.BOX;
        dprim.dimensions.resize(3);
        dprim.dimensions[0] = DIV_THICK;
        dprim.dimensions[1] = DIV_DEPTH;
        dprim.dimensions[2] = DIV_HEIGHT;

        geometry_msgs::msg::Pose dpose;
        dpose.orientation.w = 1.0;
        dpose.position.x    = div_cx;
        dpose.position.y    = div_cy;
        dpose.position.z    = div_cz;

        div.primitives.push_back(dprim);
        div.primitive_poses.push_back(dpose);
        objs.push_back(div);

        // CM解析用: 仕切り両面（x方向±）に下端・中心・上端の3点
        for (int side : {-1, +1}) {
            double face_x = div_cx + side * DIV_THICK / 2.0;
            for (int iz = 0; iz < 3; ++iz) {
                double pz = div_cz + (iz - 1) * (DIV_HEIGHT / 2.0);
                g_table_obstacles.push_back({Eigen::Vector3d(face_x, div_cy, pz)});
            }
        }
    }

    g_psi->applyCollisionObjects(objs);
    RCLCPP_INFO(rclcpp::get_logger("scene"),
        "テーブル登録完了: table_pick (上面z=%.3f) + table_place (上面z=%.3f)  CM点=%zu",
        pick_top_z, place_top_z, g_table_obstacles.size());
}

// 全物体を Planning Scene に登録し、g_obstacles を構築する
// (テーブル点は g_table_obstacles から引き継ぎ)
static void addObjectsToScene() {
    if (!g_psi) return;
    g_obstacles = g_table_obstacles;  // テーブル点を先に入れる
    std::vector<moveit_msgs::msg::CollisionObject> objs;
    objs.reserve(OBJ_POSITIONS.size());

    for (size_t i = 0; i < OBJ_POSITIONS.size(); ++i) {
        moveit_msgs::msg::CollisionObject obj;
        obj.id              = "obj_" + std::to_string(i);
        obj.header.frame_id = "base_footprint";
        obj.operation       = obj.ADD;

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.CYLINDER;
        prim.dimensions.resize(2);
        prim.dimensions[0] = OBJ_HEIGHT;
        prim.dimensions[1] = OBJ_RADIUS;

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x    = OBJ_POSITIONS[i].x;
        pose.position.y    = OBJ_POSITIONS[i].y;
        pose.position.z    = OBJ_POSITIONS[i].z;
        obj.primitives.push_back(prim);
        obj.primitive_poses.push_back(pose);
        objs.push_back(obj);

        // CM 解析用障害物点（中心・上端・下端）
        Eigen::Vector3d c(OBJ_POSITIONS[i].x, OBJ_POSITIONS[i].y, OBJ_POSITIONS[i].z);
        g_obstacles.push_back({c});
        g_obstacles.push_back({c + Eigen::Vector3d(0, 0, +OBJ_HEIGHT / 2.0)});
        g_obstacles.push_back({c + Eigen::Vector3d(0, 0, -OBJ_HEIGHT / 2.0)});
    }
    g_psi->applyCollisionObjects(objs);
}

// 把持後に単体物体をシーンへ再登録する（attachObject の前に必要）
static void addObjectToScene(size_t idx) {
    if (!g_psi) return;
    moveit_msgs::msg::CollisionObject obj;
    obj.id              = "obj_" + std::to_string(idx);
    obj.header.frame_id = "base_footprint";
    obj.operation       = obj.ADD;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.CYLINDER;
    prim.dimensions.resize(2);
    prim.dimensions[0] = OBJ_HEIGHT;
    prim.dimensions[1] = OBJ_RADIUS;

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x    = OBJ_POSITIONS[idx].x;
    pose.position.y    = OBJ_POSITIONS[idx].y;
    pose.position.z    = OBJ_POSITIONS[idx].z;
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    g_psi->applyCollisionObject(obj);
    rclcpp::sleep_for(200ms);
}

// ピック前に対象物体を場面から除去し、g_obstacles からも除外する
static void removeObjectFromScene(size_t idx) {
    if (!g_psi) return;
    g_psi->removeCollisionObjects({"obj_" + std::to_string(idx)});
    rclcpp::sleep_for(300ms);

    // テーブル点 → 残りピック物体点 → 配置済み物体点 の順で再構築
    g_obstacles = g_table_obstacles;
    for (size_t i = 0; i < OBJ_POSITIONS.size(); ++i) {
        if (i == idx) continue;
        Eigen::Vector3d c(OBJ_POSITIONS[i].x, OBJ_POSITIONS[i].y, OBJ_POSITIONS[i].z);
        g_obstacles.push_back({c});
        g_obstacles.push_back({c + Eigen::Vector3d(0, 0, +OBJ_HEIGHT / 2.0)});
        g_obstacles.push_back({c + Eigen::Vector3d(0, 0, -OBJ_HEIGHT / 2.0)});
    }
    // 配置済み物体点も追加（前サイクルで置いた物体を考慮）
    for (const auto &obs : g_placed_obstacles)
        g_obstacles.push_back(obs);
}

// 配置成功後に物体をシーンへ障害物として登録し、
// g_placed_obstacles / g_obstacles にも追加する
// （次サイクルの経路計画・CM解析で配置済み物体を考慮するため）
// jointfix_10 と同様に「アームが離れた後（retreat完了後）」に呼ぶこと
static void addPlacedObjectToScene(size_t idx) {
    if (!g_psi) return;
    const Pose3D &pl = PLACE_POSITIONS[idx];

    moveit_msgs::msg::CollisionObject obj;
    obj.id              = "placed_" + std::to_string(idx);
    obj.header.frame_id = "base_footprint";
    obj.operation       = obj.ADD;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.CYLINDER;
    prim.dimensions.resize(2);
    prim.dimensions[0] = OBJ_HEIGHT;
    prim.dimensions[1] = OBJ_RADIUS;

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x    = pl.x;
    pose.position.y    = pl.y;
    pose.position.z    = pl.z;
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    g_psi->applyCollisionObject(obj);
    rclcpp::sleep_for(300ms);

    // CM解析用: 中心・上端・下端の3点
    Eigen::Vector3d c(pl.x, pl.y, pl.z);
    g_placed_obstacles.push_back({c});
    g_placed_obstacles.push_back({c + Eigen::Vector3d(0, 0, +OBJ_HEIGHT / 2.0)});
    g_placed_obstacles.push_back({c + Eigen::Vector3d(0, 0, -OBJ_HEIGHT / 2.0)});

    // g_obstacles も即時反映（次サイクルの removeObjectFromScene 前でも有効に）
    g_obstacles.push_back({c});
    g_obstacles.push_back({c + Eigen::Vector3d(0, 0, +OBJ_HEIGHT / 2.0)});
    g_obstacles.push_back({c + Eigen::Vector3d(0, 0, -OBJ_HEIGHT / 2.0)});

    RCLCPP_INFO(rclcpp::get_logger("scene"),
        "配置済み障害物登録: placed_%zu (%.2f, %.2f, %.2f)", idx, pl.x, pl.y, pl.z);
}

// ═══════════════════════════════════════════════════════════════════════════
// Pick & Place シーケンス
// ═══════════════════════════════════════════════════════════════════════════

static bool pickAndPlace(
    rclcpp::Node::SharedPtr node,
    MoveGroupInterface &arm,
    GripperClient::SharedPtr gripper_client,
    size_t idx,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub)
{
    const Pose3D &obj   = OBJ_POSITIONS[idx];
    const Pose3D &place = PLACE_POSITIONS[idx];
    int trial = (int)idx;

    RCLCPP_INFO(node->get_logger(),
        "=== [%zu] Pick(%.2f,%.2f,%.2f) → Place(%.2f,%.2f,%.2f) ===",
        idx, obj.x, obj.y, obj.z, place.x, place.y, place.z);

    // 1. グリッパー開放
    controlGripper(node, gripper_client, GRIPPER_OPEN);

    // 2. ピック対象をシーンから除去（自分自身への衝突判定を回避）
    removeObjectFromScene(idx);

    // 3. アプローチ（拘束あり）
    if (!moveLogged(node, arm, trial, "approach_" + std::to_string(idx),
                    obj.x, obj.y, obj.z + APPROACH_HEIGHT, 0.5, true, marker_pub))
        return false;

    // 4. 降下（拘束あり）
    if (!moveLogged(node, arm, trial, "descend_" + std::to_string(idx),
                    obj.x, obj.y, obj.z, 0.2, true, marker_pub))
        return false;

    // 5. 把持
    RCLCPP_INFO(node->get_logger(), "[%zu] 段階的把持...", idx);
    closeGripperGradually(node, gripper_client, GRIPPER_CLOSE);

    // 5.5 把持物体をアームへアタッチ（touch_links でグリッパーとの誤衝突を抑制）
    //     attachObject はワールドにオブジェクトが存在している必要があるため再登録してからアタッチ
    const std::string obj_id = "obj_" + std::to_string(idx);
    addObjectToScene(idx);
    arm.attachObject(obj_id, "gripper_base_1", TOUCH_LINKS);
    rclcpp::sleep_for(500ms);

    // 6. 持ち上げ（拘束あり）
    if (!moveLogged(node, arm, trial, "lift_" + std::to_string(idx),
                    obj.x, obj.y, obj.z + APPROACH_HEIGHT, 0.3, true, marker_pub))
        return false;

    // 7. 配置位置上方へ（拘束なし・把持後）
    if (!moveLogged(node, arm, trial, "to_place_" + std::to_string(idx),
                    place.x, place.y, place.z + APPROACH_HEIGHT, 0.5, true, marker_pub))
        return false;

    // 8. 配置位置へ降下（拘束なし: 配置エリアは把持エリアと逆側で制約を満たせない場合がある）
    if (!moveLogged(node, arm, trial, "place_down_" + std::to_string(idx),
                    place.x, place.y, place.z, 0.2, true, marker_pub))
        return false;

    // 9. グリッパー開放
    controlGripper(node, gripper_client, GRIPPER_OPEN);

    // 9.5 把持物体をデタッチし、シーンからも除去
    arm.detachObject(obj_id);
    rclcpp::sleep_for(200ms);
    g_psi->removeCollisionObjects({obj_id});

    RCLCPP_INFO(node->get_logger(), "[%zu] 配置完了。", idx);

    // 10. 退避（拘束なし）
    moveLogged(node, arm, trial, "retreat_" + std::to_string(idx),
               place.x, place.y, place.z + APPROACH_HEIGHT, 0.3, false, marker_pub);

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    opts.parameter_overrides({{"use_sim_time", true}});
    auto node = rclcpp::Node::make_shared("jointfix_cm_pick_place", opts);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    // condition_id をパラメータから取得可能に
    if (node->has_parameter("condition_id"))
        g_condition_id = node->get_parameter("condition_id").as_string();

    // 実行ごとのタイムスタンプフォルダを作成
    {
        std::time_t t = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
        g_run_dir = LOG_ROOT + std::string(buf) + "/";
        std::filesystem::create_directories(g_run_dir);
    }

    auto gripper_client = rclcpp_action::create_client<GripperCommand>(
        node, "/gripper_controller/gripper_cmd");

    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/manip_markers", 10);

    MoveGroupInterface arm(node, "arm");
    arm.setPoseReferenceFrame("base_footprint");
    arm.setPlanningTime(30.0);
    arm.setNumPlanningAttempts(5);
    arm.setPlanningPipelineId("ompl");
    arm.setPlannerId("RRTConnectkConfigDefault");

    PlanningSceneInterface psi;
    g_psi = &psi;

    rclcpp::sleep_for(1s);
    addTableObstacles();   // ピック台・プレース台を先に登録（g_table_obstacles も構築）
    rclcpp::sleep_for(300ms);
    addObjectsToScene();   // ピック物体を登録（g_obstacles = テーブル点 + 物体点）
    rclcpp::sleep_for(500ms);

    const size_t total = std::min(OBJ_POSITIONS.size(), PLACE_POSITIONS.size());
    RCLCPP_INFO(node->get_logger(),
        "=== jointfix_cm Pick & Place 開始 (%zu 個) ===", total);
    RCLCPP_INFO(node->get_logger(),
        "    CSV 出力先: %s", g_run_dir.c_str());
    RCLCPP_INFO(node->get_logger(),
        "    条件ID: %s", g_condition_id.c_str());

    // 各サイクル前のホームリセット（前サイクル失敗時の連鎖を防ぐ）
    // initial_posi_gz.cpp に合わせたホーム関節角度 [rad]
    // Joint_1=0.00, Joint_2=2.3, Joint_3=-2.3, Joint_4=-0.2, Joint_5=0.0
    auto returnHome = [&]() {
        RCLCPP_INFO(node->get_logger(), "ホーム位置へリセット...");
        arm.setStartStateToCurrentState();
        arm.clearPathConstraints();
        std::map<std::string, double> home_joints = {
            {"Joint_1",  0.00},
            {"Joint_2",  2.3 },
            {"Joint_3", -2.3 },
            {"Joint_4", -0.2 },
            {"Joint_5",  0.0 },
        };
        arm.setJointValueTarget(home_joints);
        auto r = arm.move();
        if (r != moveit::core::MoveItErrorCode::SUCCESS)
            RCLCPP_WARN(node->get_logger(), "ホームへの移動失敗 (code=%d)", r.val);
    };

    size_t success_count = 0;
    for (size_t i = 0; i < total && rclcpp::ok(); ++i) {
        returnHome();  // サイクルごとに必ずホームから開始
        if (pickAndPlace(node, arm, gripper_client, i, marker_pub)) {
            ++success_count;
            // retreat 完了後（アームが離れた後）に配置済み物体を障害物として登録
            // → 次サイクルの経路計画・CM解析でこの物体を考慮できる（jointfix_10 と同じ方針）
            addPlacedObjectToScene(i);
        } else {
            RCLCPP_WARN(node->get_logger(),
                "[%zu] Pick & Place 失敗。次の物体へスキップ。", i);
        }
    }

    RCLCPP_INFO(node->get_logger(),
        "=== 全シーケンス終了: %zu / %zu 成功 ===", success_count, total);
    RCLCPP_INFO(node->get_logger(),
        "CSV 保存先: %scm_traj.csv / df_summary.csv", g_run_dir.c_str());

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
