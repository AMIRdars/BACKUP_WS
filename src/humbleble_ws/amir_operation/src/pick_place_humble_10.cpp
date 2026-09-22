 /*
 * ROS 2 Humble 向け Pick and Place サンプル
 */

// 把持物attachありの障害物回避 横から掴む　把持物の追加
//plan の joint_trajectory の各点 k ごとにデータを追加
//Collision余裕のMを計算して、データとして出す

//MoveIt2（RRTConnect）で「ペットボトルを掴んで運び、別の場所に置く」までを動かしつつ、計画軌道（plan の joint_trajectory）を使って
//Danger Field（DF）指標
//Collision Margin（CM：あなたの collision余裕度）を軌道点ごと＋軌道全体で計算してCSVに保存し、さらにRVizに矢印マーカーで可視化する。
//ロボット側の代表点と障害物側の代表点を増やした
//３つの中から一番Collision余裕度が高いものを選んで、実行する

 #include <rclcpp/rclcpp.hpp>
 #include <moveit/planning_scene_interface/planning_scene_interface.h>
 #include <moveit/move_group_interface/move_group_interface.h>
 #include <moveit_msgs/msg/collision_object.hpp>
 #include <shape_msgs/msg/solid_primitive.hpp>
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 #include <geometry_msgs/msg/pose.hpp>
 #include <chrono>
 #include <vector>
 #include <cmath>
 #include <thread>

 // 評価をするために追加
 #include <trajectory_msgs/msg/joint_trajectory.hpp>
 #include <moveit/robot_state/robot_state.h>
 #include <moveit/robot_model/joint_model_group.h>
 #include <Eigen/Dense>
 #include <fstream>
 #include <limits>
 #include <iostream>
 #include <algorithm>
 #include <visualization_msgs/msg/marker_array.hpp>
 #include <geometry_msgs/msg/point.hpp>
 
 using namespace std::chrono_literals;
 using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
 using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
 
 // ベースからテーブルまでの高さ
 const double BASE_HEIGHT = 0.248;
 // 物体の大きさ (x, y, z)
 const std::vector<double> OBJECT_DIMENSION = {0.075, 0.03, 0.13};
 // 物体のワールド座標系での位置 (x, y, z)
 const std::vector<double> OBJECT_POSITION = {
   0.4,
   -0.018171 + OBJECT_DIMENSION[1] / 2.0,
   -BASE_HEIGHT + OBJECT_DIMENSION[2] / 2.0
 };

 // メカナム分(166mm) 下げる
const double Z_DOWN = -0.166+0.06;  // [m]
 
 // 度をラジアンに変換
 inline double deg2rad(double deg) {
   return deg * M_PI / 180.0;
 }

 struct XY { double x; double y; };

 struct Obstacle { Eigen::Vector3d center; };
 static std::vector<Obstacle> g_obstacles;
 static double g_bottle_r = 0.035;  // Rと一致させる

 static inline Eigen::Vector3d getLinkOriginW(const moveit::core::RobotState& st,
                                             const std::string& link)
{
  return st.getGlobalLinkTransform(link).translation();
}

  // ===================== DF (Danger Field) 定義側コード一式 =====================

struct DFParams {
  double R = 0.035;        // ボトル半径 [m]
  double eps = 0.001;      // d' = d + eps の eps [m]
  double k1 = 1.0;         // DFd 係数
  double k2 = 1.0;         // DFa 係数
  double k3 = 1.0;         // DFσ 係数
  double gamma = 1.0;      // 論文準拠（>=1 推奨）
  double sigma0 = 0.05;    // 安全基準（仮）※後でデータから調整OK
  double Delta = 50.0;     // DFしきい値（仮）※後で調整OK
  double v_eps = 1e-6;     // 速度ゼロ割回避
};

struct RepPoint {
  std::string link;          // 座標系リンク
  Eigen::Vector3d offset;    // link座標系のオフセット
  std::string name;          // CSV表示名
};

static inline Eigen::Vector3d getPointW(const moveit::core::RobotState& st,
                                        const RepPoint& rp)
{
  const Eigen::Isometry3d& T = st.getGlobalLinkTransform(rp.link);
  return T.translation() + T.rotation() * rp.offset;
}


struct DFLogRow {
  int plan_success = 0;
  int exec_success = -1; // 未入力=-1
  std::string fail_mode = "NA";

  double dmin = std::numeric_limits<double>::infinity();
  double t_dmin = 0.0; int idx_dmin = -1;

  double amax = 0.0;       // max(max(0,cosφ))
  double t_amax = 0.0; int idx_amax = -1;

  double sigmin = std::numeric_limits<double>::infinity();
  double t_sigmin = 0.0; int idx_sigmin = -1;

  double DFmax = 0.0;
  double t_DFmax = 0.0; int idx_DFmax = -1;

  double Lrisk = 0.0;  // DF>Δ の区間長（ds足し）
  double Rrisk = 0.0;  // DF積分（DF*ds）
};

// 文字列をCSV安全に（最低限）
static std::string csv_escape(const std::string& s){
  if (s.find(',')==std::string::npos && s.find('"')==std::string::npos) return s;
  std::string out="\"";
  for(char c: s){ out += (c=='"'? "\"\"" : std::string(1,c)); }
  out += "\"";
  return out;
}

// EEリンク上の点（ee_offset_xyz: EE座標系のオフセット）を world に変換して返す
static Eigen::Vector3d getPointPk(const moveit::core::RobotState& st,
                                  const std::string& ee_link,
                                  const Eigen::Vector3d& ee_offset_xyz)
{
  const Eigen::Isometry3d& T = st.getGlobalLinkTransform(ee_link);
  return T.translation() + T.rotation() * ee_offset_xyz;
}

// 最近傍障害物とクリアランス d = ||p - o|| - R
static inline void nearestObstacle(const Eigen::Vector3d& p,
                                   const std::vector<Obstacle>& obs,
                                   double R,
                                   int& j_star,
                                   double& d_clear,
                                   Eigen::Vector3d& o_star)
{
  j_star = -1;
  d_clear = std::numeric_limits<double>::infinity();
  o_star = Eigen::Vector3d::Zero();

  for (int j=0; j<(int)obs.size(); ++j){
    const double d = (p - obs[j].center).norm() - R;
    if (d < d_clear){
      d_clear = d;
      j_star = j;
      o_star = obs[j].center;
    }
  }
}

// その姿勢でのヤコビアン最小特異値 σmin
static double minSingularValueJacobian(const moveit::core::RobotState& st,
                                       const moveit::core::JointModelGroup* jmg,
                                       const std::string& ee_link)
{
  Eigen::MatrixXd J;
  Eigen::Vector3d ref(0,0,0); // EE原点
  st.getJacobian(jmg, st.getLinkModel(ee_link), ref, J);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& s = svd.singularValues();
  if (s.size()==0) return 0.0;
  return s.minCoeff();
}

// ここが本体：trajectoryを舐めて DF系ログを作る
static DFLogRow analyzeTrajectoryMultiLinks(
    const MoveGroupInterface& arm,
    const trajectory_msgs::msg::JointTrajectory& jt,
    const std::vector<Obstacle>& obstacles,
    const DFParams& prm,
    const std::vector<RepPoint>& rep_points
){
  DFLogRow out;
  if (jt.points.size() < 2) return out;

  auto robot_model = arm.getRobotModel();
  const auto* jmg = robot_model->getJointModelGroup(arm.getName());
  if (!jmg) return out;

  for (size_t k=0; k+1<jt.points.size(); ++k){
    moveit::core::RobotState st(robot_model);
    st.setToDefaultValues();
    st.setVariablePositions(jt.joint_names, jt.points[k].positions);
    st.update();

    moveit::core::RobotState st2(robot_model);
    st2.setToDefaultValues();
    st2.setVariablePositions(jt.joint_names, jt.points[k+1].positions);
    st2.update();

    const double t  = rclcpp::Duration(jt.points[k].time_from_start).seconds();
    const double t2 = rclcpp::Duration(jt.points[k+1].time_from_start).seconds();
    const double dt = std::max(1e-6, t2 - t);

    // 特異値は姿勢で決まるので1回だけ
    // ee_linkは「ヤコビアンを取りたいリンク」でOK（tcp_linkで良い）
    const std::string jac_link = "tcp_link";
    const double sigma = minSingularValueJacobian(st, jmg, jac_link);
    const double DFsig = prm.k3 * std::max(0.0, (prm.sigma0 - sigma) / prm.sigma0);

    // この区間の最悪（最大）DFd+DFa を探す
    double DF_point_max = 0.0;
    double a_k_max = 0.0;
    double d_clear_min_all_points = std::numeric_limits<double>::infinity();

    // dsは「どの点の移動量で測る？」問題があるので、最悪に合わせて max(ds) にしておくと安全
    double ds_max = 0.0;

    for (const auto& rp : rep_points){
    const Eigen::Vector3d p  = getPointW(st,  rp);
    const Eigen::Vector3d p2 = getPointW(st2, rp);

      const double ds = (p2 - p).norm();
      ds_max = std::max(ds_max, ds);

      // 最近傍障害物
      int j_star; double d_clear; Eigen::Vector3d o_star;
      nearestObstacle(p, obstacles, prm.R, j_star, d_clear, o_star);
      d_clear_min_all_points = std::min(d_clear_min_all_points, d_clear);

      const double dprime = d_clear + prm.eps;

      // DFd
      const double DFd = prm.k1 / dprime;

      // DFa
      const Eigen::Vector3d v = (p2 - p) / dt;
      const double vnorm = v.norm();
      double cosphi = 0.0;
      if (vnorm > prm.v_eps){
        const Eigen::Vector3d g = (o_star - p);
        const double gnorm = g.norm();
        if (gnorm > 1e-9){
          cosphi = (v / vnorm).dot(g / gnorm);
        }
      }
      const double a_k = std::max(0.0, cosphi);
      a_k_max = std::max(a_k_max, a_k);

      const double dir_term = std::max(0.0, prm.gamma + cosphi);
      const double DFa = (prm.k2 * vnorm * dir_term) / (dprime * dprime);

      DF_point_max = std::max(DF_point_max, DFd + DFa);
    }

    // 合成DF（点の最悪 + 特異値）
    const double DF = DF_point_max + DFsig;

    // ログ更新（ここは「4点をまとめた代表値」にする）
    if (d_clear_min_all_points < out.dmin){
      out.dmin = d_clear_min_all_points; out.t_dmin = t; out.idx_dmin = (int)k;
    }
    if (a_k_max > out.amax){
      out.amax = a_k_max; out.t_amax = t; out.idx_amax = (int)k;
    }
    if (sigma < out.sigmin){
      out.sigmin = sigma; out.t_sigmin = t; out.idx_sigmin = (int)k;
    }
    if (DF > out.DFmax){
      out.DFmax = DF; out.t_DFmax = t; out.idx_DFmax = (int)k;
    }

    if (DF > prm.Delta) out.Lrisk += ds_max;
    out.Rrisk += DF * ds_max;
  }
  return out;
}



// ===== 時系列CSV（plan軌道の各点kで dmin/amax/sigma/DF を出す）=====

// 位置ヤコビアン(3xN)の最小特異値も見たい場合用
static double minSingularValueJacobianPos(const moveit::core::RobotState& st,
                                          const moveit::core::JointModelGroup* jmg,
                                          const std::string& ee_link)
{
  Eigen::MatrixXd J;
  Eigen::Vector3d ref(0,0,0);
  st.getJacobian(jmg, st.getLinkModel(ee_link), ref, J); // 6xN
  Eigen::MatrixXd Jpos = J.topRows(3);                  // 3xN

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jpos, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& s = svd.singularValues();
  if (s.size()==0) return 0.0;
  return s.minCoeff();
}

static void appendTimeSeriesCsvPlan(
  const std::string& path,
  const std::string& condition_id,
  int trial,
  const std::string& stage,
  double t,
  int idx,
  double dmin,
  double amax,
  double sigma_full,
  double sigma_pos,
  double DF
){
  const bool new_file = !std::ifstream(path).good();
  std::ofstream ofs(path, std::ios::app);
  if (new_file){
    ofs << "condition_id,trial,stage,idx,t,"
           "dmin_m,amax,"
           "sigma_min_full,sigma_min_pos,"
           "DF\n";
  }
  ofs << csv_escape(condition_id) << ","
      << trial << ","
      << csv_escape(stage) << ","
      << idx << ","
      << t << ","
      << dmin << ","
      << amax << ","
      << sigma_full << ","
      << sigma_pos << ","
      << DF << "\n";
}

// plan軌道から、各kでの代表値(dmin/amax/sigma/DF)をCSVに吐く
static void exportPlanTimeSeriesMultiLinks(
  const MoveGroupInterface& arm,
  const trajectory_msgs::msg::JointTrajectory& jt,
  const std::vector<Obstacle>& obstacles,
  const DFParams& prm,
  const std::vector<RepPoint>& rep_points,
  const std::string& csv_path,
  const std::string& condition_id,
  int trial,
  const std::string& stage
){
  if (jt.points.size() < 2) return;

  auto robot_model = arm.getRobotModel();
  const auto* jmg = robot_model->getJointModelGroup(arm.getName());
  if (!jmg) return;

  moveit::core::RobotState st(robot_model);
  moveit::core::RobotState st2(robot_model);
  st.setToDefaultValues();
  st2.setToDefaultValues();

  const std::string jac_link = "tcp_link";

  for (size_t k=0; k+1<jt.points.size(); ++k){
    st.setVariablePositions(jt.joint_names, jt.points[k].positions);
    st.update();
    st2.setVariablePositions(jt.joint_names, jt.points[k+1].positions);
    st2.update();

    const double t  = rclcpp::Duration(jt.points[k].time_from_start).seconds();
    const double t2 = rclcpp::Duration(jt.points[k+1].time_from_start).seconds();
    const double dt = std::max(1e-6, t2 - t);

    // --- 特異値（full と pos の両方）
    const double sigma_full = minSingularValueJacobian(st, jmg, jac_link);
    const double sigma_pos  = minSingularValueJacobianPos(st, jmg, jac_link);

    // sigma項（あなたのDF定義そのまま）
    const double DFsig = prm.k3 * std::max(0.0, (prm.sigma0 - sigma_full) / prm.sigma0);

    // --- 点（rep_links）で距離/侵入角を計算して代表値にする
    double d_clear_min_all_points = std::numeric_limits<double>::infinity();
    double a_k_max = 0.0;
    double DF_point_max = 0.0;

    for (const auto& rp : rep_points){
      const Eigen::Vector3d p  = getPointW(st,  rp);
      const Eigen::Vector3d p2 = getPointW(st2, rp);

      // 最近傍障害物
      int j_star; double d_clear; Eigen::Vector3d o_star;
      nearestObstacle(p, obstacles, prm.R, j_star, d_clear, o_star);

      d_clear_min_all_points = std::min(d_clear_min_all_points, d_clear);
      const double dprime = d_clear + prm.eps;

      // DFd
      const double DFd = prm.k1 / dprime;

      // 侵入角（cosphi）と DFa
      const Eigen::Vector3d v = (p2 - p) / dt;
      const double vnorm = v.norm();
      double cosphi = 0.0;
      if (vnorm > prm.v_eps){
        const Eigen::Vector3d g = (o_star - p);
        const double gnorm = g.norm();
        if (gnorm > 1e-9){
          cosphi = (v / vnorm).dot(g / gnorm);
        }
      }
      const double a_k = std::max(0.0, cosphi);
      a_k_max = std::max(a_k_max, a_k);

      const double dir_term = std::max(0.0, prm.gamma + cosphi);
      const double DFa = (prm.k2 * vnorm * dir_term) / (dprime * dprime);

      DF_point_max = std::max(DF_point_max, DFd + DFa);
    }

    const double DF = DF_point_max + DFsig;

    // --- 1行出力（kの時刻の代表値）
    appendTimeSeriesCsvPlan(csv_path, condition_id, trial, stage,
                            t, (int)k,
                            d_clear_min_all_points, a_k_max,
                            sigma_full, sigma_pos,
                            DF);
  }
}

// CSV追記
static void appendCsv(const std::string& path,
                      const std::string& condition_id,
                      int trial,
                      const DFLogRow& r)
{
  const bool new_file = !std::ifstream(path).good();
  std::ofstream ofs(path, std::ios::app);
  if (new_file){
    ofs << "condition_id,trial,plan_success,exec_success,fail_mode,"
           "dmin_m,t_dmin,idx_dmin,"
           "amax,t_amax,idx_amax,"
           "sigmin,t_sigmin,idx_sigmin,"
           "DFmax,t_DFmax,idx_DFmax,"
           "Lrisk,Rrisk\n";
  }
  ofs << csv_escape(condition_id) << "," << trial << ","
      << r.plan_success << "," << r.exec_success << "," << csv_escape(r.fail_mode) << ","
      << r.dmin << "," << r.t_dmin << "," << r.idx_dmin << ","
      << r.amax << "," << r.t_amax << "," << r.idx_amax << ","
      << r.sigmin << "," << r.t_sigmin << "," << r.idx_sigmin << ","
      << r.DFmax << "," << r.t_DFmax << "," << r.idx_DFmax << ","
      << r.Lrisk << "," << r.Rrisk << "\n";
}

static void appendEllipsoidCsv(const std::string& path,
                               const std::string& condition_id,
                               int trial,
                               const std::string& stage,
                               int idx,
                               double t,
                               double s1, double s2, double s3,
                               double w,
                               const Eigen::Vector3d& u1,
                               const Eigen::Vector3d& u2,
                               const Eigen::Vector3d& u3)
{
  const bool new_file = !std::ifstream(path).good();
  std::ofstream ofs(path, std::ios::app);
  if (new_file){
    ofs << "condition_id,trial,stage,idx,t,"
           "sigma1,sigma2,sigma3,volume_w,"
           "u1x,u1y,u1z,u2x,u2y,u2z,u3x,u3y,u3z\n";
  }
  ofs << csv_escape(condition_id) << ","
      << trial << ","
      << csv_escape(stage) << ","
      << idx << ","
      << t << ","
      << s1 << "," << s2 << "," << s3 << ","
      << w  << ","
      << u1.x() << "," << u1.y() << "," << u1.z() << ","
      << u2.x() << "," << u2.y() << "," << u2.z() << ","
      << u3.x() << "," << u3.y() << "," << u3.z()
      << "\n";
}

  // ===== DFParams（実験用に固定でOK）=====
  static DFParams g_prm;                 // デフォルト値は struct 内の初期値が使われる
  static std::string g_csv_path = "/tmp/df_log.csv";
  static std::string g_condition_id = "cond_single";  // 1条件だけなら固定でOK
  static std::string g_cm_traj_csv = "/tmp/collision_margin_traj.csv";
  static std::string g_cm_ts_csv   = "/tmp/collision_margin_timeseries.csv";

// ===================== DF 定義側コードここまで =====================

// ===================== Collision Margin (CM) =====================

struct CMLogRow {
  int plan_success = 0;
  int exec_success = -1;

  double plan_time_s = 0.0;

  double M = 0.0;        // time-avg
  double m_min = 1.0;    // min over points
  double d_min = std::numeric_limits<double>::infinity(); // min clearance [m]
  double T = 0.0;
  int n_points = 0;
};

static void appendCmTrajCsv(const std::string& path,
                            const std::string& condition_id,
                            int trial,
                            const std::string& stage,
                            const CMLogRow& r)
{
  const bool new_file = !std::ifstream(path).good();
  std::ofstream ofs(path, std::ios::app);
  if (new_file){
    ofs << "condition_id,trial,stage,"
           "plan_success,plan_time_s,exec_success,"
           "M,m_min,d_min,T,n_points\n";
  }
  ofs << csv_escape(condition_id) << ","
      << trial << ","
      << csv_escape(stage) << ","
      << r.plan_success << ","
      << r.plan_time_s << ","
      << r.exec_success << ","
      << r.M << ","
      << r.m_min << ","
      << r.d_min << ","
      << r.T << ","
      << r.n_points << "\n";
}

static void appendCmTimeSeriesCsv(const std::string& path,
                                  const std::string& condition_id,
                                  int trial,
                                  const std::string& stage,
                                  int idx,
                                  double t,
                                  double m_k,
                                  double d_k,
                                  double r_k,
                                  double rho,                 
                                  double alpha,               
                                  const Eigen::Vector3d& ghat,
                                  const Eigen::Vector3d& ulong,
                                  const std::string& link_star)
{
  const bool new_file = !std::ifstream(path).good();
  std::ofstream ofs(path, std::ios::app);
  if (new_file){
    ofs << "condition_id,trial,stage,idx,t,"
           "m_k,d_k,r_k,rho,alpha,"
           "ghx,ghy,ghz,ulx,uly,ulz,"
           "link_star\n";
  }
  ofs << csv_escape(condition_id) << ","
      << trial << ","
      << csv_escape(stage) << ","
      << idx << ","
      << t << ","
      << m_k << ","
      << d_k << ","
      << r_k << ","
      << rho << ","
      << alpha << ","
      << ghat.x() << "," << ghat.y() << "," << ghat.z() << ","
      << ulong.x() << "," << ulong.y() << "," << ulong.z() << ","
      << csv_escape(link_star) << "\n";
}

// 1点のm(t)を計算する関数
static bool calc_m_d_r_at_state(
    const moveit::core::RobotState& st,
    const moveit::core::JointModelGroup* jmg,
    const std::string& jac_link,          // Jacobian取るリンク（tcp_link推奨）
    const Eigen::Vector3d& p,             // 評価点（リンク原点）
    const Eigen::Vector3d& obs_center,    // 障害物中心
    double R,                             // 障害物半径
    double lambda,                        // damping
    double& out_m,
    double& out_d,
    double& out_r,
    Eigen::Vector3d& out_g_hat,   
    Eigen::Vector3d& out_o_star   
){
  const Eigen::Vector3d c = obs_center;
  Eigen::Vector3d pc = (c - p);
  const double pc_norm = pc.norm();
  if (pc_norm < 1e-9) return false;

  // 表面最近接点 o = c - R * (c-p)/||c-p||
  Eigen::Vector3d o = c - R * (pc / pc_norm);
  out_o_star = o;

  Eigen::Vector3d g = (o - p);
  double d = g.norm();         // = ||c-p|| - R （クリアランス）
  if (d <= 1e-6) {             // めり込み/接触扱い
    out_m = 0.0;
    out_d = std::max(0.0, d);
    out_r = 0.0;
    out_g_hat = Eigen::Vector3d::Zero();
    return true;
  }
  Eigen::Vector3d gh = g / d;
  out_g_hat = gh;

  // Jacobian（6xN） -> 位置3xN
  Eigen::MatrixXd J;
  st.getJacobian(jmg, st.getLinkModel(jac_link), Eigen::Vector3d::Zero(), J);
  Eigen::MatrixXd Jpos = J.topRows(3);
  Eigen::Matrix3d A = (Jpos * Jpos.transpose());

  // damping
  Eigen::Matrix3d Ainv = (A + lambda * Eigen::Matrix3d::Identity()).inverse();

  double denom = gh.transpose() * Ainv * gh;
  denom = std::max(1e-12, denom);
  double r = 1.0 / std::sqrt(denom);

  // ★追加：r_k をスケール（0.115 を掛ける）
  r *= 0.115;

  double m = 1.0 - (r / d);
  m = std::clamp(m, 0.0, 1.0);

  out_m = m;
  out_d = d;
  out_r = r;
  return true;
}

static void publishEllipsoidAxes(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
    const Eigen::Vector3d& tcp_pos,
    const Eigen::Vector3d& u1, const Eigen::Vector3d& u2, const Eigen::Vector3d& u3,
    double s1, double s2, double s3,
    const Eigen::Vector3d& p_star,
    const Eigen::Vector3d& g_hat,
    const std::string& frame_id);

// 軌道全体の M, m_min, d_min を計算し、点ごとCSVも吐く関数
static CMLogRow analyzeCollisionMarginTrajectory(
    const MoveGroupInterface& arm,
    const trajectory_msgs::msg::JointTrajectory& jt,
    const std::vector<Obstacle>& obstacles,
    double R,
    const std::vector<RepPoint>& rep_points,
    const std::string& jac_link,
    const std::string& ts_csv_path,
    const std::string& condition_id,
    int trial,
    const std::string& stage,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub
){
  CMLogRow out;
  if (jt.points.size() < 2) return out;

  auto robot_model = arm.getRobotModel();
  const auto* jmg = robot_model->getJointModelGroup(arm.getName());
  if (!jmg) return out;

  moveit::core::RobotState st(robot_model);
  st.setToDefaultValues();

  double num = 0.0, den = 0.0;
  out.m_min = 1.0;
  out.d_min = std::numeric_limits<double>::infinity();

  // ダンピング（まずは固定でOK）
  const double lambda = 1e-4;

  rclcpp::Duration prev_t(0,0);

  for (size_t k = 0; k + 1 < jt.points.size(); ++k) {
  st.setVariablePositions(jt.joint_names, jt.points[k].positions);
  st.update();

  const double t  = rclcpp::Duration(jt.points[k].time_from_start).seconds();
  const double t2 = rclcpp::Duration(jt.points[k+1].time_from_start).seconds();
  const double dt = std::max(0.0, t2 - t);

  // -----------------------------
  // 1) まず「最悪リンク」を決める（mk,dk,rk,g_hat_star）
  // -----------------------------
  double mk = 1.0;
  double dk = std::numeric_limits<double>::infinity();
  double rk = 0.0;
  std::string link_star = "NA";
  Eigen::Vector3d g_hat_star = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_star = Eigen::Vector3d::Zero();

  for (const auto& rp : rep_points) {
  const Eigen::Vector3d p = getPointW(st, rp);

  int j_star; double d_clear; Eigen::Vector3d c_star;
  nearestObstacle(p, obstacles, R, j_star, d_clear, c_star);
  if (j_star < 0) continue;

  double m_i, d_i, r_i;
  Eigen::Vector3d ghat_i, o_star_i;
  if (!calc_m_d_r_at_state(st, jmg, jac_link, p, c_star, R, lambda,
                           m_i, d_i, r_i, ghat_i, o_star_i))
    continue;

  if (m_i < mk) {
    mk = m_i;
    dk = d_i;
    rk = r_i;
    link_star = rp.name;   // ★どの点が最悪か
    g_hat_star = ghat_i;
    p_star = p;
  }
}

  // 障害物が無い/計算不能ならスキップ（ログだけ出したいならcontinueせずNAで出してもOK）
  if (link_star == "NA") {
    // 時間平均は足さない（or mk=1扱いにする等、方針次第）
    appendCmTimeSeriesCsv(ts_csv_path, condition_id, trial, stage,
                          (int)k, t,
                          1.0, std::numeric_limits<double>::infinity(), 0.0,
                          0.0, 0.0,
                          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                          "NA");
    continue;
  }

  // -----------------------------
  // 2) 次にJposの楕円体（U, u_long）を計算
  // -----------------------------
  Eigen::MatrixXd J;
  st.getJacobian(jmg, st.getLinkModel(jac_link), Eigen::Vector3d::Zero(), J);
  Eigen::MatrixXd Jpos = J.topRows(3);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jpos, Eigen::ComputeFullU | Eigen::ComputeThinV);
  auto s = svd.singularValues();

  double s1 = (s.size() > 0) ? s(0) : 0.0;
  double s2 = (s.size() > 1) ? s(1) : 0.0;
  double s3 = (s.size() > 2) ? s(2) : 0.0;
  double w  = s1 * s2 * s3;

  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Vector3d u_long = U.col(0); // 位置空間の長軸方向（world）

  // -----------------------------
  // 3) alpha, rho を計算（ここで初めて意味が通る）
  // -----------------------------
  const double alpha = std::abs(u_long.dot(g_hat_star));          // 0..1
  const double rho   = (dk > 1e-9) ? (rk / dk) : 1e9;             // r/d

  // 楕円体ログ（任意）
  static std::string g_ell_csv = "/tmp/manip_ellipsoid.csv";
  appendEllipsoidCsv(g_ell_csv, condition_id, trial, stage, (int)k, t,
                     s1, s2, s3, w, U.col(0), U.col(1), U.col(2));

  // -----------------------------
  // 4) まとめログ更新（min と time-avg）
  // -----------------------------
  out.m_min = std::min(out.m_min, mk);
  out.d_min = std::min(out.d_min, dk);

  num += mk * dt;
  den += dt;


  Eigen::Vector3d tcp_pos = st.getGlobalLinkTransform(jac_link).translation();
  if (marker_pub) {
  publishEllipsoidAxes(marker_pub,
                       tcp_pos,
                       U.col(0), U.col(1), U.col(2),
                       s1, s2, s3,
                       p_star, g_hat_star,
                       "world");
}
  // -----------------------------
  // 5) 時系列CSV（引数を全部渡す）
  // -----------------------------
  appendCmTimeSeriesCsv(ts_csv_path, condition_id, trial, stage,
                        (int)k, t,
                        mk, dk, rk,
                        rho, alpha,
                        g_hat_star, u_long,
                        link_star);
}

  out.T = rclcpp::Duration(jt.points.back().time_from_start).seconds();
  out.n_points = (int)jt.points.size();
  out.M = (den > 0.0) ? (num / den) : 0.0;
  return out;
}

// ===================== Collision Margin (CM) 追加(終) =====================
 
 
 // シーンにテーブルなどの衝突オブジェクトを追加
 void addCollisionObjects(PlanningSceneInterface &scene) {
   g_obstacles.clear();
   std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
   collision_objects.resize(4);
 
  //  // 1つ目: 大きいテーブル
  //  collision_objects[0].id = "table1";
  //  collision_objects[0].header.frame_id = "link0_1";
  //  shape_msgs::msg::SolidPrimitive table1_primitive;
  //  table1_primitive.type = table1_primitive.BOX;
  //  table1_primitive.dimensions = {2.0, 0.9, 0.01};
  //  geometry_msgs::msg::Pose table1_pose;
  //  table1_pose.orientation.w = 1.0;
  //  table1_pose.position.x = 0.5;
  //  table1_pose.position.y = 0.0;
  //  table1_pose.position.z = -BASE_HEIGHT - 0.005;
  //  collision_objects[0].primitives.push_back(table1_primitive);
  //  collision_objects[0].primitive_poses.push_back(table1_pose);
  //  collision_objects[0].operation = collision_objects[0].ADD;
 
  // 2つ目: 物体を置く小テーブル1
  collision_objects[1].id = "table_board1";
  collision_objects[1].header.frame_id = "world";

  shape_msgs::msg::SolidPrimitive table1_prim;
  table1_prim.type = table1_prim.BOX;
  table1_prim.dimensions.resize(3);
  table1_prim.dimensions[0] = 0.4;   // x
  table1_prim.dimensions[1] = 0.4;   // y
  table1_prim.dimensions[2] = 0.20;  // z (厚み)

  geometry_msgs::msg::Pose table1_pose;
  table1_pose.orientation.w = 1.0;

  // 台の中心位置(AMIRの正面)
  const double table1_center_x = 0.00;
  const double table1_center_y = 0.50;
  const double table1_top_z    = 0.20;
  const double table1_center_z = table1_top_z - table1_prim.dimensions[2] / 2.0;

  table1_pose.position.x = table1_center_x;
  table1_pose.position.y = table1_center_y;
  table1_pose.position.z = table1_center_z + Z_DOWN;

  collision_objects[1].primitives.push_back(table1_prim);
  collision_objects[1].primitive_poses.push_back(table1_pose);
  collision_objects[1].operation = collision_objects[1].ADD;

  // 3つ目: 物体を置く小テーブル2
  collision_objects[2].id = "table_board2";
  collision_objects[2].header.frame_id = "world";

  shape_msgs::msg::SolidPrimitive table2_prim;
  table2_prim.type = table2_prim.BOX;
  table2_prim.dimensions.resize(3);
  table2_prim.dimensions[0] = 0.4;   // x
  table2_prim.dimensions[1] = 0.4;   // y
  table2_prim.dimensions[2] = 0.20;  // z (厚み)

  geometry_msgs::msg::Pose table2_pose;
  table2_pose.orientation.w = 1.0;

  // 台の中心位置(AMIRの右側)
  const double table2_center_x = 0.50;
  const double table2_center_y = 0.00;
  const double table2_top_z    = 0.20;
  const double table2_center_z = table2_top_z - table2_prim.dimensions[2] / 2.0;

  table2_pose.position.x = table2_center_x;
  table2_pose.position.y = table2_center_y;
  table2_pose.position.z = table2_center_z + Z_DOWN;

  collision_objects[2].primitives.push_back(table2_prim);
  collision_objects[2].primitive_poses.push_back(table2_pose);
  collision_objects[2].operation = collision_objects[2].ADD;

  // 4つ目: ペットボトル障害物（円柱）
  // 半径 = 0.035m, 高さ = 0.22m
  // 台上面(0.2m)の上に置く => 中心z = 0.2 + 0.22/2

  // ===== テーブルは追加 =====
  std::vector<moveit_msgs::msg::CollisionObject> objs;
  objs.push_back(collision_objects[1]);
  objs.push_back(collision_objects[2]);

  // ===== ここから複数ボトル =====

  // ボトル中心z（台上面 + h/2）※Z_DOWNもここで反映
  const double bottle_h = 0.22;
  const double bottle_r = 0.035;
  const double bottle_center_z = (table2_top_z + bottle_h / 2.0) + Z_DOWN;

  // (x,y) を world 座標で指定（zは上で自動）
  std::vector<XY> bottle_xy = {
    {table1_center_x + 0.00, table1_center_y - 0.10},
    {table2_center_x - 0.10, table2_center_y - 0.10},
    {table2_center_x - 0.10, table2_center_y + 0.10},
  };

  // 形状は共通で使い回し
  shape_msgs::msg::SolidPrimitive bottle_prim;
  bottle_prim.type = bottle_prim.CYLINDER;
  bottle_prim.dimensions.resize(2);
  bottle_prim.dimensions[0] = bottle_h; // height
  bottle_prim.dimensions[1] = bottle_r; // radius

  for (size_t i = 0; i < bottle_xy.size(); ++i) {
    moveit_msgs::msg::CollisionObject bottle;
    bottle.id = "bottle_obstacle_" + std::to_string(i + 1);
    bottle.header.frame_id = "world";
    bottle.operation = bottle.ADD;

    geometry_msgs::msg::Pose bottle_pose;
    bottle_pose.orientation.w = 1.0;
    bottle_pose.position.x = bottle_xy[i].x;
    bottle_pose.position.y = bottle_xy[i].y;
    bottle_pose.position.z = bottle_center_z;

    bottle.primitives.push_back(bottle_prim);
    bottle.primitive_poses.push_back(bottle_pose);

    // DF用に中心を保存（“避ける障害物”だけ入れる）
    // ※ attachObjectで bottle_obstacle_1 を掴むなら、それは除外した方が自然
  if (i != 0) {
    Eigen::Vector3d c(bottle_pose.position.x, bottle_pose.position.y, bottle_pose.position.z);

  // 中心
  g_obstacles.push_back(Obstacle{c});
  // 上
  g_obstacles.push_back(Obstacle{c + Eigen::Vector3d(0,0,+bottle_h/2.0)});
  // 下
  g_obstacles.push_back(Obstacle{c + Eigen::Vector3d(0,0,-bottle_h/2.0)});
  }

    objs.push_back(bottle);
  }

  // ===== シーンに適用 =====
  scene.applyCollisionObjects(objs);
 }
 
 // 物体をアタッチ
//  void attachObject(MoveGroupInterface &group) {
//    group.attachObject("bottle_obstacle_1", "gripper_base_1");
//    rclcpp::sleep_for(1s);
//  }
 
 
 void attachObject(MoveGroupInterface& arm, MoveGroupInterface& gripper)
{
  std::vector<std::string> touch_links = gripper.getLinkNames();  // gripperグループ内リンク全部
  // attach先リンク名は tcp_link でも hand link でもOK（実在するリンク名にする）
  arm.attachObject("bottle_obstacle_1", "gripper_base_1", touch_links);
}

 // 物体をデタッチ
 void detachObject(MoveGroupInterface &group) {
   group.detachObject("bottle_obstacle_1");
   rclcpp::sleep_for(1s);
 }
 
 // グリッパーを開閉
 void controlGripper(MoveGroupInterface &group, double angle_deg) {
   std::vector<double> joint_positions(6, deg2rad(angle_deg));
   group.setJointValueTarget(joint_positions);
   group.setGoalTolerance(0.1);
   auto result = group.move();
   if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
     RCLCPP_WARN(rclcpp::get_logger("pick_place"), "Gripper control failed");
   }
 }
 
 // 指定姿勢へ移動
 void moveTo(MoveGroupInterface &group,
             double x, double y, double z,
             double roll_deg, double pitch_deg, double yaw_deg,
             double speed_scaling) {
   tf2::Quaternion q;
   q.setRPY(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));
   geometry_msgs::msg::Pose target;
   target.orientation = tf2::toMsg(q);
   target.position.x = x;
   target.position.y = y;
   target.position.z = z;
   group.setStartStateToCurrentState();
   group.setMaxVelocityScalingFactor(speed_scaling);
   group.setPoseTarget(target);
   group.setGoalTolerance(0.01);

   // plan と execute を分離して原因を分析する
  MoveGroupInterface::Plan plan;
  auto code = group.plan(plan);
  RCLCPP_WARN(rclcpp::get_logger("pick_place"),
              "plan() code=%d (target x=%.3f y=%.3f z=%.3f rpy=%.1f %.1f %.1f)",
              code.val, x, y, z, roll_deg, pitch_deg, yaw_deg);

  if (code != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger("pick_place"), "Motion planning failed (no execute).");
    group.clearPoseTargets();
    return;
  }

  auto ex = group.execute(plan);
  RCLCPP_WARN(rclcpp::get_logger("pick_place"), "execute() code=%d", ex.val);

  if (ex != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger("pick_place"), "Motion execution failed.");
  }

  group.clearPoseTargets();  

 }

 //一番Collison余裕が高い軌道を選択する
static inline double scoreCM(const CMLogRow& cm){
  // 基本は M（時間平均）を最大化
  return cm.M;
}

static inline bool betterCM(const CMLogRow& a, const CMLogRow& b){
  // aがbより良いならtrue
  const double sa = scoreCM(a);
  const double sb = scoreCM(b);
  if (sa != sb) return sa > sb;

  // tie-break 1: m_min（最悪点がマシ）
  if (a.m_min != b.m_min) return a.m_min > b.m_min;

  // tie-break 2: d_min（最小クリアランスが大きい）
  if (a.d_min != b.d_min) return a.d_min > b.d_min;

  // tie-break 3: 短い軌道（点数が同等ならシンプルな方）
  return a.n_points < b.n_points;
}

 // ここで距離、侵入角度、特異値を評価
 void moveToLogged(rclcpp::Node::SharedPtr node,
                  MoveGroupInterface &group,
                  int trial_id,
                  const std::string& stage,
                  double x, double y, double z,
                  double roll_deg, double pitch_deg, double yaw_deg,
                  double speed_scaling,
                  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub)
{
  tf2::Quaternion q;
  q.setRPY(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));
  geometry_msgs::msg::Pose target;
  target.orientation = tf2::toMsg(q);
  target.position.x = x;
  target.position.y = y;
  target.position.z = z;

  group.setMaxVelocityScalingFactor(speed_scaling);
  group.setPoseTarget(target);
  group.setGoalTolerance(0.01);

  const double bottle_h = 0.22;

  std::vector<RepPoint> rep_points = {
    {"outer_link_left_1",  Eigen::Vector3d(0,0,0), "outer_link_left_1"},
    {"outer_link_right_1", Eigen::Vector3d(0,0,0), "outer_link_right_1"},
    {"finger_right_1",     Eigen::Vector3d(0,0,0), "finger_right_1"},
    {"finger_left_1",      Eigen::Vector3d(0,0,0), "finger_left_1"},
    {"tcp_link", Eigen::Vector3d(0,0,+bottle_h/2.0), "grasp_obj_top"},
    {"tcp_link", Eigen::Vector3d(0,0,-bottle_h/2.0), "grasp_obj_bottom"},
  };

  const std::string jac_link = "tcp_link";

  // plan（候補を3本作って、CMでベストを選ぶ）
  const int K = 3;

  struct Cand {
    MoveGroupInterface::Plan plan;
    CMLogRow cm;
    DFLogRow df;
    double plan_time_s = 0.0;
    int ok = 0;
  };

  std::vector<Cand> cands;
  cands.reserve(K);

  // 重要：RRTConnect固定
  group.setPlanningPipelineId("ompl");
  group.setPlannerId("RRTConnectkConfigDefault");

  for (int i = 0; i < K; ++i) {
    Cand c;

    // 毎回スタート状態を更新（これが無いと同じになりやすい）
    group.setStartStateToCurrentState();

    MoveGroupInterface::Plan plan_i;

    auto t0i = std::chrono::steady_clock::now();
    auto code_i = group.plan(plan_i);
    auto t1i = std::chrono::steady_clock::now();
    c.plan_time_s = std::chrono::duration<double>(t1i - t0i).count();

    if (code_i != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(node->get_logger(),
                  "[%s][trial=%d] cand%d plan FAIL (code=%d)",
                  stage.c_str(), trial_id, i, code_i.val);
      c.ok = 0;
      cands.push_back(std::move(c));
      continue;
    }

    c.ok = 1;
    c.plan = plan_i;

    // ---- CM解析（候補ごとに別の時系列CSVへ）----
    std::string ts_path =
      "/tmp/collision_margin_timeseries_" + stage + "_trial" + std::to_string(trial_id)
      + "_cand" + std::to_string(i) + ".csv";

    CMLogRow cm_i;
    cm_i.plan_success = 1;
    cm_i.plan_time_s  = c.plan_time_s;

    cm_i = analyzeCollisionMarginTrajectory(
            group,
            plan_i.trajectory_.joint_trajectory,
            g_obstacles,
            g_bottle_r,
            rep_points,
            jac_link,
            ts_path,           // ★候補別
            g_condition_id,
            trial_id,
            stage,
            nullptr            // ★候補選定中はRViz出さない（ゴチャるので）
          );

    cm_i.plan_success = 1;
    cm_i.plan_time_s  = c.plan_time_s;
    c.cm = cm_i;

    RCLCPP_WARN(node->get_logger(),
                "[%s][trial=%d] cand%d OK: M=%.4f m_min=%.4f d_min=%.4f T=%.3f n=%d plan_time=%.3f",
                stage.c_str(), trial_id, i,
                c.cm.M, c.cm.m_min, c.cm.d_min, c.cm.T, c.cm.n_points, c.plan_time_s);

    cands.push_back(std::move(c));
  }

  // ---- best選択 ----
  int best = -1;
  for (int i = 0; i < (int)cands.size(); ++i) {
    if (!cands[i].ok) continue;
    if (best < 0 || betterCM(cands[i].cm, cands[best].cm)) best = i;
  }

  DFLogRow row;
  row.plan_success = (best >= 0) ? 1 : 0;

  if (best < 0) {
    RCLCPP_WARN(node->get_logger(),
                "[%s][trial=%d] all candidates failed planning.", stage.c_str(), trial_id);
    row.exec_success = -1;
    row.fail_mode = "plan_fail_all";
    appendCsv(g_csv_path, g_condition_id + "_" + stage, trial_id, row);
    group.clearPoseTargets();
    return;
  }

  RCLCPP_WARN(node->get_logger(),
              "[%s][trial=%d] SELECT best=cand%d  M=%.4f m_min=%.4f d_min=%.4f",
              stage.c_str(), trial_id, best,
              cands[best].cm.M, cands[best].cm.m_min, cands[best].cm.d_min);

  // ---- bestだけ、ここでRViz表示したいなら解析をもう一度(marker_pubありで) ----
  // （重いなら省略OK。見たいなら下を有効化）
  {
    // bestの時系列は既にCSV出てるので、ここはRViz用だけ再計算でもOK
    analyzeCollisionMarginTrajectory(
      group,
      cands[best].plan.trajectory_.joint_trajectory,
      g_obstacles,
      g_bottle_r,
      rep_points,
      jac_link,
      "/tmp/_tmp_best.csv",
      g_condition_id,
      trial_id,
      stage,
      marker_pub                 // ★bestだけ表示
    );
  }

  // ---- DF解析やdf_timeseries_planをbestだけでやる（今のコードはplan直後にやってたので移動）----
  static std::string g_csv_timeseries = "/tmp/df_timeseries_plan.csv";
  exportPlanTimeSeriesMultiLinks(group,
                                 cands[best].plan.trajectory_.joint_trajectory,
                                 g_obstacles, g_prm,
                                 rep_points,
                                 g_csv_timeseries,
                                 g_condition_id, trial_id, stage);

  row = analyzeTrajectoryMultiLinks(group,
                                   cands[best].plan.trajectory_.joint_trajectory,
                                   g_obstacles, g_prm,
                                   rep_points);
  row.plan_success = 1;

  // execute（bestのみ）
  auto ex = group.execute(cands[best].plan);

  RCLCPP_WARN(node->get_logger(),
              "[%s][trial=%d] execute code=%d (best=cand%d)",
              stage.c_str(), trial_id, ex.val, best);
  if (ex != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(node->get_logger(),
                "Motion execution failed (MoveItErrorCode != SUCCESS).");
  }

  // ユーザ入力
  std::cout << "\n[" << stage << "][trial=" << trial_id << "] Collision observed? [y/n] : ";
  char c; std::cin >> c;
  int exec_success = (c=='n' || c=='N') ? 1 : 0;

  row.exec_success = exec_success;
  std::cout << "[" << stage << "][trial=" << trial_id
            << "] fail_mode [ok/collision_observed/near_miss/other] : ";
  std::string mode; std::cin >> mode;
  row.fail_mode = mode;

  appendCsv(g_csv_path, g_condition_id + "_" + stage, trial_id, row);

  // ★CM（bestの結果）を1行出力
  CMLogRow cm_best = cands[best].cm;
  cm_best.exec_success = exec_success;
  appendCmTrajCsv(g_cm_traj_csv, g_condition_id, trial_id, stage, cm_best);

  group.clearPoseTargets();
  return;
}
// 方向ベクトルを表示するために追加
static visualization_msgs::msg::Marker makeArrow(
    const std::string& frame_id,
    const std::string& ns,
    int id,
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& dir_unit,
    double length,
    double scale_shaft,
    double scale_head,
    double r, double g, double b, double a)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = rclcpp::Time(0);
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point p_start, p_end;
  p_start.x = p0.x(); p_start.y = p0.y(); p_start.z = p0.z();
  Eigen::Vector3d p1 = p0 + length * dir_unit;
  p_end.x = p1.x(); p_end.y = p1.y(); p_end.z = p1.z();

  m.points.push_back(p_start);
  m.points.push_back(p_end);

  // scale: x=shaft diameter, y=head diameter, z=head length（ARROWの場合）
  m.scale.x = scale_shaft;
  m.scale.y = scale_head;
  m.scale.z = scale_head * 2.0;

  m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
  m.lifetime = rclcpp::Duration(0, 0); // 0なら更新まで保持
  return m;
}

static void publishEllipsoidAxes(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
    const Eigen::Vector3d& tcp_pos,
    const Eigen::Vector3d& u1, const Eigen::Vector3d& u2, const Eigen::Vector3d& u3,
    double s1, double s2, double s3,
    const Eigen::Vector3d& p_star,
    const Eigen::Vector3d& g_hat,
    const std::string& frame_id = "world")
{
  visualization_msgs::msg::MarkerArray arr;

  // 主軸3本（長さをσに比例させる：見やすい倍率kを入れてOK）
  const double k = 0.20; // ★見やすさ調整（0.1〜0.5くらいで）
  arr.markers.push_back(makeArrow(frame_id, "ellipsoid", 1, tcp_pos, u1.normalized(), k*s1, 0.01, 0.02, 1,0,0,0.9));
  arr.markers.push_back(makeArrow(frame_id, "ellipsoid", 2, tcp_pos, u2.normalized(), k*s2, 0.01, 0.02, 0,1,0,0.9));
  arr.markers.push_back(makeArrow(frame_id, "ellipsoid", 3, tcp_pos, u3.normalized(), k*s3, 0.01, 0.02, 0,0,1,0.9));

  // 障害物方向（固定長）
  if (g_hat.norm() > 1e-9) {
    arr.markers.push_back(makeArrow(frame_id, "danger", 10, p_star, g_hat.normalized(), 0.20, 0.015, 0.03, 1,1,0,0.9));
  }

  pub->publish(arr);
}
 
 int main(int argc, char **argv) {
   rclcpp::init(argc, argv);
   auto node = rclcpp::Node::make_shared("pick_place");

   // Gazebo シミュレーション時刻を使用
   node->set_parameter(rclcpp::Parameter("use_sim_time", true));

   // MoveIt のアクションコールバックを処理するためにバックグラウンドでスピン
   rclcpp::executors::MultiThreadedExecutor executor;
   executor.add_node(node);
   std::thread spinner([&executor](){ executor.spin(); });

   MoveGroupInterface arm(node, "arm");
  //　障害物回避の手法をRRTConnectに固定
   arm.setPlanningPipelineId("ompl");
   arm.setPlannerId("RRTConnectkConfigDefault");
   MoveGroupInterface gripper(node, "gripper");
   PlanningSceneInterface planning_scene_interface;

   auto marker_pub =
  node->create_publisher<visualization_msgs::msg::MarkerArray>("/manip_markers", 10);
 
   arm.setPlanningTime(30.0);
   gripper.setPlanningTime(10.0);
 
   rclcpp::sleep_for(1s);

  //  障害物を追加
  addCollisionObjects(planning_scene_interface);

   controlGripper(gripper, -60.0);
 
   // pick 操作
   RCLCPP_INFO(node->get_logger(), "Move above object");
   moveTo(arm, 0.0, 0.4, 0.2, 0.0, 0.0, 0.0, 0.3);
  //  moveTo(arm, OBJECT_POSITION[0], OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.08, 0.0, 0.0, 0.0, 0.3);
  //  controlGripper(gripper, -60.0);
  //  RCLCPP_INFO(node->get_logger(), "Approach object slowly");
  //  moveTo(arm, OBJECT_POSITION[0], OBJECT_POSITION[1], OBJECT_DIMENSION[2] - 0.05, 0.0, 0.0, -90.0, 0.3);
   rclcpp::sleep_for(1s);   

   // ピック対象のボトルをシーンから除去してから把持物としてアタッチ
   planning_scene_interface.removeCollisionObjects({"bottle_obstacle_1"});
   rclcpp::sleep_for(500ms);
   attachObject(arm, gripper);
   rclcpp::sleep_for(1s);
   controlGripper(gripper, -20.0);
   rclcpp::sleep_for(1s); 
   RCLCPP_INFO(node->get_logger(), "Lift object");
  //  moveTo(arm, OBJECT_POSITION[0], OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.1, 0.0, 0.0, 0.0, 0.3);
  
   // 中間姿勢
   moveTo(arm, 0.0, 0.4, 0.4, 0.0, 0.0, 0.0, 0.3);
  
  //  // place 操作
   RCLCPP_INFO(node->get_logger(), "Move to place location");
  //  moveTo(arm, OBJECT_POSITION[0]+0.10, OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.08, 0.0, 0.0, -90.0, 0.3);

   moveToLogged(node, arm, 0, "place",OBJECT_POSITION[0]+0.10, OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.08, 0.0, 0.0, -90.0, 0.3,marker_pub);
  
   controlGripper(gripper, -60.0);
   detachObject(arm);

   RCLCPP_INFO(node->get_logger(), "Retreat");
   moveTo(arm, OBJECT_POSITION[0]+0.10, OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.1, 0.0, 0.0, -90.0, 0.3);

  //  RCLCPP_INFO(node->get_logger(), "Retreat");
  //  moveTo(arm, OBJECT_POSITION[0], OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.4, 0.0, 0.0, -90.0, 0.3);
  
  // // controlGripper(gripper, -60.0);
  // // detachObject(arm);
  
   RCLCPP_INFO(node->get_logger(), "Return to home");
   moveTo(arm, 0.0, 0.3, 0.4, 0.0, 0.0, 0.0, 0.3);
    
  //  controlGripper(gripper, -20.0);
   RCLCPP_WARN(node->get_logger(), "arm planning frame = %s", arm.getPlanningFrame().c_str());
  //  RCLCPP_INFO(node->get_logger(), "Pick and place done");

   // デフォルト値付きでパラメータ宣言
    // node->declare_parameter<double>("move_to.x",      rclcpp::PARAMETER_DOUBLE);
    // node->declare_parameter<double>("move_to.y",      rclcpp::PARAMETER_DOUBLE);
    // node->declare_parameter<double>("move_to.z",      rclcpp::PARAMETER_DOUBLE);
    // node->declare_parameter<double>("move_to.roll",  rclcpp::PARAMETER_DOUBLE);
    // node->declare_parameter<double>("move_to.pitch",   rclcpp::PARAMETER_DOUBLE);
    // node->declare_parameter<double>("move_to.yaw",   rclcpp::PARAMETER_DOUBLE);
    // node->declare_parameter<double>("move_to.velocity",rclcpp::PARAMETER_DOUBLE);

    // // パラメータ取得
    // double x, y, z, roll, pitch, yaw, vel;
    // node->get_parameter("move_to.x",      x);
    // node->get_parameter("move_to.y",      y);
    // node->get_parameter("move_to.z",      z);
    // node->get_parameter("move_to.roll",  roll);
    // node->get_parameter("move_to.pitch", pitch);
    // node->get_parameter("move_to.yaw",   yaw);
    // node->get_parameter("move_to.velocity", vel);

    // moveTo(arm, x, y, z, roll, pitch, yaw, vel);



   executor.cancel();
   spinner.join();
   rclcpp::shutdown();
   return 0;
 }
 