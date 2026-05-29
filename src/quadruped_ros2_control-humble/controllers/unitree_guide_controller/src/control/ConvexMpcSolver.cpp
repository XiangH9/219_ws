#include "unitree_guide_controller/control/ConvexMpcSolver.h"

#include <unitree_guide_controller/common/mathTools.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

extern "C" {
#include <hpipm_common.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_d_ocp_qp_ipm.h>
}

#include <rclcpp/rclcpp.hpp>

namespace {

constexpr bool kEnableMpcStageLog = false;  // HPIPM内部阶段求解调试日志开关，建议false
double kDummyScalar = 0.0;

rclcpp::Logger& mpcLogger() {
  static auto logger = rclcpp::get_logger("ConvexMpcSolver");
  return logger;
}

rclcpp::Clock& mpcClock() {
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

void logFallbackReason(const char* reason) {
  RCLCPP_WARN_THROTTLE(
      mpcLogger(),
      mpcClock(),
      500,
      "[ConvexMpcSolver] fallback reason: %s",
      reason);
}

}  // namespace

// 限制到 [0,1]
// clamp = 夹紧/限幅。
// clamp01(x) 表示把 x 限制到 0 到 1 之间。
static inline double clamp01(const double x) {
  return std::max(0.0, std::min(1.0, x));
}

// 相位 wrap 到 [0,1)
// wrap = 环绕。
// 例如 1.2 会变成 0.2，-0.1 会变成 0.9。
// 用于处理步态周期相位，因为相位超过 1 后应回到 0。
static inline double wrap01(const double x) {
  double y = std::fmod(x, 1.0);
  if (y < 0.0) {
    y += 1.0;
  }
  return y;
}

/*
 * WaveGenerator 的 phase_ 是“当前支撑相/摆动相内部进度”，不是完整周期 phase。
 *
 * contact = 1:
 *   phase = normal_t / stanceRatio
 *   normal_t = phase * stanceRatio
 *
 * contact = 0:
 *   phase = (normal_t - stanceRatio) / (1 - stanceRatio)
 *   normal_t = stanceRatio + phase * (1 - stanceRatio)
 *
 * normal_t 才是完整步态周期内的归一化相位 [0,1)。
 * 
 * 这个函数的作用就是：
 * 把 WaveGenerator 给出的“支撑相/摆动相内部进度”转换成完整步态周期里的统一相位 normal_t ∈ [0,1)，
 * 方便后面预测 horizon 内每条腿未来是支撑还是摆动。
 */
static inline double legModePhaseToCyclePhase(
    const double mode_phase,      // mode_phase 是 WaveGenerator 的 phase_，表示当前支撑相/摆动相内部进度
    const int contact,            // contact 是当前接触状态，1 支撑，0 摆动
    const double stance_ratio) {  // stance_ratio 是一个步态周期内支撑相的占比，例如 0.6 表示支撑相占 60%，摆动相占 40%
  const double ph = clamp01(mode_phase);
  const double st = clamp01(stance_ratio);

  // 一个完整周期里先支撑、后摆动
  if (contact == 1) {
    return ph * st;
  }

  return st + ph * (1.0 - st);
}

/*
 * 本函数用于计算本次 MPC 实际应该用多少个预测步 N
 * 根据“半个步态周期”限制，计算实际 HPIPM horizon steps。
 *
 * 参考论文中使用固定接触点近似。
 *
 * 这要求预测时域不能超过半个步态周期，否则同一条腿可能在预测域内二次落地，
 * 第二次落地的足端点就不能继续用当前记录的固定接触点近似。
 */
static inline int computeEffectiveHorizon(
    const int max_N,
    const double dt,
    const double gait_period,
    const bool enforce_half_gait_horizon) {
  if (max_N <= 0 || dt <= 0.0 || !std::isfinite(dt)) {  // 安全限制
    return 0;
  }

  if (!enforce_half_gait_horizon) { // 如果不用半步态周期限制，也就是不用论文方法，直接返回 max_N
    return max_N;
  }

  if (gait_period <= 0.0 || !std::isfinite(gait_period)) {
    return 0;
  }

  const int N = static_cast<int>(std::floor(0.5 * gait_period / dt)); // 计算半个步态周期内能有多少个控制周期，也就是预测步数 N

  if (N < 1) {
    return 0;
  }

  return std::min(max_N, N);
}

/*
 * HPIPM 的 C API 需要手动分配内存：
 * 1. memsize
 * 2. malloc
 * 3. create
 *
 * MemoryBlock 负责 RAII 管理 malloc/free。
 */
class MemoryBlock {
public:
  explicit MemoryBlock(size_t size = 0) {
    reserve(size);
  }

  ~MemoryBlock() noexcept {
    std::free(ptr_);
  }

  void reserve(size_t size) {
    if (size > size_) {
      std::free(ptr_);
      ptr_ = std::malloc(size);
      if (ptr_ == nullptr) {
        throw std::bad_alloc();
      }
      size_ = size;
    }
  }

  void* get() {
    return ptr_;
  }

  MemoryBlock(const MemoryBlock&) = delete;
  MemoryBlock& operator=(const MemoryBlock&) = delete;

private:
  void* ptr_ = nullptr;
  size_t size_ = 0;
};

struct ConvexMpcSolver::HpipmWorkspace {
  // 这是缓存当前 HPIPM 的问题规模。
  // 如果本次 MPC 的规模和上次一样，就不重新分配 HPIPM 内存，只更新求解器参数
  int cached_N = -1;
  int cached_nx = -1;
  int cached_nu = -1;
  int cached_ng = -1;
  bool initialized = false;

  std::vector<int> nx;
  std::vector<int> nu;
  std::vector<int> nbx;
  std::vector<int> nbu;
  std::vector<int> ng;
  std::vector<int> nsbx;
  std::vector<int> nsbu;
  std::vector<int> nsg;

  std::vector<Mat13> A_stage;
  std::vector<Mat1312> B_stage;
  std::vector<Vec13> bb_stage;
  std::vector<VecX> q_stage;
  std::vector<VecX> lg_stage;
  std::vector<VecX> ug_stage;

  std::vector<double*> AA;
  std::vector<double*> BB;
  std::vector<double*> bb;

  std::vector<double*> QQ;
  std::vector<double*> RR;
  std::vector<double*> SS;
  std::vector<double*> qq;
  std::vector<double*> rr;

  std::vector<double*> CC;
  std::vector<double*> DD;
  std::vector<double*> llg;
  std::vector<double*> uug;

  MemoryBlock dimMem;
  d_ocp_qp_dim dim;

  MemoryBlock qpMem;
  d_ocp_qp qp;

  MemoryBlock qpSolMem;
  d_ocp_qp_sol qpSol;

  MemoryBlock ipmArgMem;
  d_ocp_qp_ipm_arg arg;

  MemoryBlock ipmMem;
  d_ocp_qp_ipm_ws workspace;

  void resizeIfNeeded(int N, int nx_stage, int nu_stage, int ng_stage) {
    if (initialized &&  // 缓存优化，如果数组大小不变，则不重新申请内存
        cached_N == N &&
        cached_nx == nx_stage &&
        cached_nu == nu_stage &&
        cached_ng == ng_stage) {
      applySettings();
      return;
    }

    cached_N = N;
    cached_nx = nx_stage;
    cached_nu = nu_stage;
    cached_ng = ng_stage;

    nx.assign(N + 1, nx_stage);
    nu.assign(N + 1, nu_stage);
    nbx.assign(N + 1, 0);
    nbu.assign(N + 1, 0);
    ng.assign(N + 1, ng_stage);
    nsbx.assign(N + 1, 0);
    nsbu.assign(N + 1, 0);
    nsg.assign(N + 1, 0);

    // stage 0 的 x0 是已知初始状态，不作为 HPIPM 决策变量。
    // 因此 nx[0] = 0。
    nx[0] = 0;

    // terminal stage 没有输入，也没有接触力约束。
    nu[N] = 0;
    ng[N] = 0;

    A_stage.assign(N, Mat13::Zero());
    B_stage.assign(N, Mat1312::Zero());
    bb_stage.assign(N, Vec13::Zero());
    q_stage.assign(N + 1, VecX::Zero(nx_stage));
    lg_stage.assign(N, VecX::Zero(ng_stage));
    ug_stage.assign(N, VecX::Zero(ng_stage));

    AA.assign(N, nullptr);
    BB.assign(N, nullptr);
    bb.assign(N, nullptr);

    QQ.assign(N + 1, nullptr);
    RR.assign(N + 1, nullptr);
    SS.assign(N + 1, nullptr);
    qq.assign(N + 1, nullptr);
    rr.assign(N + 1, nullptr);

    CC.assign(N + 1, nullptr);
    DD.assign(N + 1, nullptr);
    llg.assign(N + 1, nullptr);
    uug.assign(N + 1, nullptr);

    const int dim_size = d_ocp_qp_dim_memsize(N);
    dimMem.reserve(dim_size);
    d_ocp_qp_dim_create(N, &dim, dimMem.get());

    d_ocp_qp_dim_set_all(
        nx.data(),
        nu.data(),
        nbx.data(),
        nbu.data(),
        ng.data(),
        nsbx.data(),
        nsbu.data(),
        nsg.data(),
        &dim
    );

    const int qp_size = d_ocp_qp_memsize(&dim);
    qpMem.reserve(qp_size);
    d_ocp_qp_create(&dim, &qp, qpMem.get());

    const int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
    qpSolMem.reserve(qp_sol_size);
    d_ocp_qp_sol_create(&dim, &qpSol, qpSolMem.get());

    const int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
    ipmArgMem.reserve(ipm_arg_size);
    d_ocp_qp_ipm_arg_create(&dim, &arg, ipmArgMem.get());

    applySettings();

    const int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
    ipmMem.reserve(ipm_size);
    d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipmMem.get());

    initialized = true;
  }

  void applySettings() {
    d_ocp_qp_ipm_arg_set_default(hpipm_mode::SPEED, &arg);

    int iter_max = 60;          // hpipm_iter_max = HPIPM 最大迭代次数
    double alpha_min = 1e-12;   // hpipm_alpha_min = HPIPM 线搜索最小步长
    double mu0 = 1e1;           // hpipm_mu0 = 初始 barrier parameter，内点法初始障碍参数

    double tol_stat = 1e-4;     // hpipm_tol_stat = stationarity tolerance，驻点残差容忍度
    double tol_eq = 1e-4;       // hpipm_tol_eq = equality constraint tolerance，等式约束残差容忍度
    double tol_ineq = 1e-4;     // hpipm_tol_ineq = inequality constraint tolerance，不等式约束残差容忍度
    double tol_comp = 1e-4;     // hpipm_tol_comp = complementarity tolerance，互补条件残差容忍度

    double reg_prim = 1e-10;    // hpipm_reg_prim = primal regularization，原始变量正则化，用于改善数值稳定性
    int warm_start = 1;         // hpipm_warm_start = 是否启用 HPIPM 内部热启动，0 表示不启用
    int pred_corr = 1;          // hpipm_pred_corr = predictor-corrector 开关，pred = predictor，预测步；corr = corrector，校正步，1 表示启用
    int ric_alg = 0;            // hpipm_ric_alg = Riccati algorithm，Riccati 递推算法类型，ric = Riccati，0 表示 square-root Riccati

    d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
    d_ocp_qp_ipm_arg_set_alpha_min(&alpha_min, &arg);
    d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
    d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
    d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
    d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
    d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
    d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &arg);
    d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &arg);
    d_ocp_qp_ipm_arg_set_pred_corr(&pred_corr, &arg);
    d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &arg);
  }
};

ConvexMpcSolver::ConvexMpcSolver()
  : nx_stage(13),     // 每个状态维度：Theta(3), p_com(3), omega(3), v_com(3), g_z(1)
    nu_stage(12),     // 每个输入维度：4 条腿的足底力，每条腿 3 维
    rows_per_leg(7),  // 每条腿的约束行数：摩擦锥 5 行 + 摆动腿强制 fx=0, fy=0 的 2 行
    ng_stage(28),     // 每步一般线性约束行数：4 条腿 * 每条腿 7 行约束
    INF(1e10),         // 有限大上界，避免约束尺度过大导致 HPIPM 数值病态
    hpipm_(new HpipmWorkspace()) {
  // 状态顺序：
  // x = [Theta(3), p_com(3), omega(3), v_com(3), g_z(1)]

  N = 25;                         // N = 最大预测步数，先缩短 horizon 提高收敛率

  // GO2 URDF-derived SRBD parameters.
  // total mass: sum of all link inertias in go2_description/urdf/robot.urdf
  mass = 15.098;
  // trunk inertia / COM from go2_description trunk inertial block
  Ib << 0.02448,    0.00012166,  0.0014849,
        0.00012166, 0.098077,   -0.0000312,
        0.0014849, -0.0000312,   0.107;
  pcb_B << 0.021112, 0.0, -0.005366;

  // GO1 / previous tuned values kept here for quick A/B rollback.
  // mass = 40.5;
  // Ib << 0.624,     0.000079993,  -0.000069927,
  //       0.000079993,  2.683,      -0.0000043683,
  //      -0.000069927, -0.0000043683,  2.934;
  // pcb_B << 0, 0, 0;
  g = Vec3(0.0, 0.0, -9.81);      // 重力加速度向量

  mu = 0.4;                       // 摩擦系数
  // Dynamic stance lower bound now scales with stance count.
  // This base value is kept as the minimum per-leg floor in 4-leg stance.
  fzMin = 25.0;
  // Previous fixed lower bound experiments kept for quick rollback:
  // fzMin = 40.0;
  // fzMin = 70.0;
  fzMax = 450.0;

  enforceHalfGaitHorizon = true;  // 论文：预测时域不能超过半个步态周期

  // regularization = 数值正则化系数。
  // 加到 QP Hessian 对角线上，避免矩阵病态或半正定导致求解器不稳定。
  regularization = 1e-8;
  enableFallbackToLast = true;    // enableFallbackToLast = 求解失败时是否回退到上一帧成功求解的足底力。

  Q.setZero();
  R.setZero();
  S.setZero();
  Q.diagonal() << 90, 90, 12,   // 3轴角度，姿态的权重
                      24, 24, 180, // 3轴位置，机身位置的权重
                      1.0, 1.0, 0.8,  // 3轴角速度，姿态变化的权重
                      3,   3,   10,   // 3轴速度，机身速度的权重
                      0.0;            // 重力
  // Previous values for rollback:
  // Q.diagonal() << 120, 120, 15,
  //                     30,  30,  120,
  //                     1.5, 1.5, 1.0,
  //                     4,   4,   8,
  //                     0.0;

  R.diagonal() << 0.5, 0.5, 0.03,  // 力大小限制惩罚权重
                      0.5, 0.5, 0.03,
                      0.5, 0.5, 0.03,
                      0.5, 0.5, 0.03;
  // Previous values for rollback:
  // R.diagonal() << 0.3, 0.3, 0.05,
  //                     0.3, 0.3, 0.05,
  //                     0.3, 0.3, 0.05,
  //                     0.3, 0.3, 0.05;

  S.diagonal() << 0.003, 0.003, 0.003, // 力变化平滑限制惩罚权重
                      0.003, 0.003, 0.003,
                      0.003, 0.003, 0.003,
                      0.003, 0.003, 0.003;
  // Previous values for rollback:
  // S.diagonal() << 0.01, 0.01, 0.01,
  //                     0.01, 0.01, 0.01,
  //                     0.01, 0.01, 0.01,
  //                     0.01, 0.01, 0.01;

  rebuildFixedMatrices();
}

ConvexMpcSolver::~ConvexMpcSolver() = default;

void ConvexMpcSolver::rebuildFixedMatrices() {

  /*
  R / S 是你定义的权重。
  R_cost_ / R_cost_rate_ 是根据权重预先算好的矩阵。
  hpipm_->RR[k] / hpipm_->rr[k] 是最后传给 HPIPM 的接口。
  */
  Q_cost_ = MatX::Zero(nx_stage, nx_stage);
  Q_cost_.noalias() = 2.0 * Q;
  Q_cost_.diagonal().array() += regularization;

  R_cost_ = MatX::Zero(nu_stage, nu_stage);
  R_cost_.noalias() = 2.0 * R;
  R_cost_.diagonal().array() += regularization;

  // HPIPM 的 stage cost 里包含输入-状态交叉项 S。
  // 当前论文实现没有这部分，因此这里显式传一个全零矩阵，
  // 避免 d_ocp_qp_set_all 在 nx>0 && nu>0 的 stage 读到空指针。
  S_cross_cost_ = MatX::Zero(nu_stage, nx_stage);

  // R：惩罚足底力本身太大
  // S：惩罚当前足底力相对上一帧变化太大
  R_cost_rate_ = MatX::Zero(nu_stage, nu_stage);
  R_cost_rate_.noalias() = 2.0 * (R + S); // 这里是计算之后的中间过程项，为了符合HPIPM的格式生态
  R_cost_rate_.diagonal().array() += regularization;

  r_cost_zero_ = VecX::Zero(nu_stage);
  r_cost_rate_ = VecX::Zero(nu_stage);

  // 单条腿的固定输入约束矩阵。
  // 这个矩阵只依赖 mu，不依赖 k，也不依赖 contact。
  //
  // 前 5 行严格对应论文式(7)：
  // [ -1  0  mu ]
  // [  0 -1  mu ]
  // [  1  0  mu ]
  // [  0  1  mu ]
  // [  0  0   1 ]
  //
  // 后 2 行只用于摆动腿时强制 fx=0, fy=0。
  MatX leg_force_constraint = MatX::Zero(rows_per_leg, 3);
  leg_force_constraint <<
      -1.0,  0.0, mu,
       0.0, -1.0, mu,
       1.0,  0.0, mu,
       0.0,  1.0, mu,
       0.0,  0.0,          1.0,
       1.0,  0.0,          0.0,
       0.0,  1.0,          0.0;

  // 输入约束矩阵，论文中的摩擦锥约束实际填在这里。
  D_force_constraint_ = MatX::Zero(ng_stage, nu_stage);
  for (int leg = 0; leg < 4; ++leg) {
    const int row = rows_per_leg * leg;
    const int col = 3 * leg;
    D_force_constraint_.block(row, col, rows_per_leg, 3) = leg_force_constraint;
  }

  // 状态约束矩阵，论文中没有状态约束，所以 C_stage 全零。
  // 对所有 k>=1 相同，所以只构造一次。
  C_zero_constraint_ = MatX::Zero(ng_stage, nx_stage);


  // 以下为约束的上下界模板
  // 对应的是论文中摩擦锥约束公式两端的两个ci上下界
  // 在支撑足情况下，前5项都与论文公式一致
  // 6和7项在支撑足时不约束，在摆动足时强制 fx=0, fy=0
  // 摆动足限幅约束全部取消，而是所有力全部强制置0，因为这里限幅的是地面对足端的接触反作用力，所以摆动时设置为0是正确的


  // 支撑腿上下界模板。
  lower_bound_stance_ = VecX::Zero(rows_per_leg);
  upper_bound_stance_ = VecX::Zero(rows_per_leg);

  lower_bound_stance_ <<
       0.0,
       0.0,
       0.0,
       0.0,
       fzMin,
      -INF,
      -INF;

  upper_bound_stance_ <<
       INF,
       INF,
       INF,
       INF,
       fzMax,
       INF,
       INF;

  // 摆动腿上下界模板。
  lower_bound_swing_ = VecX::Zero(rows_per_leg);
  upper_bound_swing_ = VecX::Zero(rows_per_leg);

  lower_bound_swing_ <<
      -INF,
      -INF,
      -INF,
      -INF,
       0.0,
       0.0,
       0.0;

  upper_bound_swing_ <<
       INF,
       INF,
       INF,
       INF,
       0.0,
       0.0,
       0.0;
}


void ConvexMpcSolver::reset() {
  last_u0_.setZero();
  has_last_solution_ = false;
  has_logged_first_trot_mpc_snapshot_ = false;
  has_logged_first_qp_structure_snapshot_ = false;
}

double ConvexMpcSolver::computeStanceLegFzMin(int stance_count) const {
  const int effective_stance_count = std::max(1, stance_count);
  const double nominal_per_leg =
      mass * std::abs(g(2)) / static_cast<double>(effective_stance_count);

  // Only use this in the constraint, not in the objective:
  // keep enough normal force margin for the current support pattern.
  const double scaled_min = 1.0 * nominal_per_leg;
  return std::min(fzMax, std::max(fzMin, scaled_min));
}

// MPC 求解失败/输入非法时的保底力分配函数
Vec34 ConvexMpcSolver::makeFallbackForces(const VecInt4& contact_now) const {
  Vec34 force;
  force.setZero();

  int stance_count = 0; // 统计当前有几条腿是支撑腿
  for (int leg = 0; leg < 4; ++leg) {
    if (contact_now(leg) == 1) {
      ++stance_count;
    }
  }

  const bool no_stance = (stance_count == 0);
  const int effective_stance_count = no_stance ? 4 : stance_count;
  const double fz = computeStanceLegFzMin(effective_stance_count);

  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("ConvexMpcSolver"),
      steady_clock,
      350,
      "[ConvexMpcSolver] 警告：MPC 求解失败或输入非法，当前使用保底力。"
      "contact=[%d %d %d %d]，支撑腿数量=%d，保底 fz=%.3f",
      contact_now(0),
      contact_now(1),
      contact_now(2),
      contact_now(3),
      stance_count,
      fz
  );

  for (int leg = 0; leg < 4; ++leg) {
    if (no_stance || contact_now(leg) == 1) {
      force(2, leg) = fz;
    }
  }

  return force;
}

ConvexMpcOutput ConvexMpcSolver::solveMpc(const ConvexMpcInput& in) {
  ConvexMpcOutput out;

  const int horizon_N = in.N;

  if (horizon_N <= 0 ||
      static_cast<int>(in.xRef.size()) < horizon_N + 1 ||
      static_cast<int>(in.contact.size()) < horizon_N ||
      static_cast<int>(in.rFeet.size()) < horizon_N ||
      in.dt <= 0.0 ||
      !std::isfinite(in.dt) ||
      !in.x0.allFinite() ||
      !in.Iw_inv.allFinite()) {
    return out;
  }

  
  /*
    N          // MPC 预测步数
    nx_stage   // 每个状态维度，13，包括 Theta(3), p_com(3), omega(3), v_com(3), g_z(1)
    nu_stage   // 每个输入维度，12，包括 4 条腿的足底力，每条腿 3 维
    ng_stage   // 每步一般线性约束行数，4*7=28，包括 4 条腿，每条腿 7 行约束（摩擦锥 5 行 + 摆动腿强制 fx=0, fy=0 的 2 行）
  */
  // HPIPM 的求解器内存准备函数，设置基础参数并分配内存。
  if (kEnableMpcStageLog) {
    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][stage] solveMpc enter: N=%d dt=%.6f nx=%d nu=%d ng=%d",
        horizon_N,
        in.dt,
        nx_stage,
        nu_stage,
        ng_stage);
  }

  if (kEnableMpcStageLog) {
    RCLCPP_INFO(mpcLogger(), "[ConvexMpcSolver][stage] before resizeIfNeeded");
  }
  hpipm_->resizeIfNeeded(horizon_N, nx_stage, nu_stage, ng_stage);
  if (kEnableMpcStageLog) {
    RCLCPP_INFO(mpcLogger(), "[ConvexMpcSolver][stage] after resizeIfNeeded");
  }

  // ====== 线性离散 SRBD 模型 ======
  //
  // 论文状态：
  // x = [Theta, p_com, omega, v_com, g_z]
  // 离散形式：
  // x(k+1) = A_k x(k) + B_k u(k)
  //
  // 重力通过状态最后一维 g_z 进入速度更新：
  // v(k+1) = v(k) + dt * e_z * g_z + dt/m * sum(f_i)

  for (int k = 0; k < horizon_N; ++k) {
    hpipm_->A_stage[k].setIdentity(); //先补充为单位阵，再填入非单位部分
    hpipm_->B_stage[k].setZero();

    // yaw_d：第 k 个参考状态里的期望 yaw。
    // 论文公式 A_k 中使用 R_z^T(_d psi^k)。
    const double yaw_d = in.xRef[k](2); // xRef[k] 的第 3 个元素是 yaw。
    hpipm_->A_stage[k].block<3,3>(0, 6) = rotz(yaw_d).transpose() * in.dt;  // <行数, 列数>(起始行, 起始列)，dt * Rz^T(yaw_d)
    hpipm_->A_stage[k].block<3,3>(3, 9) = Mat3::Identity() * in.dt;
    hpipm_->A_stage[k](11, 12) = in.dt;

    for (int leg = 0; leg < 4; ++leg) {
      const Vec3& r = in.rFeet[k][leg];     // 力臂r
      hpipm_->B_stage[k].block<3,3>(6, 3 * leg) =
          in.dt * in.Iw_inv * skew(r);
      hpipm_->B_stage[k].block<3,3>(9, 3 * leg) =
          (in.dt / mass) * Mat3::Identity();
    }
  }

  // ====== Stage cost 代价函数 ======
  //
  // 论文目标：
  // J = (X-D)^T Q (X-D) + U^T R U
  //
  // HPIPM stage cost 形式：
  // 0.5*x^T Q*x + 0.5*u^T R*u + q^T*x + r^T*u
  //
  // 因此为了等价于论文中的平方项，代码中使用 2*Q 和 2*R。

  // stage 1 ... stage N-1 有状态代价；
  // stage 0 的 x0 是已知初始状态，不作为 HPIPM 决策变量，所以没有 Q_stage[0]。
  for (int k = 1; k < horizon_N; ++k) {
    hpipm_->q_stage[k].noalias() = -2.0 * Q * in.xRef[k];
  }

  // terminal cost：终端状态 x_N 的跟踪代价,单独拿出来方便以后算法单独设置Q权重
  hpipm_->q_stage[horizon_N].noalias() = -2.0 * Q * in.xRef[horizon_N];

  // ====== Constraints: lg <= C*x + D*u <= ug ======
  // 约束下界 l_g，约束矩阵 C 和 D，约束上界 u_g
  // 每条腿 7 行：
  // 0: -fx + mu*fz
  // 1: -fy + mu*fz
  // 2:  fx + mu*fz
  // 3:  fy + mu*fz
  // 4:  fz
  // 5:  fx
  // 6:  fy


  // 设置摩擦锥的上下界矩阵ci
  for (int k = 0; k < horizon_N; ++k) {
    int stance_count = 0;
    for (int leg = 0; leg < 4; ++leg) {
      if (in.contact[k][leg] == 1) {
        ++stance_count;
      }
    }
    VecX lower_bound_stance_dynamic = lower_bound_stance_;
    lower_bound_stance_dynamic(4) = computeStanceLegFzMin(stance_count);

    for (int leg = 0; leg < 4; ++leg) {
      const int row = rows_per_leg * leg;

      if (in.contact[k][leg] == 0) {
        // swing: fx=fy=fz=0
        hpipm_->lg_stage[k].segment(row, rows_per_leg) = lower_bound_swing_;
        hpipm_->ug_stage[k].segment(row, rows_per_leg) = upper_bound_swing_;
      } else {
        // stance
        hpipm_->lg_stage[k].segment(row, rows_per_leg) = lower_bound_stance_dynamic;
        hpipm_->ug_stage[k].segment(row, rows_per_leg) = upper_bound_stance_;
      }
    }
  }

  // AA/BB/bb 是 HPIPM 对动力学矩阵的命名习惯。
  //
  // 对 k>=1：
  //   x_{k+1} = AA[k] * x_k + BB[k] * u_k + bb[k]
  //   其中 bb[k] = 0
  //
  // 对 k=0：
  //   因为 x0 是已知初始状态，不作为决策变量，
  //   所以 HPIPM stage 0 写成：
  //   x_1 = BB[0] * u_0 + bb[0]
  //
  // 这里的 bb[0] = A_0 * x_0。
  // 这不是论文外新增的物理 b_k，只是 HPIPM 在消去已知 x0 后的接口写法。
  // 因为nx[0] = 0，x0不进入优化器，所以要把x1的状态转移方程中的 A_0*x0 这部分当作常数项 bb[0] 告诉 HPIPM。
  hpipm_->bb_stage[0] = hpipm_->A_stage[0] * in.x0; // 只有bb_stage[0] 是非零的

  // 赋值第0步单独写，因为 AA[0] 不作为优化变量不能传
  hpipm_->AA[0] = &kDummyScalar;
  hpipm_->BB[0] = hpipm_->B_stage[0].data();
  hpipm_->bb[0] = hpipm_->bb_stage[0].data();

  for (int k = 1; k < horizon_N; ++k) {
    hpipm_->AA[k] = hpipm_->A_stage[k].data();
    hpipm_->BB[k] = hpipm_->B_stage[k].data();
    hpipm_->bb[k] = hpipm_->bb_stage[k].data();
  }

  // QQ/RR/SS/qq/rr 是 HPIPM 对 stage cost 的命名习惯。
  //
  // QQ[k]：状态二次项 Q。
  // RR[k]：输入二次项 R。
  // SS[k]：输入-状态交叉项 S。
  // qq[k]：状态一次项 q。
  // rr[k]：输入一次项 r。
  //
  // 论文目标：
  // J = (X-D)^T Q (X-D) + U^T R U
  //
  // HPIPM stage cost 形式：
  // 0.5*x^T Q*x + 0.5*u^T R*u + q^T*x + r^T*u
  //
  // 当前在论文目标基础上，对真正会执行的 u0 额外加入：
  // (u0 - last_u0)^T S (u0 - last_u0)，其中这个权重系数S代码中是S，不是HPIPM里的状态交叉项SS，完全不同的
  if (has_last_solution_ && !S.isZero(0)) {
    r_cost_rate_.noalias() = -2.0 * S * last_u0_;
    hpipm_->RR[0] = R_cost_rate_.data();
    hpipm_->rr[0] = r_cost_rate_.data();
  } else {
    hpipm_->RR[0] = R_cost_.data();
    hpipm_->rr[0] = r_cost_zero_.data();
  }
  hpipm_->QQ[0] = &kDummyScalar;
  hpipm_->qq[0] = &kDummyScalar;
  hpipm_->SS[0] = &kDummyScalar;

  for (int k = 1; k < horizon_N; ++k) {
    hpipm_->QQ[k] = Q_cost_.data();
    hpipm_->RR[k] = R_cost_.data();
    hpipm_->SS[k] = S_cross_cost_.data();
    hpipm_->qq[k] = hpipm_->q_stage[k].data();
    hpipm_->rr[k] = r_cost_zero_.data();
  }

  // xN不是没用，而是用来评价：最后一步控制 u(N-1) 把系统推到了哪里
  hpipm_->QQ[horizon_N] = Q_cost_.data();
  hpipm_->RR[horizon_N] = &kDummyScalar;
  hpipm_->SS[horizon_N] = &kDummyScalar;
  hpipm_->qq[horizon_N] = hpipm_->q_stage[horizon_N].data();
  hpipm_->rr[horizon_N] = &kDummyScalar;

  // CC/DD/llg/uug 是 HPIPM 对一般线性约束的命名习惯。
  //
  // 约束形式：
  // llg[k] <= CC[k]*x_k + DD[k]*u_k <= uug[k]
  //
  // 当前约束只作用于输入足底力 u，所以 CC 可以为零。
  for (int k = 0; k < horizon_N; ++k) {
    // 第一句分类讨论的意义
    // 第 0 步：C0 连尺寸都不该是 28x13，因为 nx[0]=0
    // 第 1 步以后：Ck 尺寸是 28x13，但内容全是 0
    hpipm_->CC[k] = (k == 0) ? &kDummyScalar : C_zero_constraint_.data(); // C没用到
    hpipm_->DD[k] = D_force_constraint_.data();
    hpipm_->llg[k] = hpipm_->lg_stage[k].data();
    hpipm_->uug[k] = hpipm_->ug_stage[k].data();
  }
  hpipm_->CC[horizon_N] = &kDummyScalar;
  hpipm_->DD[horizon_N] = &kDummyScalar;
  hpipm_->llg[horizon_N] = &kDummyScalar;
  hpipm_->uug[horizon_N] = &kDummyScalar;

  if (!has_logged_first_qp_structure_snapshot_) {
    const auto q_diag = Q_cost_.diagonal();
    const auto r_diag = R_cost_.diagonal();
    const auto rr0_diag =
        (has_last_solution_ && !S.isZero(0))
            ? R_cost_rate_.diagonal()
            : R_cost_.diagonal();

    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][qp] cost diag Q=(%.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f | %.3f) "
        "R0=(%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f) "
        "RR0=(%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f)",
        q_diag(0), q_diag(1), q_diag(2),
        q_diag(3), q_diag(4), q_diag(5),
        q_diag(6), q_diag(7), q_diag(8),
        q_diag(9), q_diag(10), q_diag(11),
        q_diag(12),
        r_diag(0), r_diag(1), r_diag(2), r_diag(3), r_diag(4), r_diag(5),
        r_diag(6), r_diag(7), r_diag(8), r_diag(9), r_diag(10), r_diag(11),
        rr0_diag(0), rr0_diag(1), rr0_diag(2), rr0_diag(3), rr0_diag(4), rr0_diag(5),
        rr0_diag(6), rr0_diag(7), rr0_diag(8), rr0_diag(9), rr0_diag(10), rr0_diag(11));

    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][qp] dynamics norm A0=%.6f B0=%.6f bb0=%.6f q1=%.6f qN=%.6f D=%.6f C=%.6f x0=%.6f",
        hpipm_->A_stage[0].norm(),
        hpipm_->B_stage[0].norm(),
        hpipm_->bb_stage[0].norm(),
        (horizon_N > 1) ? hpipm_->q_stage[1].norm() : 0.0,
        hpipm_->q_stage[horizon_N].norm(),
        D_force_constraint_.norm(),
        C_zero_constraint_.norm(),
        in.x0.norm());

    for (int leg = 0; leg < 4; ++leg) {
      const int row = rows_per_leg * leg;
      const auto lg = hpipm_->lg_stage[0].segment(row, rows_per_leg);
      const auto ug = hpipm_->ug_stage[0].segment(row, rows_per_leg);
      const double b_ang_norm = hpipm_->B_stage[0].block<3,3>(6, 3 * leg).norm();
      const double b_lin_norm = hpipm_->B_stage[0].block<3,3>(9, 3 * leg).norm();

      RCLCPP_INFO(
          mpcLogger(),
          "[ConvexMpcSolver][qp] k=0 leg=%d contact=%d r=(%.3f %.3f %.3f) "
          "B_ang_norm=%.6f B_lin_norm=%.6f "
          "lg=(%.3f %.3f %.3f %.3f %.3f %.3f %.3f) "
          "ug=(%.3f %.3f %.3f %.3f %.3f %.3f %.3f)",
          leg,
          in.contact[0][leg],
          in.rFeet[0][leg](0),
          in.rFeet[0][leg](1),
          in.rFeet[0][leg](2),
          b_ang_norm,
          b_lin_norm,
          lg(0), lg(1), lg(2), lg(3), lg(4), lg(5), lg(6),
          ug(0), ug(1), ug(2), ug(3), ug(4), ug(5), ug(6));
    }

    has_logged_first_qp_structure_snapshot_ = true;
  }

  // unused box/slack constraints，不用管
  std::vector<int*> hidxbx(horizon_N + 1, nullptr);
  std::vector<double*> hlbx(horizon_N + 1, nullptr);
  std::vector<double*> hubx(horizon_N + 1, nullptr);

  std::vector<int*> hidxbu(horizon_N + 1, nullptr);
  std::vector<double*> hlbu(horizon_N + 1, nullptr);
  std::vector<double*> hubu(horizon_N + 1, nullptr);

  std::vector<double*> hZl(horizon_N + 1, nullptr);
  std::vector<double*> hZu(horizon_N + 1, nullptr);
  std::vector<double*> hzl(horizon_N + 1, nullptr);
  std::vector<double*> hzu(horizon_N + 1, nullptr);

  std::vector<int*> hidxs(horizon_N + 1, nullptr);
  std::vector<double*> hlls(horizon_N + 1, nullptr);
  std::vector<double*> hlus(horizon_N + 1, nullptr);

  // 不求解，只装填，d_ocp_qp_set_all函数把前面准备好的所有 MPC 数学数据，统一写进 HPIPM 的 QP 问题对象 hpipm_->qp 里
  if (kEnableMpcStageLog) {
    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][stage] before d_ocp_qp_set_all: bb0_norm=%.6f x0_norm=%.6f contact0=[%d %d %d %d]",
        hpipm_->bb_stage[0].norm(),
        in.x0.norm(),
        in.contact[0][0],
        in.contact[0][1],
        in.contact[0][2],
        in.contact[0][3]);
  }
  d_ocp_qp_set_all(
      hpipm_->AA.data(), hpipm_->BB.data(), hpipm_->bb.data(),
      hpipm_->QQ.data(), hpipm_->SS.data(), hpipm_->RR.data(), hpipm_->qq.data(), hpipm_->rr.data(),
      hidxbx.data(), hlbx.data(), hubx.data(),
      hidxbu.data(), hlbu.data(), hubu.data(),
      hpipm_->CC.data(), hpipm_->DD.data(), hpipm_->llg.data(), hpipm_->uug.data(),
      hZl.data(), hZu.data(), hzl.data(), hzu.data(),
      hidxs.data(), hlls.data(), hlus.data(),
      &hpipm_->qp
  );
  if (kEnableMpcStageLog) {
    RCLCPP_INFO(mpcLogger(), "[ConvexMpcSolver][stage] after d_ocp_qp_set_all");
  }

  // 求解 QP 问题，得到 hpipm_->qpSol 解
  if (kEnableMpcStageLog) {
    RCLCPP_INFO(mpcLogger(), "[ConvexMpcSolver][stage] before d_ocp_qp_ipm_solve");
  }
  d_ocp_qp_ipm_solve(&hpipm_->qp, &hpipm_->qpSol, &hpipm_->arg, &hpipm_->workspace);
  if (kEnableMpcStageLog) {
    RCLCPP_INFO(mpcLogger(), "[ConvexMpcSolver][stage] after d_ocp_qp_ipm_solve");
  }

  // 从 HPIPM 的 workspace 里读取求解状态，hpipm_status == 0表示求解成功，hpipm_status < 0 表示求解失败。
  int hpipm_status = -1;
  d_ocp_qp_ipm_get_status(&hpipm_->workspace, &hpipm_status);

  if (kEnableMpcStageLog) {
    int hpipm_iter = -1;
    d_ocp_qp_ipm_get_iter(&hpipm_->workspace, &hpipm_iter);
    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][stage] after status query: status=%d iter=%d",
        hpipm_status,
        hpipm_iter);
  }

  if (hpipm_verbose) { // hpipm_verbose 默认是false
    int hpipm_iter = -1;
    d_ocp_qp_ipm_get_iter(&hpipm_->workspace, &hpipm_iter);

    std::fprintf(stderr,
                 "[ConvexMpcSolver][HPIPM] status=%d, iter=%d, N=%d, dt=%.6f\n",
                 hpipm_status,
                 hpipm_iter,
                 horizon_N,
                 in.dt);
  }

  if (hpipm_status != 0) {// 求解失败
    int hpipm_iter = -1;
    d_ocp_qp_ipm_get_iter(&hpipm_->workspace, &hpipm_iter);
    RCLCPP_WARN_THROTTLE(
        mpcLogger(),
        mpcClock(),
        500,
        "[ConvexMpcSolver] MPC not converged, status=%d iter=%d N=%d dt=%.6f",
        hpipm_status,
        hpipm_iter,
        horizon_N,
        in.dt);

    if (enableFallbackToLast && has_last_solution_ && last_u0_.allFinite()) {// 如果可以返回上一帧就返回
      out.u0 = last_u0_;
      out.success = true;
      return out;
    }

    return out;
  }

  // 从 HPIPM 解中提取第一步的控制输入 u0
  d_ocp_qp_sol_get_u(0, &hpipm_->qpSol, out.u0.data());

  out.success = out.u0.allFinite();

  if (out.success) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    const double fz_fr = out.u0(2);
    const double fz_fl = out.u0(5);
    const double fz_rr = out.u0(8);
    const double fz_rl = out.u0(11);
    const double sum_fz = fz_fr + fz_fl + fz_rr + fz_rl;

    if (!has_last_solution_) {
      RCLCPP_INFO(
          mpcLogger(),
          "[ConvexMpcSolver] MPC first successful solve: u0_norm=%.6f N=%d dt=%.6f",
          out.u0.norm(),
          horizon_N,
          in.dt);
    }
    RCLCPP_INFO_THROTTLE(
        mpcLogger(),
        steady_clock,
        250,
        "[ConvexMpcSolver] MPC u0 fz=(%.3f %.3f %.3f %.3f) sum_fz=%.3f u0_norm=%.6f",
        fz_fr,
        fz_fl,
        fz_rr,
        fz_rl,
        sum_fz,
        out.u0.norm());
    last_u0_ = out.u0;
    has_last_solution_ = true;
  }

  return out;
}

Vec34 ConvexMpcSolver::solveFromDogWrench(
    const Vec3& dd_pcd_G,        // 期望质心/机身线加速度，G 系表达。用于生成参考位置轨迹 p_ref = p_now + v_ref*t + 0.5*a_ref*t^2。
    const Vec34& foot_hold_G,    // 四条腿最近一次落地/进入支撑时记录的足底固定接触点，G 系表达。每一列对应一条腿的足底世界坐标。
    const Vec34& foot_end_G,     // 在摆动腿时，预期足底支撑落点
    const VecInt4& contact_now,  // 当前四条腿接触状态。1 表示支撑腿，0 表示摆动腿。顺序为 FR, FL, RR, RL。
    const Vec4& phase_now,       // 当前四条腿在各自支撑相/摆动相内部的归一化进度，范围 [0,1]。注意不是完整步态周期相位。
    const double control_dt,     // 当前主控制周期，单位秒。来自 ros2_control 的 period.seconds()，用于离散化动力学。
    const double gait_period,    // 当前完整步态周期，单位秒。例如 trot 一个完整周期的时间。
    const double stance_ratio,   // 支撑相占完整步态周期的比例。stance_ratio = 支撑相时间 / 完整步态周期。
    const Vec3& p_body_G,        // 当前机身原点位置，G 系表达。后面会结合 pcb_B 转成质心位置 p_com_G。
    const Vec3& v_body_G,        // 当前机身原点线速度，G 系表达。后面会结合质心偏置和角速度转成质心速度 v_com_G。
    const RotMat& R_GB,          // 当前机身姿态旋转矩阵，表示从 B 系到 G 系的旋转。用于计算 RPY、质心偏置、世界系惯量。
    const Vec3& gyro_G,          // 当前机身角速度，G 系表达。对应论文状态里的 omega。
    const RotMat& Rd_GB,         // 期望机身姿态旋转矩阵，B 系到 G 系。用于生成参考姿态 theta_ref。
    const Vec3& v_ref_G          // 期望机身/质心线速度，G 系表达。用于生成参考速度和参考位置轨迹。
    ) {
  Vec3 dd_pcd_clamped = dd_pcd_G;
  dd_pcd_clamped(0) = saturation(dd_pcd_clamped(0), Vec2(-1.0, 1.0));
  dd_pcd_clamped(1) = saturation(dd_pcd_clamped(1), Vec2(-1.0, 1.0));
  dd_pcd_clamped(2) = saturation(dd_pcd_clamped(2), Vec2(-5.0, 5.0));

  if (kEnableMpcStageLog) {
    RCLCPP_INFO_THROTTLE(
        mpcLogger(),
        mpcClock(),
        500,
        "[ConvexMpcSolver][stage] solveFromDogWrench enter: dt=%.6f gait_period=%.6f stance_ratio=%.3f contact=[%d %d %d %d]",
        control_dt,
        gait_period,
        stance_ratio,
        contact_now(0),
        contact_now(1),
        contact_now(2),
        contact_now(3));
  }

  if (control_dt <= 0.0 || !std::isfinite(control_dt)) {  // 控制周期必须正数且有限
    logFallbackReason("invalid control_dt");
    return makeFallbackForces(contact_now);
  }

  if (gait_period <= 0.0 || !std::isfinite(gait_period)) { // 步态周期必须正数且有限
    logFallbackReason("invalid gait_period");
    return makeFallbackForces(contact_now);
  }

  if (stance_ratio <= 0.0 || stance_ratio >= 1.0 || !std::isfinite(stance_ratio)) {
    logFallbackReason("invalid stance_ratio");
    return makeFallbackForces(contact_now);   // 支撑比必须在 (0,1) 之间且有限
  }

  if (N <= 0 ||
      mass <= 0.0 ||
      !std::isfinite(mass) ||
      !std::isfinite(mu) ||
      !std::isfinite(fzMin) ||
      !std::isfinite(fzMax) ||
      mu <= 0.0 ||
      fzMax < fzMin ||
      !Ib.allFinite() ||
      !pcb_B.allFinite() ||
      !g.allFinite()) {
    logFallbackReason("invalid solver config");
    return makeFallbackForces(contact_now);
  }

  if (!dd_pcd_G.allFinite() ||
      !foot_hold_G.allFinite() ||
      !foot_end_G.allFinite() ||
      !phase_now.allFinite() ||
      !p_body_G.allFinite() ||
      !v_body_G.allFinite() ||
      !R_GB.allFinite() ||
      !gyro_G.allFinite() ||
      !Rd_GB.allFinite() ||
      !v_ref_G.allFinite()) {
    logFallbackReason("non-finite input");
    return makeFallbackForces(contact_now);
  }

  for (int leg = 0; leg < 4; ++leg) {
    if (contact_now(leg) != 0 && contact_now(leg) != 1) {
      logFallbackReason("invalid contact_now");
      return makeFallbackForces(contact_now);
    }
  }

  ConvexMpcInput in;
  in.dt = control_dt;

  in.N = computeEffectiveHorizon(
      N,
      in.dt,
      gait_period,
      enforceHalfGaitHorizon
  );

  if (in.N <= 0) {
    logFallbackReason("effective horizon <= 0");
    return makeFallbackForces(contact_now);
  }

  if (kEnableMpcStageLog) {
    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][stage] horizon ready: N=%d dt=%.6f dd_pcd=(%.3f %.3f %.3f)",
        in.N,
        in.dt,
        dd_pcd_clamped(0),
        dd_pcd_clamped(1),
        dd_pcd_clamped(2));
  }

  // ====== x0 = [Theta, p_com, omega, v_com, g_z] ======
  //
  // 论文状态顺序：
  // x = [Theta(3), p_com(3), omega(3), v_com(3), g_z(1)]

  const Vec3 theta_now = rotMatToRPY(R_GB);
  const Vec3 p_com_G = p_body_G + R_GB * pcb_B; // 将标准中心位置加上质心位置偏移
  const Vec3 v_com_G = v_body_G + gyro_G.cross(R_GB * pcb_B); // cross()是 Eigen 的叉乘函数，也是质心位置补偿

  in.x0.setZero();
  in.x0.segment<3>(0) = theta_now;
  in.x0.segment<3>(3) = p_com_G;
  in.x0.segment<3>(6) = gyro_G;
  in.x0.segment<3>(9) = v_com_G;
  in.x0(12) = g(2);

  // ====== xRef (N + 1) ======
  in.xRef.resize(in.N + 1);

  const Vec3 theta_ref = rotMatToRPY(Rd_GB);

  for (int k = 0; k <= in.N; ++k) {
    const double t_k = static_cast<double>(k) * in.dt;

    in.xRef[k].setZero();

    in.xRef[k].segment<3>(0) = theta_ref;

    // 参考位置轨迹：p_ref = p_now + v_ref*t + 0.5*a_ref*t^2
    in.xRef[k].segment<3>(3) <<
        p_com_G(0) + v_ref_G(0) * t_k + 0.5 * dd_pcd_clamped(0) * t_k * t_k,
        p_com_G(1) + v_ref_G(1) * t_k + 0.5 * dd_pcd_clamped(1) * t_k * t_k,
        p_com_G(2) + v_ref_G(2) * t_k + 0.5 * dd_pcd_clamped(2) * t_k * t_k;

    in.xRef[k].segment<3>(6) << 0.0, 0.0, 0.0;

    // 参考速度：当前先保持 v_ref_G，不额外积分 dd_pcd_G
    in.xRef[k].segment<3>(9) = v_ref_G;

    in.xRef[k](12) = g(2);
  }

  // ====== contact schedule (N) ======
  //
  // 使用当前每条腿的 phase_now 和 contact_now 预测未来接触状态。
  //
  // WaveGenerator 的 phase_now 是当前支撑相/摆动相内部进度：
  // contact=1: normal_phase = phase_now * stance_ratio
  // contact=0: normal_phase = stance_ratio + phase_now * (1 - stance_ratio)

  in.contact.resize(in.N);

  // normal_phase_now是一个完整步态周期内的相位，范围[0,1)，先支撑再摆动
  Vec4 normal_phase_now;
  normal_phase_now.setZero();

  for (int leg = 0; leg < 4; ++leg) {
    normal_phase_now(leg) =
        legModePhaseToCyclePhase(
            phase_now(leg),
            contact_now(leg),
            stance_ratio  // stance_ratio是1个步态周期内支撑相的占比
        );
  }
  /*  
  * 根据normal_phase_now，
  * 预测 MPC horizon 内每一个未来时刻 k，四条腿分别是支撑还是摆动，
  * 并写入 in.contact[k]
  */
  for (int k = 0; k < in.N; ++k) {
    const double t_k = static_cast<double>(k) * in.dt;

    std::array<int, 4> ck;
    for (int leg = 0; leg < 4; ++leg) {
      const double normal_k =
          wrap01(normal_phase_now(leg) + t_k / gait_period);  // 相位 wrap 到 [0,1)

      ck[leg] = (normal_k < stance_ratio) ? 1 : 0;
    }

    in.contact[k] = ck;
  }

  // ====== rFeet schedule (N) ======
  //
  // foot_hold_G 由 StateTrotting 传入：
  // foot_hold_G.col(i) = 第 i 条腿最近一次落地/支撑时记录的 G 系足底接触点。
  //
  // 这里不做任何足端正解，也不重新做 body->foot 到 world 的坐标变换。
  // MPC 这里只需要计算 SRBD 转动动力学里的力臂：
  //
  // r_i(k) = p_foot_hold_i - p_com_ref(k)
  //
  // 其中：
  // p_foot_hold_i 是外部已经记录好的 G 系固定接触点；
  // p_com_ref(k) 是第 k 个预测步的参考 COM 位置。

  in.rFeet.resize(in.N);

  const Mat3 Iw = R_GB * Ib * R_GB.transpose(); // 计算G系下的惯性矩阵
  const Eigen::LDLT<Mat3> ldlt(Iw); // LDLT 是一种矩阵分解方法

  if (ldlt.info() == Eigen::Success) {
    in.Iw_inv = ldlt.solve(Mat3::Identity()); // 求逆矩阵
  } else {
    logFallbackReason("Iw LDLT failed");
    return makeFallbackForces(contact_now);
  }

  if (!in.Iw_inv.allFinite()) {
    logFallbackReason("Iw_inv non-finite");
    return makeFallbackForces(contact_now);
  }

  // 构造 MPC 预测时域内每一步的 足端力臂 rFeet
  for (int k = 0; k < in.N; ++k) {
    std::array<Vec3, 4> rk;

    const Vec3 p_com_ref_G = in.xRef[k].segment<3>(3);

    for (int leg = 0; leg < 4; ++leg) {
      if (contact_now(leg) == 1) {
        rk[leg] = foot_hold_G.col(leg) - p_com_ref_G;
      } else {
        rk[leg] = foot_end_G.col(leg) - p_com_ref_G;
      }
    }

    in.rFeet[k] = rk;
  }

  if (!has_logged_first_trot_mpc_snapshot_) {
    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][inspect] x0: theta=(%.3f %.3f %.3f) p=(%.3f %.3f %.3f) omega=(%.3f %.3f %.3f) v=(%.3f %.3f %.3f) gz=%.3f",
        in.x0(0), in.x0(1), in.x0(2),
        in.x0(3), in.x0(4), in.x0(5),
        in.x0(6), in.x0(7), in.x0(8),
        in.x0(9), in.x0(10), in.x0(11),
        in.x0(12));
    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][inspect] xRef0: theta=(%.3f %.3f %.3f) p=(%.3f %.3f %.3f) omega=(%.3f %.3f %.3f) v=(%.3f %.3f %.3f) gz=%.3f",
        in.xRef[0](0), in.xRef[0](1), in.xRef[0](2),
        in.xRef[0](3), in.xRef[0](4), in.xRef[0](5),
        in.xRef[0](6), in.xRef[0](7), in.xRef[0](8),
        in.xRef[0](9), in.xRef[0](10), in.xRef[0](11),
        in.xRef[0](12));
    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][inspect] xRef1: theta=(%.3f %.3f %.3f) p=(%.3f %.3f %.3f) omega=(%.3f %.3f %.3f) v=(%.3f %.3f %.3f) gz=%.3f",
        in.xRef[1](0), in.xRef[1](1), in.xRef[1](2),
        in.xRef[1](3), in.xRef[1](4), in.xRef[1](5),
        in.xRef[1](6), in.xRef[1](7), in.xRef[1](8),
        in.xRef[1](9), in.xRef[1](10), in.xRef[1](11),
        in.xRef[1](12));
    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][inspect] Ib diag=(%.3f %.3f %.3f) Iw diag=(%.3f %.3f %.3f) Iw_inv diag=(%.3f %.3f %.3f)",
        Ib(0, 0), Ib(1, 1), Ib(2, 2),
        Iw(0, 0), Iw(1, 1), Iw(2, 2),
        in.Iw_inv(0, 0), in.Iw_inv(1, 1), in.Iw_inv(2, 2));
    for (int leg = 0; leg < 4; ++leg) {
      RCLCPP_INFO(
          mpcLogger(),
          "[ConvexMpcSolver][inspect] k=0 leg=%d contact_now=%d contact_pred=%d rFeet=(%.3f %.3f %.3f)",
          leg,
          contact_now(leg),
          in.contact[0][leg],
          in.rFeet[0][leg](0),
          in.rFeet[0][leg](1),
          in.rFeet[0][leg](2));
    }
    has_logged_first_trot_mpc_snapshot_ = true;
  }

  if (kEnableMpcStageLog) {
    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][stage] input ready: x0=(roll=%.3f pitch=%.3f yaw=%.3f z=%.3f gz=%.3f) r0_leg0=(%.3f %.3f %.3f)",
        in.x0(0),
        in.x0(1),
        in.x0(2),
        in.x0(5),
        in.x0(12),
        in.rFeet[0][0](0),
        in.rFeet[0][0](1),
        in.rFeet[0][0](2));
  }

  // ====== solve MPC ======
  if (kEnableMpcStageLog) {
    RCLCPP_INFO(mpcLogger(), "[ConvexMpcSolver][stage] before solveMpc");
  }
  const ConvexMpcOutput out = solveMpc(in);
  if (kEnableMpcStageLog) {
    RCLCPP_INFO(
        mpcLogger(),
        "[ConvexMpcSolver][stage] after solveMpc: success=%d u0_norm=%.6f",
        out.success ? 1 : 0,
        out.u0.norm());
  }

  if (!out.success) {
    logFallbackReason("solveMpc returned failure");
    return makeFallbackForces(contact_now);
  }

  Vec34 force_feet_P;

  for (int leg = 0; leg < 4; ++leg) {
    force_feet_P(0, leg) = out.u0(3 * leg + 0);
    force_feet_P(1, leg) = out.u0(3 * leg + 1);
    force_feet_P(2, leg) = out.u0(3 * leg + 2);
  }

  return force_feet_P;
}
