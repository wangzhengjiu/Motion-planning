#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>

using namespace Eigen;
using namespace std;

// 车辆动力学模型
class VehicleModel {
public:
    // 状态向量：[x, y, theta, v]
    // 控制输入向量：[delta, a]
    Vector4d update(const Vector4d& state, const Vector2d& control, double dt) {
        Vector4d new_state;
        double L = 2.5; // 车辆轴距
        double max_steering_angle = 0.5; // 最大转角

        new_state(0) = state(0) + state(3) * cos(state(2)) * dt;
        new_state(1) = state(1) + state(3) * sin(state(2)) * dt;
        new_state(2) = state(2) + (state(3) / L) * tan(control(0)) * dt;
        new_state(3) = state(3) + control(1) * dt;

        // 限制转角范围
        new_state(2) = max(-max_steering_angle, min(max_steering_angle, new_state(2)));

        return new_state;
    }
};

// 轨迹生成
class TrajectoryGenerator {
public:
    // 生成一段简单的直线轨迹
    static std::vector<Vector2d> generateTrajectory(double length, double dt) {
        std::vector<Vector2d> trajectory;
        for (double t = 0; t <= length; t += dt) {
            trajectory.push_back(Vector2d(t, 0.5 * sin(t))); // 曲线轨迹
        }
        return trajectory;
    }
};

// MPC控制器
class MPC {
public:
    MPC(double dt, int horizon)
        : dt(dt), horizon(horizon), state_size(4), control_size(2) {
        Q = MatrixXd::Identity(state_size, state_size);  // 状态权重矩阵
        R = MatrixXd::Identity(control_size, control_size);  // 控制权重矩阵
    }

    VectorXd solve(const Vector4d& x0, const std::vector<Vector2d>& reference_trajectory) {
        MatrixXd H = buildCostMatrix(reference_trajectory);  // 构建代价矩阵
        VectorXd g = buildCostVector(x0, reference_trajectory);  // 构建代价向量
        MatrixXd Aeq = buildEqualityMatrix();  // 构建等式约束矩阵
        VectorXd beq = buildEqualityVector(x0);  // 构建等式约束向量

        // 使用osqp-eigen库求解二次规划问题
        VectorXd u = solveQP(H, g, Aeq, beq);

        return u;
    }
    VehicleModel model;

private:
    double dt;
    int horizon;
    int state_size;
    int control_size;
    MatrixXd Q;
    MatrixXd R;

    VectorXd solveQP(const MatrixXd& H, const VectorXd& g, const MatrixXd& Aeq, const VectorXd& beq) {
        OsqpEigen::Solver solver;

        // 定义OSQP问题
        solver.settings()->setVerbosity(false);

        // 设置QP问题
        solver.data()->setNumberOfVariables(H.cols());
        solver.data()->setNumberOfConstraints(Aeq.rows());
        if(!solver.data()->setGradient(g)) return 1;
        if(!solver.data()->setLinearConstraintMatrix(beq)) return 1;
        // if(!solver.data()->setLowerBound(lowerBound)) return 1;
        // if(!solver.data()->setUpperBound(upperBound)) return 1;

        // 求解QP问题
        solver.initSolver();
        solver.solve();

        // 获取最优解
        VectorXd u = solver.getSolution();

        return u;
    }

    MatrixXd buildCostMatrix(const std::vector<Vector2d>& reference_trajectory) {
        // 构建代价矩阵
        MatrixXd H = MatrixXd::Zero(horizon * control_size, horizon * control_size);
        for (int i = 0; i < horizon; ++i) {
            H.block(i * control_size, i * control_size, control_size, control_size) = R;
        }
        for (int i = 0; i < horizon - 1; ++i) {
            Vector2d error = reference_trajectory[i + 1] - reference_trajectory[i];
            H.block((i + 1) * control_size, i * control_size, control_size, control_size) =
                Q * error * error.transpose();
        }
        return H;
    }

    VectorXd buildCostVector(const Vector4d& x0, const std::vector<Vector2d>& reference_trajectory) {
        // 构建代价向量
        VectorXd g = VectorXd::Zero(horizon * control_size);
        for (int i = 0; i < horizon - 1; ++i) {
            Vector2d error = reference_trajectory[i + 1] - reference_trajectory[i];
            g.segment((i + 1) * control_size, control_size) =
                -2.0 * Q * error * error.transpose() * model.update(x0, g.segment(i * control_size, control_size), dt);
        }
        return g;
    }

    MatrixXd buildEqualityMatrix() {
        // 构建等式约束矩阵
        MatrixXd Aeq = MatrixXd::Zero(horizon * state_size, horizon * control_size);
        for (int i = 0; i < horizon; ++i) {
            Aeq.block(i * state_size, i * control_size, state_size, control_size) =
                model.update(MatrixXd::Zero(4), Vector2d::Zero(), dt).transpose();
        }
        return Aeq;
    }

    VectorXd buildEqualityVector(const Vector4d& x0) {
        // 构建等式约束向量
        VectorXd beq = VectorXd::Zero(horizon * state_size);
        for (int i = 0; i < horizon; ++i) {
            beq.segment(i * state_size, state_size) = model.update(x0, Vector2d::Zero(), dt);
        }
        return beq;
    }
};

int main() {

    double dt = 0.1;
    int horizon = 10; 


    MPC mpc(dt, horizon);


    Vector4d x0;
    x0 << 0, 0, 0, 5;  // 初始状态：[x, y, theta,
    // 生成参考轨迹
    std::vector<Vector2d> reference_trajectory = TrajectoryGenerator::generateTrajectory(10.0, dt);

    // 迭代预测与控制
    int num_iterations = 50;
    for (int i = 0; i < num_iterations; ++i) {
        // 使用MPC求解控制输入
        VectorXd u = mpc.solve(x0, reference_trajectory);

        // 更新车辆状态
        x0 = mpc.model.update(x0, u.head<2>(), dt);

        // 打印结果
        cout << "Iteration: " << i + 1 << ", State: " << x0.transpose() << ", Control Input: " << u.transpose() << endl;
    }

    return 0;
}
