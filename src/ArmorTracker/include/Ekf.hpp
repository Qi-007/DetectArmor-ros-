#ifndef EKF_HPP
#define EKF_HPP

#include <eigen3/Eigen/Core>
#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

// S_NUM 状态维度    M_NUM 观测维度
template<typename T = double, int S_NUM = 6, int M_NUM = 2>
class Ekf {
public:
    // typedef Eigen::Matrix<double, S_NUM, S_NUM> F;
    // typedef Eigen::Matrix<double, M_NUM, S_NUM> H;
    // typedef Eigen::Matrix<double, S_NUM, S_NUM> FJacobi;
    // typedef Eigen::Matrix<double, M_NUM, S_NUM> HJacobi;
    // typedef Eigen::Matrix<double, S_NUM, S_NUM> Q;
    // typedef Eigen::Matrix<double, M_NUM, M_NUM> R;
    // typedef Eigen::Matrix<double, S_NUM, S_NUM> P;
    // typedef Eigen::Matrix<double, S_NUM, 1> State;      
    // typedef Eigen::Matrix<double, M_NUM, 1> Measurement;
    // typedef Eigen::Matrix<double, S_NUM, M_NUM> K;

    using F = Eigen::Matrix<double, S_NUM, S_NUM>;
    using H = Eigen::Matrix<double, M_NUM, S_NUM>;
    using FJacobi = Eigen::Matrix<double, S_NUM, S_NUM>;
    using HJacobi = Eigen::Matrix<double, M_NUM, S_NUM>;
    using Q = Eigen::Matrix<double, S_NUM, S_NUM>;
    using R = Eigen::Matrix<double, M_NUM, M_NUM>;
    using P = Eigen::Matrix<double, S_NUM, S_NUM>;
    using State = Eigen::Matrix<double, S_NUM, 1>;
    using Measurement = Eigen::Matrix<double, M_NUM, 1>;
    using K = Eigen::Matrix<double, S_NUM, M_NUM>;

    Ekf() = default;  // 默认构造函数

    Ekf(F f, H h, Q q, R r, P p):
        m_f(f),
        m_h(h),
        // m_f_jacobi(f_j),
        // m_h_jacobi(h_j),
        m_q(q),
        m_r(r),
        m_p(p){
            std::cout << "init successfully" << std::endl;
    }

    void initState(State init_state) {
        assert(init_state.rows() == S_NUM && init_state.cols() == 1);
        m_state = init_state;
    }

    void predict(T dt) {
        updateF(dt);
        updateQ(dt);
        
        // 1.
        m_pre_state = m_f * m_state;

        // 2.
        m_pre_p = m_f * m_p * m_f.transpose() + m_q;
    }

    void update(Measurement measurement) {
        if (!std::isfinite(measurement(0)) || !std::isfinite(measurement(1))) {
            std::cout << "Measurement contains NaN or Inf!" << std::endl;
            return;
        }

        // 3.
        // m_k =  m_pre_p * m_h.transpose() * ((m_h * m_pre_p * m_h.transpose() + m_r).inverse());
        Eigen::Matrix<double, M_NUM, M_NUM> S = m_h * m_pre_p * m_h.transpose() + m_r; // 固定大小矩阵
        Eigen::LLT<Eigen::Matrix<double, M_NUM, M_NUM>> llt(S);                       // LLT 分解
        m_k = m_pre_p * m_h.transpose() * llt.solve(Eigen::Matrix<double, M_NUM, M_NUM>::Identity());

        // 4.
        m_state = m_pre_state + m_k * (measurement - m_h * (m_pre_state));
        // 5.
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_p.rows(), m_p.cols());
        m_p = (I - m_k * m_h) * m_pre_p;
    }

    void setF(const Eigen::Matrix<double, S_NUM, S_NUM> &new_F) {
        // std::cout << "Updated F matrix:\n" << new_F << std::endl;
        m_f = new_F;
    }

    State getPreState() {return m_pre_state;}
    State getState() {return m_state;}
    P getP() {return m_p;}
    virtual ~Ekf() {
        std::cout << "Ekf<" << S_NUM << ", " << M_NUM << "> Destructor" << std::endl;
    }

private:
    F m_f;
    H m_h;
    FJacobi m_f_jacobi;
    HJacobi m_h_jacobi;
    Q m_q;
    R m_r;
    P m_p;
    P m_pre_p;
    State m_state;
    State m_pre_state;
    K m_k;
    Measurement m_measurement;

    void updateF(T dt) {
        m_f << 1, 0, dt, 0, 0.5 * dt * dt, 0,
               0, 1, 0, dt, 0, 0.5 * dt * dt,
               0, 0, 1, 0, dt, 0,
               0, 0, 0, 1, 0, dt,
               0, 0, 0, 0, 1, 0,
               0, 0, 0, 0, 0, 1;
    }

    void updateQ(T dt) {
        T q = 1e-2;  // 过程噪声因子
        m_q << q * pow(dt, 4) / 4, 0.0, q * pow(dt, 3) / 2, 0.0, q * pow(dt, 2) / 2, 0.0,
               0.0, q * pow(dt, 4) / 4, 0.0, q * pow(dt, 3) / 2, 0.0, q * pow(dt, 2) / 2,
               q * pow(dt, 3) / 2, 0.0, q * pow(dt, 2), 0.0, q * dt, 0.0,
               0.0, q * pow(dt, 3) / 2, 0.0, q * pow(dt, 2), 0.0, q * dt,
               q * pow(dt, 2) / 2, 0.0, q * dt, 0.0, q, 0.0,
               0.0, q * pow(dt, 2) / 2, 0.0, q * dt, 0.0, q;
    }
};

#endif