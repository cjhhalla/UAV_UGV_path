#include <Eigen/Dense>
#include <Eigen/QR>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

void saveMatrixToCSV(const std::string& filename, const Eigen::MatrixXd& matrix) {
    std::ofstream file(filename);
    
    if (file.is_open()) {
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                file << matrix(i, j);
                if (j != matrix.cols() - 1) {
                    file << ",";
                }
            }
            file << "\n";
        }
        file.close();
    } else {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
    }
}

Eigen::MatrixXd matrixPower(Eigen::MatrixXd A, int power) {
    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(A.rows(), A.cols());
    for (int i = 0; i < power; ++i) {
        result = result * A;
    }
    return result;
}

Eigen::MatrixXd trajectory_gen(Eigen::VectorXd x_0, Eigen::MatrixXd wp, std::vector<double> tk) {
    double T = tk.back(); // Total time
    double delt = 0.1; // Time interval
    int n = static_cast<int>(T/delt); // Number of timesteps

    double gamma = .05; // Drag coefficient

    // Define coefficient matrices A, B, C, G, and H
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3, 6);

    // Initialize matrices A, B, C
    A.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3, 3);
    A.block<3,3>(0,3) = Eigen::MatrixXd::Identity(3, 3) * (1-gamma*delt/2) * delt;
    A.block<3,3>(3,3) = Eigen::MatrixXd::Identity(3, 3) * (1 - gamma*delt);
    B.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3, 3) * (delt*delt/2);
    B.block<3,3>(3,0) = Eigen::MatrixXd::Identity(3, 3) * delt;
    C.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3, 3);

    // Convert tk to timestep indices
    for (double &time : tk) {
        time = std::floor(time/delt) - 1;
    }

    int K = wp.cols(); // Number of waypoints

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(3*n, 6);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3*n, 3*n);
    Eigen::MatrixXd H_first = Eigen::MatrixXd::Zero(3*n, 3);

    // Fill matrices G, H, and H_first
    for (int i = 0; i < n; ++i) {
        // G.block(3*i, 0, 3, 6) = C * A.pow(i+1);
        G.block(3*i, 0, 3, 6) = C * matrixPower(A,i+1);
        // H_first.block(3*i, 0, 3, 3) = C * A.pow(i) * B;
        H_first.block(3*i, 0, 3, 3) = C * matrixPower(A,i) * B;
    }
    for (int i = 0; i < n; ++i) {
        H.block(3*i, 3*i, 3*(n-i), 3) = H_first.topRows(3*(n-i));
    }

    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3*K, 3*n);
    for (int k = 0; k < K; ++k) {
        int index = static_cast<int>(tk[k]);
        S.block(3*k, 3*index, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
    }

    // Solve optimization problem
    Eigen::VectorXd wp_flat = Eigen::Map<Eigen::VectorXd>(wp.data(), wp.size());
    Eigen::VectorXd u_hat = (S * H).colPivHouseholderQr().solve(wp_flat - S * (G * x_0));
    Eigen::MatrixXd u_opt = Eigen::Map<Eigen::MatrixXd>(u_hat.data(), 3, n);

    // Compute trajectory
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(6, n+1);
    x.col(0) = x_0;
    for (int t = 0; t < n; ++t) {
        x.col(t+1) = A * x.col(t) + B * u_opt.col(t);
    }

    return x;

}
int main()
{
    Eigen::VectorXd x_0(6);
    x_0 << 0,0,0,0,0,0;
    Eigen::MatrixXd wp(3,4);
    wp << 10 + x_0[0], 20 + x_0[0], 40 + x_0[0], 50 + x_0[0],
        5 + x_0[1], -10 + x_0[1], 10 + x_0[1], 10 + x_0[1],
        6 + x_0[2], 7 + x_0[2], 9 + x_0[2], 10 + x_0[2];

    std::vector<double> tk = {10,15,20,30};

    Eigen::MatrixXd a =trajectory_gen(x_0,wp,tk);
    saveMatrixToCSV("/workspace/tra.csv",a);
}