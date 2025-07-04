#include <iostream>
#include "armadillo"
#include "KalmanFilter.h"
#include "json.hpp"
#include "sensor.h"
// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
int main() {

    // TIP Press <shortcut actionId="RenameElement"/> when your caret is at the <b>lang</b> variable name to see how CLion can help you rename it.

    arma::mat A(4, 4, arma::fill::eye);
    arma::mat B(4, 4, arma::fill::randu);

    arma::vec x0(4, arma::fill::ones);
    arma::mat Sigma0(4, 4, arma::fill::eye);

    std::cout << A*B.t() << std::endl;
    std::cout << A << std::endl;
    std::cout << B << std::endl;

    return EXIT_SUCCESS;
    // TIP See CLion help at <a href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>. Also, you can try interactive lessons for CLion by selecting 'Help | Learn IDE Features' from the main menu.
}