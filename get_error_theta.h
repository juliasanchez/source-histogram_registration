#ifndef GET_ERROR_THETA
#define GET_ERROR_THETA

#include <iostream>
#include <string>

void get_error_phi(std::vector<std::vector<float>>& hist1, std::vector<std::vector<float>>& hist2, std::vector<float>& error_theta, int *rotation_theta);

#include "get_error_theta.inl"

#endif // GET_ERROR_THETA
