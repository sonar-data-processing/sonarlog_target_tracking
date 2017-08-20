#include <cstdlib>
#include "Common.hpp"
#include "ROC.hpp"

namespace sonarlog_target_tracking
{

ROC::ROC(
    const std::vector<double>& true_positive_rate,
    const std::vector<double>& false_positive_rate)
{
    Calculate(true_positive_rate, false_positive_rate);
}

ROC::~ROC()
{
}

void ROC::Calculate(
    const std::vector<double>& true_positive_rate,
    const std::vector<double>& false_positive_rate)
{
    size_t total_elements = false_positive_rate.size();
    std::vector<size_t> indices(total_elements);
    for (size_t i = 0; i < indices.size(); i++) indices[i]=i;
    std::sort(indices.begin(), indices.end(), common::IndexComparator<double>(false_positive_rate));

    for (size_t i = 0; i < indices.size(); i++) {
        std::cout << false_positive_rate[indices[i]] << std::endl;
    }
}

} // namespace sonarlog_target_tracking
